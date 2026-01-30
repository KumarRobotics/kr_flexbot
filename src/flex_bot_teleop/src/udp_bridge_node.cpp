#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;

static inline double radps_to_rpm(double rad_s) {
  return rad_s * 60.0 / (2.0 * M_PI);
}
static inline double rpm_to_radps(double rpm) {
  return rpm * (2.0 * M_PI) / 60.0;
}

#pragma pack(push, 1)
struct CmdPacket {
  float left_rpm;
  float right_rpm;
};
struct RpmFeedbackPacket {
  float left_rpm;
  float right_rpm;
  double timestamp;
};
#pragma pack(pop)

class FlexBotUdpBridge : public rclcpp::Node {
public:
  FlexBotUdpBridge() : Node("flex_bot_udp_bridge") {
    imx7_ip_     = declare_parameter<std::string>("imx7_ip", "192.168.1.50");
    cmd_port_    = declare_parameter<int>("cmd_port", 5001);
    fb_port_     = declare_parameter<int>("fb_port", 5002);

    cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 50.0);

    // If true: ROS cmd topics are rad/s (teleop publishes rad/s) -> convert to rpm for UDP.
    // If false: ROS cmd topics are already rpm -> send directly.
    cmds_are_radps_ = declare_parameter<bool>("cmds_are_radps", true);

    // If true: publish feedback as rad/s topics (optional convenience)
    publish_feedback_radps_ = declare_parameter<bool>("publish_feedback_radps", true);

    auto qos = rclcpp::QoS(10).best_effort();

    // Subscribe commands
    sub_left_cmd_ = create_subscription<std_msgs::msg::Float64>(
      "/left_wheel/cmd_vel", qos,
      [this](const std_msgs::msg::Float64 &m){
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        left_cmd_ = m.data;
      });

    sub_right_cmd_ = create_subscription<std_msgs::msg::Float64>(
      "/right_wheel/cmd_vel", qos,
      [this](const std_msgs::msg::Float64 &m){
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        right_cmd_ = m.data;
      });

    // Publish feedback (rpm always)
    pub_left_rpm_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/vel_rpm", qos);
    pub_right_rpm_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/vel_rpm", qos);

    // Optional: publish rad/s too (helps drop-in with controllers expecting rad/s)
    if (publish_feedback_radps_) {
      pub_left_radps_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/vel_radps", qos);
      pub_right_radps_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/vel_radps", qos);
    }

    setup_udp();

    running_.store(true);
    rx_thread_ = std::thread([this]{ this->rx_loop(); });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
    cmd_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]{ this->send_cmd_once(); });

    RCLCPP_INFO(get_logger(),
      "UDP bridge: send cmd -> %s:%d, recv fb <- :%d (cmds_are_radps=%s)",
      imx7_ip_.c_str(), cmd_port_, fb_port_, cmds_are_radps_ ? "true":"false");
  }

  ~FlexBotUdpBridge() override {
    running_.store(false);
    if (rx_thread_.joinable()) rx_thread_.join();
    if (sock_tx_ >= 0) ::close(sock_tx_);
    if (sock_rx_ >= 0) ::close(sock_rx_);
  }

private:
  void setup_udp() {
    // TX
    sock_tx_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_tx_ < 0) throw std::runtime_error("Failed to create TX socket");

    std::memset(&imx7_addr_, 0, sizeof(imx7_addr_));
    imx7_addr_.sin_family = AF_INET;
    imx7_addr_.sin_port   = htons(cmd_port_);
    if (::inet_pton(AF_INET, imx7_ip_.c_str(), &imx7_addr_.sin_addr) != 1) {
      throw std::runtime_error("inet_pton failed for imx7_ip");
    }

    // RX bind
    sock_rx_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_rx_ < 0) throw std::runtime_error("Failed to create RX socket");

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port = htons(fb_port_);

    int reuse = 1;
    ::setsockopt(sock_rx_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (::bind(sock_rx_, (sockaddr*)&local, sizeof(local)) < 0) {
      throw std::runtime_error("bind() failed on fb_port");
    }

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 20000;
    ::setsockopt(sock_rx_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  void send_cmd_once() {
    double l = 0.0, r = 0.0;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      l = left_cmd_;
      r = right_cmd_;
    }

    CmdPacket pkt{};
    if (cmds_are_radps_) {
      pkt.left_rpm  = static_cast<float>(radps_to_rpm(l));
      pkt.right_rpm = static_cast<float>(radps_to_rpm(r));
    } else {
      pkt.left_rpm  = static_cast<float>(l);
      pkt.right_rpm = static_cast<float>(r);
    }

    ::sendto(sock_tx_, &pkt, sizeof(pkt), 0,
             (sockaddr*)&imx7_addr_, sizeof(imx7_addr_));
  }

  void rx_loop() {
    RpmFeedbackPacket pkt{};
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);

    while (running_.load()) {
      const ssize_t n = ::recvfrom(sock_rx_, &pkt, sizeof(pkt), 0, (sockaddr*)&src, &srclen);
      if (n < 0) continue;
      if (n != (ssize_t)sizeof(pkt)) continue;

      // Publish RPM
      std_msgs::msg::Float64 m;
      m.data = (double)pkt.left_rpm;  pub_left_rpm_->publish(m);
      m.data = (double)pkt.right_rpm; pub_right_rpm_->publish(m);

      // Optionally publish rad/s
      if (publish_feedback_radps_) {
        m.data = rpm_to_radps((double)pkt.left_rpm);  pub_left_radps_->publish(m);
        m.data = rpm_to_radps((double)pkt.right_rpm); pub_right_radps_->publish(m);
      }
    }
  }

private:
  // params
  std::string imx7_ip_;
  int cmd_port_{5001};
  int fb_port_{5002};
  double cmd_rate_hz_{50.0};
  bool cmds_are_radps_{true};
  bool publish_feedback_radps_{true};

  // ros
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_cmd_, sub_right_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_rpm_, pub_right_rpm_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_radps_, pub_right_radps_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  // cmd state
  std::mutex cmd_mtx_;
  double left_cmd_{0.0};
  double right_cmd_{0.0};

  // udp
  int sock_tx_{-1};
  int sock_rx_{-1};
  sockaddr_in imx7_addr_{};

  // thread
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlexBotUdpBridge>());
  rclcpp::shutdown();
  return 0;
}
