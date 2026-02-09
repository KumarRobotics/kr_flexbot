/**
 * Low-Level Motor Controller with UDP Communication (C++)
 * 
 * Receives: (left_rpm, right_rpm) from high-level via UDP
 * Sends:    (left_enc, right_enc, timestamp) to high-level via UDP
 * Controls: Motors via SocketCAN
 * 
 * Build:
 *   g++ -std=c++17 -O2 -o motor_controller motor_controller.cpp -lpthread
 * 
 * Run:
 *   sudo ./motor_controller
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>

// ─── Configuration ───────────────────────────────────────────────────────────

static constexpr const char*  CAN_INTERFACE       = "can0";
static constexpr double       CMD_PER_WHEEL_RPM   = 4145.0;
static constexpr uint16_t     UDP_LISTEN_PORT     = 5001;
static constexpr const char*  UDP_SEND_IP         = "192.168.0.20";  // Jetson IP
static constexpr uint16_t     UDP_SEND_PORT       = 5002;
static constexpr int          CONTROL_RATE_HZ     = 50;   // 50Hz motor control
static constexpr int          ENCODER_RATE_HZ     = 20;   // 20Hz encoder feedback
static constexpr double       ENCODER_SDO_TIMEOUT = 0.05; // 50ms SDO timeout

// ─── CAN IDs ─────────────────────────────────────────────────────────────────

static constexpr uint32_t RPDO_BASE   = 0x500;  // RPDO: 0x501 (node1), 0x502 (node2)
static constexpr uint32_t SDO_REQ     = 0x600;  // SDO request: 0x601, 0x602
static constexpr uint32_t SDO_RESP    = 0x580;  // SDO response: 0x581, 0x582
static constexpr uint32_t SYNC_ID     = 0x080;  // SYNC broadcast

// ─── CANopen Object Indices ──────────────────────────────────────────────────

static constexpr uint16_t OBJ_CONTROLWORD      = 0x6040;
static constexpr uint16_t OBJ_POSITION_ACTUAL  = 0x6064;
static constexpr uint16_t OBJ_VELOCITY_ACTUAL  = 0x606C;

// ─── Global State ────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};  // Set to false on shutdown

struct MotorCommand {
    double left_rpm  = 0.0;
    double right_rpm = 0.0;
};

static MotorCommand g_cmd;
static std::mutex   g_cmd_mutex;

// ─── Utility: Sleep for exact duration ───────────────────────────────────────

static void sleep_until(std::chrono::steady_clock::time_point target) {
    auto now = std::chrono::steady_clock::now();
    if (target > now)
        std::this_thread::sleep_until(target);
}

static double get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto dur = now.time_since_epoch();
    return std::chrono::duration<double>(dur).count();
}

// ─── CAN Socket ──────────────────────────────────────────────────────────────

class CANSocket {
public:
    CANSocket() : fd_(-1) {}

    ~CANSocket() {
        if (fd_ >= 0) close(fd_);
    }

    bool open(const char* iface) {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) {
            perror("CAN socket()");
            return false;
        }

        struct ifreq ifr;
        strncpy(ifr.ifr_name, iface, sizeof(ifr.ifr_name) - 1);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            perror("CAN ioctl(SIOCGIFINDEX)");
            return false;
        }

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family   = AF_CAN;
        addr.can_ifindex  = ifr.ifr_ifindex;

        if (bind(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("CAN bind()");
            return false;
        }

        // Set receive timeout to 10ms
        struct timeval tv = {0, 10000};
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        return true;
    }

    bool send(uint32_t id, const uint8_t* data, uint8_t len) {
        struct can_frame frame;
        frame.can_id  = id;
        frame.can_dlc = len;
        memcpy(frame.data, data, len);
        return ::send(fd_, &frame, sizeof(frame), 0) > 0;
    }

    // Returns true if a frame was received, false on timeout
    bool recv(struct can_frame& frame) {
        ssize_t nbytes = ::read(fd_, &frame, sizeof(frame));
        return nbytes == (ssize_t)sizeof(frame);
    }

    int fd() const { return fd_; }

private:
    int fd_;
};

static CANSocket g_can;

// ─── CAN Motor Functions ─────────────────────────────────────────────────────

/**
 * Send RPDO command to a specific motor node.
 * Payload: [controlword (u16)][velocity (i32)][unused (u16)]
 */
static void send_cmd_motor(uint8_t node, uint16_t ctrl, int32_t vel = 0) {
    uint8_t data[8];
    memcpy(data + 0, &ctrl, 2);  // controlword (little-endian on ARM)
    memcpy(data + 2, &vel,  4);  // velocity
    uint16_t pad = 0;
    memcpy(data + 6, &pad,  2);  // padding

    g_can.send(RPDO_BASE + node, data, 8);
    usleep(1000);  // 1ms
}

/**
 * Send SYNC message to trigger PDO processing on all nodes.
 */
static void send_sync() {
    uint8_t empty[8] = {};
    g_can.send(SYNC_ID, empty, 0);
    usleep(1000);  // 1ms
}

/**
 * Read encoder position (0x6064) from a motor node via SDO.
 * Returns true if successful and writes count to `out_counts`.
 */
static bool read_encoder(uint8_t node, int32_t& out_counts) {
    // SDO upload request for index 0x6064, subindex 0x00
    uint8_t req[8] = {
        0x40,                          // SDO upload initiate
        (uint8_t)(OBJ_POSITION_ACTUAL & 0xFF),
        (uint8_t)(OBJ_POSITION_ACTUAL >> 8),
        0x00,                          // subindex
        0x00, 0x00, 0x00, 0x00
    };

    uint32_t resp_id = SDO_RESP + node;
    g_can.send(SDO_REQ + node, req, 8);

    auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds((int)(ENCODER_SDO_TIMEOUT * 1000));

    struct can_frame frame;
    while (std::chrono::steady_clock::now() < deadline) {
        if (g_can.recv(frame)) {
            if (frame.can_id == resp_id && frame.can_dlc >= 8) {
                // SDO response: bytes 4-7 contain the 32-bit value
                memcpy(&out_counts, &frame.data[4], 4);
                return true;
            }
        }
    }
    return false;  // timeout
}

// ─── Emergency Stop ──────────────────────────────────────────────────────────

static void emergency_stop() {
    fprintf(stderr, "\n[!] Emergency stop!\n");
    for (int i = 0; i < 50; ++i) {
        send_cmd_motor(1, 0x000F, 0);
        send_cmd_motor(2, 0x000F, 0);
        send_sync();
        usleep(1000);
    }
}

// ─── Signal Handler ──────────────────────────────────────────────────────────

static void signal_handler(int sig) {
    (void)sig;
    g_running.store(false);
}

// ─── Thread: UDP Receiver ────────────────────────────────────────────────────
/**
 * Listens on UDP_LISTEN_PORT for velocity commands from the high-level.
 * Expected payload: two floats (left_rpm, right_rpm) = 8 bytes.
 */

static void udp_receiver_thread() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("UDP cmd socket()");
        return;
    }

    // Allow port reuse (useful during restart)
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Set receive timeout to 10ms (non-blocking style)
    struct timeval tv = {0, 10000};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(UDP_LISTEN_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("UDP cmd bind()");
        close(sock);
        return;
    }

    printf("[UDP RX] Listening on port %d\n", UDP_LISTEN_PORT);

    uint8_t buf[1024];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);

    while (g_running.load()) {
        ssize_t nbytes = recvfrom(sock, buf, sizeof(buf), 0,
                                  (struct sockaddr*)&sender, &sender_len);

        if (nbytes == 8) {  // 2 floats = 8 bytes
            float left_rpm, right_rpm;
            memcpy(&left_rpm,  buf + 0, 4);
            memcpy(&right_rpm, buf + 4, 4);

            {
                std::lock_guard<std::mutex> lock(g_cmd_mutex);
                g_cmd.left_rpm  = left_rpm * -1.0f;   // Invert left wheel if needed
                g_cmd.right_rpm = right_rpm * 1.0f;  // Invert right wheel if needed
            }

            printf("[UDP RX] L=%.1f R=%.1f RPM\n", left_rpm, right_rpm);

        } else if (nbytes < 0) {
            continue;
        }
    }

    close(sock);
}

// ─── Thread: Motor Control Loop ──────────────────────────────────────────────
/**
 * Runs at CONTROL_RATE_HZ (50Hz).
 * Reads the latest command from g_cmd and sends CAN RPDO + SYNC.
 */

static void motor_control_thread() {
    printf("[CTRL] Motor control loop started at %d Hz\n", CONTROL_RATE_HZ);

    auto interval = std::chrono::milliseconds(1000 / CONTROL_RATE_HZ);
    auto next     = std::chrono::steady_clock::now() + interval;

    while (g_running.load()) {
        double left_rpm, right_rpm;
        {
            std::lock_guard<std::mutex> lock(g_cmd_mutex);
            left_rpm  = g_cmd.left_rpm;
            right_rpm = g_cmd.right_rpm;
        }

        int32_t left_cmd  = (int32_t)std::round(left_rpm  * CMD_PER_WHEEL_RPM);
        int32_t right_cmd = (int32_t)std::round(right_rpm * CMD_PER_WHEEL_RPM);

        send_cmd_motor(1, 0x000F, left_cmd);
        send_cmd_motor(2, 0x000F, right_cmd);
        send_sync();

        next += interval;
        sleep_until(next);
    }
}

// ─── Thread: Encoder Feedback Loop ───────────────────────────────────────────
/**
 * Runs at ENCODER_RATE_HZ (20Hz).
 * Reads encoder positions via SDO and sends them back via UDP.
 * Payload: [left_enc (i32)][right_enc (i32)][timestamp (f64)] = 16 bytes
 */

static void encoder_feedback_thread() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("UDP enc socket()");
        return;
    }

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_SEND_PORT);
    inet_pton(AF_INET, UDP_SEND_IP, &dest.sin_addr);

    printf("[ENC] Encoder feedback started at %d Hz → %s:%d\n",
           ENCODER_RATE_HZ, UDP_SEND_IP, UDP_SEND_PORT);

    auto interval = std::chrono::milliseconds(1000 / ENCODER_RATE_HZ);
    auto next     = std::chrono::steady_clock::now() + interval;
    int  tick     = 0;

    while (g_running.load()) {
        int32_t left_enc  = 0;
        int32_t right_enc = 0;

        bool left_ok  = read_encoder(1, left_enc);
        bool right_ok = read_encoder(2, right_enc);

        if (left_ok && right_ok) {
            double ts = get_timestamp();

            // Pack: [i32][i32][f64] = 16 bytes
            uint8_t buf[16];
            memcpy(buf + 0, &left_enc,  4);
            memcpy(buf + 4, &right_enc, 4);
            memcpy(buf + 8, &ts,        8);

            sendto(sock, buf, 16, 0,
                   (struct sockaddr*)&dest, sizeof(dest));

            // Print every 0.5s
            if (++tick % (ENCODER_RATE_HZ / 2) == 0)
                printf("[ENC] L=%d R=%d\n", left_enc, right_enc);
        }

        next += interval;
        sleep_until(next);
    }

    close(sock);
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    // Install signal handlers for clean shutdown
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    printf("======================================================================\n");
    printf("  LOW-LEVEL MOTOR CONTROLLER (C++)\n");
    printf("======================================================================\n");
    printf("  CAN interface : %s\n", CAN_INTERFACE);
    printf("  CMD/RPM       : %.1f\n", CMD_PER_WHEEL_RPM);
    printf("  UDP listen    : port %d\n", UDP_LISTEN_PORT);
    printf("  UDP send      : %s:%d\n", UDP_SEND_IP, UDP_SEND_PORT);
    printf("  Control rate  : %d Hz\n", CONTROL_RATE_HZ);
    printf("  Encoder rate  : %d Hz\n", ENCODER_RATE_HZ);
    printf("======================================================================\n");

    // Open CAN socket
    if (!g_can.open(CAN_INTERFACE)) {
        fprintf(stderr, "[ERR] Failed to open CAN interface '%s'\n", CAN_INTERFACE);
        return 1;
    }
    printf("[CAN] Opened %s\n", CAN_INTERFACE);

    // Launch threads
    std::thread t_udp_rx  (udp_receiver_thread);
    std::thread t_ctrl    (motor_control_thread);
    std::thread t_enc     (encoder_feedback_thread);

    // Wait for shutdown signal
    t_udp_rx.join();
    t_ctrl.join();
    t_enc.join();

    // Clean shutdown
    emergency_stop();
    printf("\n✓ Shutdown complete\n");
    return 0;
}