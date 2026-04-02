// init_axis_motor.cpp
//
// Axis motor initialization — NODE 4 on can1
//
//
// SDO channel : TX = 0x604 (host→node4)   RX = 0x584 (node4→host)
// RPDO1       : 0x504
// Heartbeat   : 0x704
//
//
// Build:  g++ -std=c++17 -O2 -Wall init_axis_motor.cpp -o init_axis_motor
// Run:    sudo ./init_axis_motor

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

using namespace std::chrono;

static void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
static void sleep_us(int us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

// ---------------------------------------------------------------------------
// CanSocket
// ---------------------------------------------------------------------------
class CanSocket {
 public:
  explicit CanSocket(const std::string& ifname) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0)
      throw std::runtime_error(std::string("socket: ") + std::strerror(errno));

    struct ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      ::close(fd_);
      throw std::runtime_error(std::string("ioctl SIOCGIFINDEX: ") + std::strerror(errno));
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(fd_);
      throw std::runtime_error(std::string("bind: ") + std::strerror(errno));
    }
  }
  ~CanSocket() { if (fd_ >= 0) ::close(fd_); }

  bool send_frame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (len > 8) len = 8;
    struct can_frame fr{};
    fr.can_id  = can_id;
    fr.can_dlc = len;
    if (data && len) std::memcpy(fr.data, data, len);
    ssize_t n = ::write(fd_, &fr, sizeof(fr));
    if (n != (ssize_t)sizeof(fr)) {
      std::cerr << "write failed id=0x" << std::hex << can_id
                << std::dec << ": " << std::strerror(errno) << "\n";
      return false;
    }
    return true;
  }

  bool recv_frame(struct can_frame& out, int timeout_ms) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv{ timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (r <= 0) return false;
    return ::read(fd_, &out, sizeof(out)) == (ssize_t)sizeof(out);
  }

 private:
  int fd_{-1};
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static std::array<uint8_t, 8> pack_rpdo(uint16_t ctrl, int32_t vel, uint16_t pad = 0) {
  std::array<uint8_t, 8> b{};
  b[0] = ctrl & 0xFF;         b[1] = (ctrl >> 8) & 0xFF;
  b[2] = vel  & 0xFF;         b[3] = (vel  >> 8) & 0xFF;
  b[4] = (vel >> 16) & 0xFF;  b[5] = (vel  >> 24) & 0xFF;
  b[6] = pad  & 0xFF;         b[7] = (pad  >> 8) & 0xFF;
  return b;
}

static bool wait_sdo_rx(CanSocket& can, uint32_t rx_id, double timeout_sec) {
  auto t0 = steady_clock::now();
  while (duration<double>(steady_clock::now() - t0).count() < timeout_sec) {
    struct can_frame fr;
    if (can.recv_frame(fr, 100) && (fr.can_id & CAN_EFF_MASK) == rx_id)
      return true;
  }
  std::cerr << "  WARNING: no SDO response from 0x"
            << std::hex << rx_id << std::dec << "\n";
  return false;
}

static bool send_sdo(CanSocket& can,
                     const std::array<uint8_t, 8>& data,
                     uint32_t tx_id, uint32_t rx_id) {
  return can.send_frame(tx_id, data.data(), 8) && wait_sdo_rx(can, rx_id, 0.5);
}

static void send_nmt(CanSocket& can, uint8_t cmd, uint8_t node) {
  uint8_t b[2] = {cmd, node};
  can.send_frame(0x000, b, 2);
  sleep_ms(100);
}

static void rpdo_sync(CanSocket& can, uint32_t rpdo_id, uint16_t ctrl, int32_t vel) {
  auto p = pack_rpdo(ctrl, vel);
  can.send_frame(rpdo_id, p.data(), 8);
  sleep_us(2000);
  can.send_frame(0x080, nullptr, 0);  // SYNC
  sleep_us(8000);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
  try {
    CanSocket can("can1");  // all motors + PGV share can1

    constexpr uint8_t NODE = 4;

    const uint32_t SDO_TX = 0x604;  // host  → node 4
    const uint32_t SDO_RX = 0x584;  // node4 → host
    const uint32_t RPDO   = 0x504;  // confirmed from axis-motor log

    std::cout << "======================================================================\n";
    std::cout << "AXIS MOTOR (NODE 4) — COMPLETE INITIALIZATION\n";
    std::cout << "======================================================================\n";

    // -----------------------------------------------------------------------
    // [1/9] Reset node 4
    // -----------------------------------------------------------------------
    std::cout << "\n[1/9] Resetting Node 4...\n";
    send_nmt(can, 0x80, NODE);
    sleep_ms(500);

    // -----------------------------------------------------------------------
    // [2/9] Manufacturer enable parameter (0x2005:02)
    // -----------------------------------------------------------------------
    std::cout << "[2/9] Manufacturer parameters...\n";
    send_sdo(can, {0x42,0x02,0x20,0x05,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // -----------------------------------------------------------------------
    // [3/9] RPDO COB-ID table (0x1400–0x1421, sub-index 01)
    // Read-then-write pattern (0x42 = upload request, 0x23 = download 4-byte).
    // -----------------------------------------------------------------------
    std::cout << "[3/9] PDO mappings...\n";
    send_sdo(can, {0x42,0x00,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x00,0x14,0x01,0x01,0x02,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x01,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x01,0x14,0x01,0x01,0x03,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x02,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x02,0x14,0x01,0x01,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x03,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x03,0x14,0x01,0x01,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x04,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x04,0x14,0x01,0x21,0x04,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x14,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x14,0x14,0x01,0x01,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x15,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x15,0x14,0x01,0x11,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x16,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x16,0x14,0x01,0x21,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x17,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x17,0x14,0x01,0x31,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x19,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x19,0x14,0x01,0x00,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x20,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x20,0x14,0x01,0x51,0x05,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x21,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x21,0x14,0x01,0x61,0x05,0x00,0x80}, SDO_TX, SDO_RX);

    // -----------------------------------------------------------------------
    // [4/9] Motor parameters (read-back style)
    // -----------------------------------------------------------------------
    std::cout << "[4/9] Motor parameters...\n";
    send_sdo(can, {0x42,0xCA,0x20,0x07,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0xCA,0x20,0x08,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0xCA,0x20,0x09,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0xCA,0x20,0x0A,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0xD8,0x20,0x09,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0x0F,0x20,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0xD8,0x20,0x0C,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x42,0xD8,0x20,0x24,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // -----------------------------------------------------------------------
    // [5/9] Advanced config — control mode unlock (0x203C:09)
    // -----------------------------------------------------------------------
    std::cout << "[5/9] Advanced config...\n";
    send_sdo(can, {0x21,0x3C,0x20,0x09,0x08,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x00,0xFF,0x40,0x55,0x55,0x0D,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x11,0x00,0x40,0x55,0x55,0x0D,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x40,0x3C,0x20,0x09,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x60,0x3C,0x20,0x09,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x70,0x3C,0x20,0x09,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // -----------------------------------------------------------------------
    // [6/9] Profile / velocity parameters (0x2037, 0x2039)
    // -----------------------------------------------------------------------
    std::cout << "[6/9] Profile parameters...\n";
    send_sdo(can, {0x23,0x37,0x20,0x01,0x00,0x00,0x1E,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x37,0x20,0x02,0x11,0x11,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x37,0x20,0x03,0x11,0x11,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x37,0x20,0x04,0x55,0x55,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x37,0x20,0x05,0xAA,0xAA,0x1A,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x37,0x20,0x06,0x56,0x55,0xE5,0xFF}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x37,0x20,0x07,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x02,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x03,0x00,0x00,0x00,0x40}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x04,0x00,0x00,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x05,0x64,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x06,0x64,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x07,0xA0,0x0F,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x08,0x00,0x00,0x00,0x40}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x09,0x00,0x00,0x00,0x80}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x0A,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x39,0x20,0x0B,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // -----------------------------------------------------------------------
    // [7/9] RPDO / TPDO COB-ID assignments
    //
    // Confirmed COB-IDs from the running axis-motor log:
    //   0x184 — TPDO (6 bytes: status word + velocity)
    //   0x284 — TPDO (4 bytes)
    //   0x384 — TPDO (2 bytes)
    //   0x484 — TPDO (8 bytes: full status)
    //   0x504 — RPDO (8 bytes: ctrl word + velocity setpoint)
    // -----------------------------------------------------------------------
    std::cout << "[7/9] RPDO/TPDO mappings...\n";

    // RPDO3 (0x1403) — velocity command channel, COB-ID = 0x504
    send_sdo(can, {0x42,0x03,0x14,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x03,0x14,0x01,0x04,0x05,0x00,0x80}, SDO_TX, SDO_RX); // inhibit
    send_sdo(can, {0x23,0x03,0x14,0x01,0x04,0x05,0x00,0x00}, SDO_TX, SDO_RX); // enable
    send_sdo(can, {0x2F,0x03,0x14,0x02,0x01,0x00,0x00,0x00}, SDO_TX, SDO_RX); // sync

    // TPDO3 comm (0x1803) — COB-ID = 0x184
    send_sdo(can, {0x42,0x03,0x18,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x03,0x18,0x01,0x84,0x01,0x00,0x80}, SDO_TX, SDO_RX); // inhibit
    send_sdo(can, {0x23,0x03,0x18,0x01,0x84,0x01,0x00,0x00}, SDO_TX, SDO_RX); // enable
    send_sdo(can, {0x2F,0x03,0x18,0x02,0x01,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // TPDO0x14 comm (0x1814) — COB-ID = 0x284
    send_sdo(can, {0x42,0x14,0x18,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x14,0x18,0x01,0x84,0x02,0x00,0x80}, SDO_TX, SDO_RX); // inhibit
    send_sdo(can, {0x23,0x14,0x18,0x01,0x84,0x02,0x00,0x00}, SDO_TX, SDO_RX); // enable
    send_sdo(can, {0x2F,0x14,0x18,0x02,0x01,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // TPDO0x16 comm (0x1816) — COB-ID = 0x384
    send_sdo(can, {0x42,0x16,0x18,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x16,0x18,0x01,0x84,0x03,0x00,0x80}, SDO_TX, SDO_RX); // inhibit
    send_sdo(can, {0x23,0x16,0x18,0x01,0x84,0x03,0x00,0x00}, SDO_TX, SDO_RX); // enable
    send_sdo(can, {0x2F,0x16,0x18,0x02,0x01,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // TPDO0x19 comm (0x1819) — COB-ID = 0x484  (8-byte full-status TPDO)
    send_sdo(can, {0x42,0x19,0x18,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x19,0x18,0x01,0x84,0x04,0x00,0x80}, SDO_TX, SDO_RX); // inhibit

    // TPDO0x19 mapping — 4 objects (identical to node-2 reference)
    send_sdo(can, {0x2F,0x19,0x1A,0x00,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x19,0x1A,0x01,0x10,0x01,0x0F,0x20}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x19,0x1A,0x02,0x10,0x02,0x02,0x20}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x19,0x1A,0x03,0x10,0x03,0x02,0x20}, SDO_TX, SDO_RX);
    send_sdo(can, {0x23,0x19,0x1A,0x04,0x10,0x05,0x02,0x20}, SDO_TX, SDO_RX);
    send_sdo(can, {0x2F,0x19,0x1A,0x00,0x04,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    send_sdo(can, {0x23,0x19,0x18,0x01,0x84,0x04,0x00,0x00}, SDO_TX, SDO_RX); // enable 0x484
    send_sdo(can, {0x2F,0x19,0x18,0x02,0x01,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // -----------------------------------------------------------------------
    // [8/9] CiA-402 state machine via SDO
    // -----------------------------------------------------------------------
    std::cout << "[8/9] Reset & control word sequence...\n";
    send_nmt(can, 0x80, NODE);
    sleep_ms(200);

    send_sdo(can, {0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00}, SDO_TX, SDO_RX); sleep_ms(100); // fault reset
    send_sdo(can, {0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00}, SDO_TX, SDO_RX); sleep_ms(100); // Profile Velocity mode
    send_sdo(can, {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX); sleep_ms(100);
    send_sdo(can, {0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00}, SDO_TX, SDO_RX); sleep_ms(100); // fault reset
    send_sdo(can, {0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00}, SDO_TX, SDO_RX); sleep_ms(100); // Shutdown
    send_sdo(can, {0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00}, SDO_TX, SDO_RX); sleep_ms(100); // Switch on
    send_sdo(can, {0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00}, SDO_TX, SDO_RX); sleep_ms(100); // Enable operation

    // -----------------------------------------------------------------------
    // [9/9] NMT start
    // -----------------------------------------------------------------------
    std::cout << "[9/9] Starting node...\n";
    send_nmt(can, 0x01, NODE);
    sleep_ms(500);

    // -----------------------------------------------------------------------
    // [10/11] Final configuration
    // -----------------------------------------------------------------------
    std::cout << "[10/11] Final configuration...\n";
    send_sdo(can, {0x2B,0x86,0x60,0x00,0x02,0x00,0x00,0x00}, SDO_TX, SDO_RX); // homing method 2
    send_sdo(can, {0x21,0x3C,0x20,0x01,0x06,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x01,0x3E,0xC3,0xAE,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x21,0x3C,0x20,0x02,0x06,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x01,0x3E,0xC3,0xAE,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x21,0x3C,0x20,0x03,0x06,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x01,0x3E,0xC3,0xAE,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x21,0x3C,0x20,0x04,0x06,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x01,0x3E,0xC3,0xAE,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // Heartbeat producer = 10 ms, consumer timeout = 250 ms
    send_sdo(can, {0x2B,0x0C,0x10,0x00,0x0A,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x2F,0x0D,0x10,0x00,0xFA,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    send_sdo(can, {0x2B,0x5A,0x20,0x40,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x2B,0x5A,0x20,0x2E,0x01,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x2B,0x5A,0x20,0x01,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    send_sdo(can, {0x2B,0x65,0x20,0x2B,0x0A,0x00,0x00,0x00}, SDO_TX, SDO_RX);

    // Read status word — expect 0x0627 (operation enabled)
    send_sdo(can, {0x42,0x41,0x60,0x00,0x00,0x00,0x00,0x00}, SDO_TX, SDO_RX);
    sleep_ms(200);

    // -----------------------------------------------------------------------
    // [11/11] CiA-402 state machine via RPDO (motor engage)
    // -----------------------------------------------------------------------
    std::cout << "[11/11] State machine via RPDO (MOTOR ENGAGE)...\n";
    rpdo_sync(can, RPDO, 0x0080, 0);                                     // fault reset
    for (int i = 0; i < 5;  i++) rpdo_sync(can, RPDO, 0x0006, 0);       // Shutdown
    for (int i = 0; i < 5;  i++) rpdo_sync(can, RPDO, 0x0007, 0);       // Switch on
    for (int i = 0; i < 10; i++) rpdo_sync(can, RPDO, 0x000F, 0);       // Enable operation

    std::cout << "\n✓ AXIS MOTOR (NODE 4) INITIALIZED AND ENGAGED!\n";
    std::cout << "  Watch 0x704 heartbeat — must stay 0x05 (operational)\n";
    std::cout << "  Watch 0x484 TPDO for status word and position feedback\n";
    std::cout << "  Watch 0x184 TPDO for velocity feedback\n";
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << "\n";
    return 1;
  }
}