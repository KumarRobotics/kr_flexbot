// pgv_tx.cpp
// IMX7: read PGV PDOs from SocketCAN and transmit UDP packets to 192.168.0.20:5003
//
// Build:
//   g++ -std=c++17 -O2 -Wall -Wextra -pthread -o pgv_tx pgv_tx.cpp
//
// Run (example):
//   ./pgv_tx can1 192.168.0.20 5003
//
// Notes:
// - Needs SocketCAN enabled and interface up (e.g., ip link set can1 up type can bitrate 1000000)
// - Might require root or CAP_NET_RAW to open CAN raw socket.

#include <arpa/inet.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

// ---------------- PGV CAN config ----------------
static constexpr uint8_t  NODE_ID = 0x05;
static constexpr uint32_t TPDO2   = 0x280 + NODE_ID;
static constexpr uint32_t TPDO3   = 0x380 + NODE_ID;
static constexpr uint32_t TPDO4   = 0x480 + NODE_ID;
static constexpr uint32_t PDO5    = 0x680 + NODE_ID;

// Python used big-endian for payload decoding.
// We'll decode big-endian as well.

static inline int16_t be_i16(const uint8_t* p) {
  return static_cast<int16_t>((p[0] << 8) | p[1]);
}
static inline int32_t be_i32(const uint8_t* p) {
  uint32_t u = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
  return static_cast<int32_t>(u);
}
static inline uint32_t be_u32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

// ---------------- State ----------------
// Keep scaled ints to avoid floats.
// x_01mm means "x in 0.1mm units"
struct PgvState {
  int32_t  x_01mm      = 0;   // from TPDO4
  int32_t  y_01mm      = 0;   // from TPDO2
  int16_t  ang_01deg   = 0;   // from TPDO2
  int16_t  spd_01mms   = 0;   // from PDO5 (speed * 0.1 mm/s)
  uint32_t pgv_ts_ms   = 0;   // from TPDO4
  uint8_t  tag_id      = 0;   // from TPDO3 (d[7])
};

#pragma pack(push, 1)
struct UdpPgvPacket {
  uint32_t magic;      // 'PGV1' = 0x50475631
  uint16_t version;    // 1
  uint16_t payload_len;// sizeof(UdpPgvPacket) - header? (kept for sanity)
  uint32_t seq;        // sequence number
  uint64_t host_time_us; // sender time (monotonic) in us

  int32_t  x_01mm;
  int32_t  y_01mm;
  int16_t  ang_01deg;
  int16_t  spd_01mms;
  uint32_t pgv_ts_ms;
  uint8_t  tag_id;
  uint8_t  reserved[3]; // pad to 4B alignment, future use
};
#pragma pack(pop)

static_assert(sizeof(UdpPgvPacket) == 4 + 2 + 2 + 4 + 8 + 4 + 4 + 2 + 2 + 4 + 1 + 3, "Packet size mismatch");

// host->net helpers for 64-bit
static inline uint64_t htonll_u64(uint64_t x) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (uint64_t(htonl(uint32_t(x & 0xFFFFFFFFULL))) << 32) | htonl(uint32_t(x >> 32));
#else
  return x;
#endif
}

static inline uint64_t monotonic_us() {
  using namespace std::chrono;
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

static int open_can_socket(const std::string& ifname) {
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    std::perror("socket(PF_CAN)");
    return -1;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    std::perror("ioctl(SIOCGIFINDEX)");
    close(s);
    return -1;
  }

  // Filter only the PDO IDs we care about.
  can_filter rfilter[4];
  rfilter[0].can_id   = TPDO2;
  rfilter[0].can_mask = CAN_SFF_MASK;
  rfilter[1].can_id   = TPDO3;
  rfilter[1].can_mask = CAN_SFF_MASK;
  rfilter[2].can_id   = TPDO4;
  rfilter[2].can_mask = CAN_SFF_MASK;
  rfilter[3].can_id   = PDO5;
  rfilter[3].can_mask = CAN_SFF_MASK;
  if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
    std::perror("setsockopt(CAN_RAW_FILTER)");
    // not fatal; continue
  }

  sockaddr_can addr {};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::perror("bind(CAN)");
    close(s);
    return -1;
  }

  return s;
}

static int open_udp_socket(sockaddr_in& dst, const std::string& dst_ip, uint16_t dst_port) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) {
    std::perror("socket(AF_INET,SOCK_DGRAM)");
    return -1;
  }

  std::memset(&dst, 0, sizeof(dst));
  dst.sin_family = AF_INET;
  dst.sin_port   = htons(dst_port);
  if (inet_pton(AF_INET, dst_ip.c_str(), &dst.sin_addr) != 1) {
    std::cerr << "inet_pton failed for " << dst_ip << "\n";
    close(s);
    return -1;
  }

  return s;
}

static void decode_frame_update_state(const can_frame& f, PgvState& st) {
  const uint32_t id = (f.can_id & CAN_SFF_MASK);
  const uint8_t* d  = f.data;
  const int dlc      = f.can_dlc;

  if (id == TPDO2) {
    if (dlc < 6) return;
    const int32_t y_raw     = be_i32(&d[0]);  // in 0.1mm units (POS_RES=0.1 in python)
    const int16_t ang_raw   = be_i16(&d[4]);  // in 0.1deg units (ANGLE_RES=0.1)
    st.y_01mm    = y_raw;    // already scaled as per python’s POS_RES (assuming PGV sends raw in 0.1mm units)
    st.ang_01deg = ang_raw;  // already scaled in 0.1deg
  } else if (id == TPDO3) {
    if (dlc < 8) return;
    uint8_t tag = d[7];
    if (tag != 0) st.tag_id = tag;
  } else if (id == TPDO4) {
    if (dlc < 8) return;
    const int32_t  x_raw = be_i32(&d[0]); // 0.1mm units
    const uint32_t ts    = be_u32(&d[4]); // ms
    st.x_01mm    = x_raw;
    st.pgv_ts_ms = ts;
  } else if (id == PDO5) {
    if (dlc < 2) return;
    const int16_t spd = be_i16(&d[0]);
    // python: speed_mm_s = speed * 0.1
    // so keep 0.1 mm/s units:
    st.spd_01mms = spd;
  }
}

int main(int argc, char** argv) {
  std::string can_if = "can1";
  std::string dst_ip = "192.168.0.20";
  uint16_t    dst_port = 5003;

  if (argc >= 2) can_if = argv[1];
  if (argc >= 3) dst_ip = argv[2];
  if (argc >= 4) dst_port = static_cast<uint16_t>(std::stoi(argv[3]));

  int can_sock = open_can_socket(can_if);
  if (can_sock < 0) return 1;

  sockaddr_in dst {};
  int udp_sock = open_udp_socket(dst, dst_ip, dst_port);
  if (udp_sock < 0) {
    close(can_sock);
    return 1;
  }

  std::cerr << "[pgv_tx] CAN: " << can_if << "  -> UDP: " << dst_ip << ":" << dst_port
            << " (IMX7 IP expected 192.168.0.2)\n";

  PgvState st {};
  uint32_t seq = 0;

  using clock = std::chrono::steady_clock;
  auto last_send = clock::now();

  while (true) {
    // Use select with timeout so we can send at 10Hz even if no CAN traffic arrives.
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(can_sock, &rfds);

    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 20 * 1000; // 20ms tick

    int ret = select(can_sock + 1, &rfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) continue;
      std::perror("select");
      break;
    }

    if (ret > 0 && FD_ISSET(can_sock, &rfds)) {
      can_frame f {};
      const int n = read(can_sock, &f, sizeof(f));
      if (n == (int)sizeof(f)) {
        decode_frame_update_state(f, st);
      }
    }

    auto now = clock::now();
    if (std::chrono::duration<double>(now - last_send).count() >= 0.1) { // 10 Hz
      UdpPgvPacket pkt {};
      pkt.magic       = htonl(0x50475631u); // 'PGV1'
      pkt.version     = htons(1);
      pkt.payload_len = htons(static_cast<uint16_t>(sizeof(UdpPgvPacket)));
      pkt.seq         = htonl(seq++);
      pkt.host_time_us = htonll_u64(monotonic_us());

      pkt.x_01mm    = htonl(static_cast<uint32_t>(st.x_01mm));
      pkt.y_01mm    = htonl(static_cast<uint32_t>(st.y_01mm));
      pkt.ang_01deg = htons(static_cast<uint16_t>(st.ang_01deg));
      pkt.spd_01mms = htons(static_cast<uint16_t>(st.spd_01mms));
      pkt.pgv_ts_ms = htonl(st.pgv_ts_ms);
      pkt.tag_id    = st.tag_id;

      const ssize_t sent = sendto(udp_sock, &pkt, sizeof(pkt), 0,
                                  reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
      if (sent < 0) {
        std::perror("sendto");
      }

      last_send = now;
    }
  }

  close(udp_sock);
  close(can_sock);
  return 0;
}