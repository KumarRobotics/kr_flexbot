// pgv_rx.cpp
// PC: receive PGV UDP packets on port 5003 and display a live dashboard
//
// Build:
//   g++ -std=c++17 -O2 -Wall -Wextra -pthread -o pgv_rx pgv_rx.cpp
//
// Run:
//   ./pgv_rx 5003
//
// (Make sure PC has IP 192.168.0.20 on that subnet.)

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#pragma pack(push, 1)
struct UdpPgvPacket {
  uint32_t magic;       // 'PGV1'
  uint16_t version;     // 1
  uint16_t payload_len; // sizeof(UdpPgvPacket)
  uint32_t seq;
  uint64_t host_time_us;

  int32_t  x_01mm;
  int32_t  y_01mm;
  int16_t  ang_01deg;
  int16_t  spd_01mms;
  uint32_t pgv_ts_ms;
  uint8_t  tag_id;
  uint8_t  reserved[3];
};
#pragma pack(pop)

static inline uint64_t ntohll_u64(uint64_t x) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (uint64_t(ntohl(uint32_t(x & 0xFFFFFFFFULL))) << 32) | ntohl(uint32_t(x >> 32));
#else
  return x;
#endif
}

struct State {
  uint32_t seq = 0;
  uint64_t host_time_us = 0;

  int32_t x_01mm = 0;
  int32_t y_01mm = 0;
  int16_t ang_01deg = 0;
  int16_t spd_01mms = 0;
  uint32_t pgv_ts_ms = 0;
  uint8_t tag_id = 0;
};

static void clear_screen() {
  std::cout << "\033[2J\033[H";
}

static void print_dashboard(const State& s) {
  clear_screen();
  std::cout << "==============================================\n";
  std::cout << "      PGV100RS UDP LIVE DASHBOARD\n";
  std::cout << "==============================================\n\n";

  auto mm = [](int32_t v01) { return double(v01) * 0.1; };
  auto deg = [](int16_t v01) { return double(v01) * 0.1; };
  auto mms = [](int16_t v01) { return double(v01) * 0.1; };

  std::cout << std::fixed << std::setprecision(2);

  std::cout << " Seq          : " << s.seq << "\n";
  std::cout << " Sender t(us) : " << s.host_time_us << "\n\n";

  std::cout << " X Position   : " << std::setw(8) << mm(s.x_01mm) << " mm\n";
  std::cout << " PGV Timestamp: " << std::setw(8) << s.pgv_ts_ms << " ms\n";
  std::cout << " Y Offset     : " << std::setw(8) << mm(s.y_01mm) << " mm\n";
  std::cout << " Angle        : " << std::setw(8) << deg(s.ang_01deg) << " deg\n";
  std::cout << " Speed        : " << std::setw(8) << mms(s.spd_01mms) << " mm/s\n";
  std::cout << " Tag ID       : " << std::setw(8) << int(s.tag_id) << "\n";

  std::cout << "\nPress CTRL+C to stop.\n";
  std::cout.flush();
}

int main(int argc, char** argv) {
  uint16_t port = 5003;
  if (argc >= 2) port = static_cast<uint16_t>(std::stoi(argv[1]));

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::perror("socket");
    return 1;
  }

  sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::perror("bind");
    close(sock);
    return 1;
  }

  std::cerr << "[pgv_rx] Listening UDP :0.0.0.0:" << port << "\n";

  State st {};

  using clock = std::chrono::steady_clock;
  auto last_refresh = clock::now();

  while (true) {
    UdpPgvPacket pkt {};
    sockaddr_in src {};
    socklen_t slen = sizeof(src);

    const ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0,
                               reinterpret_cast<sockaddr*>(&src), &slen);
    if (n < 0) {
      std::perror("recvfrom");
      continue;
    }
    if (n != (ssize_t)sizeof(UdpPgvPacket)) {
      continue;
    }

    const uint32_t magic = ntohl(pkt.magic);
    const uint16_t ver   = ntohs(pkt.version);
    const uint16_t plen  = ntohs(pkt.payload_len);

    if (magic != 0x50475631u || ver != 1 || plen != sizeof(UdpPgvPacket)) {
      continue;
    }

    st.seq         = ntohl(pkt.seq);
    st.host_time_us = ntohll_u64(pkt.host_time_us);

    st.x_01mm      = (int32_t)ntohl((uint32_t)pkt.x_01mm);
    st.y_01mm      = (int32_t)ntohl((uint32_t)pkt.y_01mm);
    st.ang_01deg   = (int16_t)ntohs((uint16_t)pkt.ang_01deg);
    st.spd_01mms   = (int16_t)ntohs((uint16_t)pkt.spd_01mms);
    st.pgv_ts_ms   = ntohl(pkt.pgv_ts_ms);
    st.tag_id      = pkt.tag_id;

    // refresh at ~10Hz even if packets are faster
    auto now = clock::now();
    if (std::chrono::duration<double>(now - last_refresh).count() >= 0.1) {
      print_dashboard(st);
      last_refresh = now;
    }
  }

  close(sock);
  return 0;
}