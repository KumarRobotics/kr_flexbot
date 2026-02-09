// udp_cmd_client.cpp  (run on companion)
// Sends LED or LCD commands to IMX7 at 192.168.10.2:5006

#include <arpa/inet.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>
#include <sys/socket.h>

static constexpr const char* IMX7_IP = "192.168.0.2";
static constexpr int IMX7_PORT = 5006;

static uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

#pragma pack(push, 1)
struct CmdHeader {
  uint32_t magic;   // "CMDS"
  uint16_t version; // 1
  uint16_t cmd_id;  // 1=LED, 2=LCD
  uint32_t seq;
  uint16_t len;
  uint16_t reserved;
  uint32_t crc32;
};

struct LedPayload { uint8_t r, g, b, _pad; };

struct LcdPayload {
  uint8_t line;   // 0..3
  char text[20];  // padded with spaces
};
#pragma pack(pop)

static void send_packet(int fd, const sockaddr_in& dst, CmdHeader& hdr, const uint8_t* payload) {
  uint8_t buf[512];
  std::memcpy(buf, &hdr, sizeof(hdr));
  if (hdr.len) std::memcpy(buf + sizeof(hdr), payload, hdr.len);

  // CRC over header(with crc=0) + payload
  CmdHeader tmp = hdr;
  tmp.crc32 = 0;
  std::memcpy(buf, &tmp, sizeof(tmp));
  uint32_t c = crc32(buf, sizeof(tmp) + hdr.len);

  hdr.crc32 = c;
  std::memcpy(buf, &hdr, sizeof(hdr));

  sendto(fd, buf, sizeof(hdr) + hdr.len, 0, (const sockaddr*)&dst, sizeof(dst));
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
      "Usage:\n"
      "  %s led R G B\n"
      "  %s lcd LINE \"text\"\n", argv[0], argv[0]);
    return 1;
  }

  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) { std::perror("socket"); return 1; }

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(IMX7_PORT);
  inet_pton(AF_INET, IMX7_IP, &dst.sin_addr);

  static uint32_t seq = 0;

  std::string mode = argv[1];
  if (mode == "led" && argc == 5) {
    LedPayload p{};
    p.r = (uint8_t)std::stoi(argv[2]);
    p.g = (uint8_t)std::stoi(argv[3]);
    p.b = (uint8_t)std::stoi(argv[4]);

    CmdHeader h{};
    h.magic = 0x53444D43u;
    h.version = 1;
    h.cmd_id = 1;
    h.seq = seq++;
    h.len = sizeof(LedPayload);
    h.reserved = 0;
    h.crc32 = 0;

    send_packet(fd, dst, h, (const uint8_t*)&p);
    std::printf("Sent LED %u %u %u\n", p.r, p.g, p.b);
  }
  else if (mode == "lcd" && argc >= 4) {
    int line = std::stoi(argv[2]);
    std::string text = argv[3];

    LcdPayload p{};
    p.line = (uint8_t)line;
    // pad to 20
    for (int i=0;i<20;i++) p.text[i] = ' ';
    for (int i=0;i<20 && i<(int)text.size(); i++) p.text[i] = text[i];

    CmdHeader h{};
    h.magic = 0x53444D43u;
    h.version = 1;
    h.cmd_id = 2;
    h.seq = seq++;
    h.len = sizeof(LcdPayload);
    h.reserved = 0;
    h.crc32 = 0;

    send_packet(fd, dst, h, (const uint8_t*)&p);
    std::printf("Sent LCD line %d: %.*s\n", line, 20, p.text);
  }
  else {
    std::fprintf(stderr, "Bad args. Try:\n  %s led 255 0 0\n  %s lcd 0 \"Hello\"\n", argv[0], argv[0]);
    return 1;
  }

  ::close(fd);
  return 0;
}
