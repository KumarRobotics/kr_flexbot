// udp_cmd_server.cpp  (run on IMX7)
// Listens on UDP :5006 for LED + LCD commands.

#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <termios.h>

static constexpr int LISTEN_PORT = 5006;

// LED sysfs
static constexpr const char* LED_R = "/sys/class/leds/red:status/brightness";
static constexpr const char* LED_G = "/sys/class/leds/green:status/brightness";
static constexpr const char* LED_B = "/sys/class/leds/blue:status/brightness";

// LCD serial
static constexpr const char* LCD_PORT = "/dev/ttymxc2";
static constexpr speed_t LCD_BAUD = B57600;
static constexpr uint8_t LCD_CMD = 0xFE;

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

static bool write_file_u8(const char* path, uint8_t v) {
  int fd = ::open(path, O_WRONLY);
  if (fd < 0) { std::perror(path); return false; }
  char buf[8];
  int n = std::snprintf(buf, sizeof(buf), "%u", (unsigned)v);
  bool ok = (::write(fd, buf, n) == n);
  ::close(fd);
  return ok;
}

static void sleep_ms(int ms) { ::usleep(ms * 1000); }

static bool write_all(int fd, const uint8_t* data, size_t n) {
  while (n > 0) {
    ssize_t w = ::write(fd, data, n);
    if (w < 0) { if (errno == EINTR) continue; return false; }
    data += (size_t)w;
    n -= (size_t)w;
  }
  return true;
}

static bool lcd_setup(int fd) {
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;
  cfmakeraw(&tty);
  if (cfsetispeed(&tty, LCD_BAUD) != 0 || cfsetospeed(&tty, LCD_BAUD) != 0) return false;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 10; // 1s
  if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
  tcflush(fd, TCIOFLUSH);
  return true;
}

static bool lcd_cmd(int fd, uint8_t cmd) {
  uint8_t buf[2] = {LCD_CMD, cmd};
  if (!write_all(fd, buf, sizeof(buf))) return false;
  sleep_ms(20);
  return true;
}

static bool lcd_goto(int fd, uint8_t pos) {
  uint8_t buf[3] = {LCD_CMD, 0x45, pos};
  if (!write_all(fd, buf, sizeof(buf))) return false;
  sleep_ms(20);
  return true;
}

static uint8_t line_to_pos(uint8_t line) {
  // 20x4 typical
  switch (line) {
    case 0: return 0x00;
    case 1: return 0x40;
    case 2: return 0x14;
    case 3: return 0x54;
    default: return 0x00;
  }
}

static bool lcd_write20(int fd, uint8_t line, const char text20[20]) {
  if (!lcd_goto(fd, line_to_pos(line))) return false;
  return write_all(fd, reinterpret_cast<const uint8_t*>(text20), 20);
}

#pragma pack(push, 1)
struct CmdHeader {
  uint32_t magic;   // "CMDS" = 0x53444D43
  uint16_t version; // 1
  uint16_t cmd_id;  // 1=LED, 2=LCD
  uint32_t seq;
  uint16_t len;     // payload bytes
  uint16_t reserved;
  uint32_t crc32;
};

struct LedPayload {
  uint8_t r, g, b, _pad;
};

struct LcdPayload {
  uint8_t line;     // 0..3
  char text[20];    // exactly 20 bytes
};
#pragma pack(pop)

int main() {
  // LCD open (optional: still allow LED commands if LCD fails)
  int lcd_fd = ::open(LCD_PORT, O_RDWR | O_NOCTTY);
  bool lcd_ok = (lcd_fd >= 0) && lcd_setup(lcd_fd);
  if (lcd_fd < 0) std::perror("open lcd");
  if (lcd_fd >= 0 && !lcd_ok) std::fprintf(stderr, "LCD setup failed, continuing (LED only ok)\n");

  if (lcd_ok) {
    // init command (matches your test)
    lcd_cmd(lcd_fd, 0x41);
    sleep_ms(100);
  }

  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) { std::perror("socket"); return 1; }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(LISTEN_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::perror("bind");
    return 1;
  }

  std::printf("IMX7 UDP command server listening on 0.0.0.0:%d\n", LISTEN_PORT);

  alignas(8) uint8_t buf[512];

  while (true) {
    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &slen);
    if (n < (ssize_t)sizeof(CmdHeader)) continue;

    CmdHeader hdr{};
    std::memcpy(&hdr, buf, sizeof(CmdHeader));
    if (hdr.magic != 0x53444D43u || hdr.version != 1) continue;
    if ((size_t)n != sizeof(CmdHeader) + hdr.len) continue;

    uint32_t rx_crc = hdr.crc32;
    hdr.crc32 = 0;

    // compute crc over header(with crc=0) + payload
    uint8_t tmp[sizeof(CmdHeader)];
    std::memcpy(tmp, &hdr, sizeof(CmdHeader));
    uint32_t calc = crc32(tmp, sizeof(CmdHeader));
    calc = crc32(buf + sizeof(CmdHeader), hdr.len) ^ calc; 
    uint8_t stitched[sizeof(CmdHeader) + 256];
    if (hdr.len > 256) continue;
    std::memcpy(stitched, &hdr, sizeof(CmdHeader));
    std::memcpy(stitched + sizeof(CmdHeader), buf + sizeof(CmdHeader), hdr.len);
    calc = crc32(stitched, sizeof(CmdHeader) + hdr.len);

    if (calc != rx_crc) {
      std::fprintf(stderr, "Bad CRC (seq=%u)\n", hdr.seq);
      continue;
    }

    if (hdr.cmd_id == 1 && hdr.len == sizeof(LedPayload)) {
      LedPayload p{};
      std::memcpy(&p, buf + sizeof(CmdHeader), sizeof(p));
      write_file_u8(LED_R, p.r);
      write_file_u8(LED_G, p.g);
      write_file_u8(LED_B, p.b);
      std::printf("LED set: r=%u g=%u b=%u\n", p.r, p.g, p.b);
    } else if (hdr.cmd_id == 2 && hdr.len == sizeof(LcdPayload)) {
      if (!lcd_ok) { std::fprintf(stderr, "LCD not available\n"); continue; }
      LcdPayload p{};
      std::memcpy(&p, buf + sizeof(CmdHeader), sizeof(p));
      lcd_write20(lcd_fd, p.line, p.text);
      std::printf("LCD line %u updated\n", p.line);
    }
  }

  if (lcd_fd >= 0) ::close(lcd_fd);
  ::close(fd);
  return 0;
}
