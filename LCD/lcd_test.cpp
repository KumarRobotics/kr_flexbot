#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

static constexpr const char* LCD_PORT = "/dev/ttymxc2";
static constexpr speed_t BAUDRATE = B57600;
static constexpr uint8_t CMD = 0xFE;

// Small helper: write all bytes (handles partial writes)
bool write_all(int fd, const uint8_t* data, size_t n) {
    while (n > 0) {
        ssize_t w = ::write(fd, data, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            std::cerr << "write() failed: " << std::strerror(errno) << "\n";
            return false;
        }
        data += static_cast<size_t>(w);
        n -= static_cast<size_t>(w);
    }
    return true;
}

void sleep_ms(int ms) { ::usleep(ms * 1000); }

bool lcd_cmd(int fd, uint8_t cmd) {
    uint8_t buf[2] = {CMD, cmd};
    if (!write_all(fd, buf, sizeof(buf))) return false;
    sleep_ms(20);
    return true;
}

bool lcd_goto(int fd, uint8_t pos) {
    uint8_t buf[3] = {CMD, 0x45, pos};
    if (!write_all(fd, buf, sizeof(buf))) return false;
    sleep_ms(20);
    return true;
}

bool lcd_write(int fd, const std::string& text) {
    // ASCII bytes (like the Python version). If you need strict ASCII validation,
    // you can add checks here.
    return write_all(fd,
                     reinterpret_cast<const uint8_t*>(text.data()),
                     text.size());
}

std::string ljust20(const std::string& s) {
    if (s.size() >= 20) return s.substr(0, 20);
    return s + std::string(20 - s.size(), ' ');
}

bool lcd_clear_all(int fd) {
    const uint8_t rows[] = {0x00, 0x40, 0x14, 0x54}; // 20x4 typical DDRAM starts
    for (uint8_t pos : rows) {
        if (!lcd_goto(fd, pos)) return false;
        if (!lcd_write(fd, std::string(20, ' '))) return false;
    }
    return true;
}

bool setup_serial(int fd) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << std::strerror(errno) << "\n";
        return false;
    }

    // Raw-ish mode: no line processing
    cfmakeraw(&tty);

    // 57600 baud
    if (cfsetispeed(&tty, BAUDRATE) != 0 || cfsetospeed(&tty, BAUDRATE) != 0) {
        std::cerr << "cfset*speed failed: " << std::strerror(errno) << "\n";
        return false;
    }

    // 8N1
    tty.c_cflag &= ~PARENB;            // no parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                // 8 data bits
    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem lines, enable receiver

    // Non-blocking read behavior like timeout=1 in pySerial:
    // VTIME is in deciseconds, VMIN=0 means "return as soon as any data or timeout"
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10; // 1.0 seconds

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << "\n";
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

int main() {
    int fd = ::open(LCD_PORT, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open " << LCD_PORT << ": "
                  << std::strerror(errno) << "\n";
        return 1;
    }

    if (!setup_serial(fd)) {
        ::close(fd);
        return 1;
    }

    // ---- TEST ----
    if (!lcd_cmd(fd, 0x41)) { ::close(fd); return 1; } // init
    sleep_ms(100);

    if (!lcd_clear_all(fd)) { ::close(fd); return 1; }

    if (!lcd_goto(fd, 0x00)) { ::close(fd); return 1; }
    if (!lcd_write(fd, ljust20("Dhyey is the BEST"))) { ::close(fd); return 1; }

    if (!lcd_goto(fd, 0x40)) { ::close(fd); return 1; }
    if (!lcd_write(fd, ljust20("Team UPenn !!!"))) { ::close(fd); return 1; }

    ::close(fd);
    return 0;
}
