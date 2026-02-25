#include <cerrno>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <cmath>

// CAN for battery
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// ─── Configuration ───────────────────────────────────────────────────────────

static constexpr const char* LCD_PORT        = "/dev/ttymxc2";
static constexpr speed_t     BAUDRATE        = B57600;
static constexpr const char* CAN_INTERFACE   = "can1";
static constexpr const char* WIFI_INTERFACE  = "wlp1s0";
static constexpr uint32_t    BATTERY_CAN_ID  = 0x1B1;

// LCD refresh rate
static constexpr int         UPDATE_RATE_HZ  = 2;

// Battery assumptions (for smooth SOC via coulomb counting)
// If you know your pack capacity, set this correctly.
static constexpr double      PACK_CAPACITY_AH = 20.0;   // <-- CHANGE to your real pack Ah
static constexpr int         SERIES_CELLS     = 15;     // 15S assumption (used for voltage->soc fallback)

// ─── LCD Commands ────────────────────────────────────────────────────────────
static constexpr uint8_t CMD = 0xFE;

// ─── Global State ────────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};

struct BatteryStatus {
    double voltage_v = 0.0;
    double current_a = 0.0;   // + means charging, - means discharging (depends on your BMS convention)
    double power_w   = 0.0;
    double soc_pct   = 0.0;   // smoothed SOC estimate
    bool   valid     = false;
};

struct WiFiStatus {
    std::string ssid;
    std::string ip;
    bool connected = false;   // IP-based
};

// ─── Signal Handler ──────────────────────────────────────────────────────────
static void signal_handler(int sig) {
    (void)sig;
    g_running.store(false);
}

// ─── LCD Functions ───────────────────────────────────────────────────────────

static bool write_all(int fd, const uint8_t* data, size_t n) {
    while (n > 0) {
        ssize_t w = ::write(fd, data, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        data += w;
        n -= static_cast<size_t>(w);
    }
    return true;
}

static void sleep_ms(int ms) { ::usleep(static_cast<useconds_t>(ms) * 1000); }

static bool lcd_cmd(int fd, uint8_t cmd) {
    uint8_t buf[2] = {CMD, cmd};
    if (!write_all(fd, buf, 2)) return false;
    sleep_ms(20);
    return true;
}

static bool lcd_goto(int fd, uint8_t pos) {
    uint8_t buf[3] = {CMD, 0x45, pos};
    if (!write_all(fd, buf, 3)) return false;
    sleep_ms(20);
    return true;
}

static bool lcd_write(int fd, const std::string& text) {
    return write_all(fd, reinterpret_cast<const uint8_t*>(text.data()), text.size());
}

static std::string pad20(const std::string& s) {
    if (s.size() >= 20) return s.substr(0, 20);
    return s + std::string(20 - s.size(), ' ');
}

static bool setup_serial(int fd) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) return false;

    cfmakeraw(&tty);
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
    tcflush(fd, TCIOFLUSH);
    return true;
}

static bool lcd_init(int fd) {
    if (!lcd_cmd(fd, 0x41)) return false;  // Init
    sleep_ms(100);

    const uint8_t rows[] = {0x00, 0x40, 0x14, 0x54};
    for (uint8_t pos : rows) {
        if (!lcd_goto(fd, pos)) return false;
        if (!lcd_write(fd, std::string(20, ' '))) return false;
    }
    return true;
}

// ─── Battery Reading ─────────────────────────────────────────────────────────

// Fallback SOC from per-cell voltage (coarse)
static double voltage_to_soc_lifepo4(double cell_voltage) {
    if (cell_voltage >= 3.40) return 100.0;
    if (cell_voltage >= 3.32) return 70.0 + ((cell_voltage - 3.32) / 0.08) * 30.0;
    if (cell_voltage >= 3.25) return 20.0 + ((cell_voltage - 3.25) / 0.07) * 50.0;
    if (cell_voltage >= 3.00) return ((cell_voltage - 3.00) / 0.25) * 20.0;
    return 0.0;
}

// Read one CAN frame with ID BATTERY_CAN_ID and decode voltage/current
static BatteryStatus read_battery_raw() {
    BatteryStatus status;

    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return status;

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd);
        return status;
    }

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return status;
    }

    timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
        can_frame frame{};
        ssize_t r = ::read(fd, &frame, sizeof(frame));
        if (r == static_cast<ssize_t>(sizeof(frame))) {
            if (frame.can_id == BATTERY_CAN_ID && frame.can_dlc >= 8) {
                // Voltage in 1/256V
                uint16_t v_raw = static_cast<uint16_t>(frame.data[0]) |
                                 (static_cast<uint16_t>(frame.data[1]) << 8);
                status.voltage_v = static_cast<double>(v_raw) / 256.0;

                // Current in 0.1A (signed)
                int16_t i_raw = static_cast<int16_t>(
                    static_cast<uint16_t>(frame.data[4]) |
                    (static_cast<uint16_t>(frame.data[5]) << 8)
                );
                status.current_a = static_cast<double>(i_raw) / 10.0;

                status.power_w = status.voltage_v * status.current_a;

                // Coarse voltage-based SOC fallback
                double cell_v = status.voltage_v / static_cast<double>(SERIES_CELLS);
                status.soc_pct = voltage_to_soc_lifepo4(cell_v);

                status.valid = true;
                break;
            }
        }
    }

    ::close(fd);
    return status;
}

// Coulomb counter state for smooth SOC
struct SocFilter {
    bool initialized = false;
    double soc_pct = 0.0;
    std::chrono::steady_clock::time_point last_t{};
};

static double clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Update smooth SOC using current integration, anchored slowly to voltage-based SOC
static double update_smooth_soc(SocFilter& f, double soc_from_voltage, double current_a) {
    using clock = std::chrono::steady_clock;
    auto now = clock::now();

    if (!f.initialized) {
        f.soc_pct = soc_from_voltage;
        f.last_t = now;
        f.initialized = true;
        return f.soc_pct;
    }

    const double dt_s = std::chrono::duration<double>(now - f.last_t).count();
    f.last_t = now;

    // Integrate current -> delta Ah
    // Convention: if current_a is negative while discharging, SOC should go down.
    const double dAh = (current_a * dt_s) / 3600.0;  // Ah
    const double dSoc = (dAh / PACK_CAPACITY_AH) * 100.0;

    // Apply coulomb counting
    f.soc_pct = f.soc_pct + dSoc;

    // Softly correct drift towards voltage-based estimate (very small blending)
    // This prevents runaway if current sign/scale is imperfect.
    const double alpha = 0.02;  // 2% correction per update
    f.soc_pct = (1.0 - alpha) * f.soc_pct + alpha * soc_from_voltage;

    f.soc_pct = clamp(f.soc_pct, 0.0, 100.0);
    return f.soc_pct;
}

// Public battery read: raw + smooth SOC
static BatteryStatus read_battery() {
    static SocFilter soc_filter{};

    BatteryStatus s = read_battery_raw();
    if (!s.valid) return s;

    // Replace coarse SOC with smooth SOC
    s.soc_pct = update_smooth_soc(soc_filter, s.soc_pct, s.current_a);
    return s;
}

// ─── WiFi Status ─────────────────────────────────────────────────────────────

static std::string exec_command(const std::string& cmd) {
    char buffer[256];
    std::string result;
    FILE* pipe = ::popen(cmd.c_str(), "r");
    if (!pipe) return "";
    while (::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }
    ::pclose(pipe);
    return result;
}

static inline void rstrip_newlines(std::string& s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
}

static bool file_read_one_line(const std::string& path, std::string& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::getline(f, out);
    rstrip_newlines(out);
    return true;
}

// Parse first IPv4 from: ip -4 -o addr show dev <if>
static std::string first_ipv4_on_iface(const std::string& ifname) {
    std::string out = exec_command("ip -4 -o addr show dev " + ifname + " 2>/dev/null");
    if (out.empty()) return "";
    auto pos = out.find(" inet ");
    if (pos == std::string::npos) return "";
    pos += 6;
    auto end = out.find('/', pos);
    if (end == std::string::npos) return "";
    return out.substr(pos, end - pos);
}

// SSID getter: prefer iw; fallback iwgetid. If both missing => ""
static std::string get_ssid(const std::string& ifname) {
    std::string out = exec_command("iw dev " + ifname + " link 2>/dev/null");
    if (!out.empty()) {
        if (out.find("Not connected") != std::string::npos) return "";
        std::istringstream iss(out);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.rfind("SSID:", 0) == 0) {
                std::string ssid = line.substr(5);
                while (!ssid.empty() && ssid.front() == ' ') ssid.erase(ssid.begin());
                rstrip_newlines(ssid);
                return ssid;
            }
        }
    }

    std::string ssid = exec_command("iwgetid -r 2>/dev/null");
    rstrip_newlines(ssid);
    return ssid;
}

static WiFiStatus read_wifi() {
    WiFiStatus status;
    const std::string ifname = WIFI_INTERFACE;

    // operstate (up/down)
    std::string oper;
    bool have_oper = file_read_one_line("/sys/class/net/" + ifname + "/operstate", oper);
    bool up = have_oper && (oper == "up");

    status.ip   = first_ipv4_on_iface(ifname);
    status.ssid = get_ssid(ifname);

    // IMPORTANT: connected is IP-based (so you keep IP/connected even if SSID tools fail)
    status.connected = up && !status.ip.empty();

    return status;
}

// ─── Display Update ──────────────────────────────────────────────────────────

static void update_display(int lcd_fd, const BatteryStatus& battery, const WiFiStatus& wifi) {
    char line[64];

    // Line 1: Battery SOC
    if (battery.valid) {
        const char* emoji = (battery.soc_pct > 80) ? "OK" :
                            (battery.soc_pct > 50) ? "OK" :
                            (battery.soc_pct > 20) ? "LO" : "!!";
        std::snprintf(line, sizeof(line), "Battery: %3.0f%% %s", battery.soc_pct, emoji);
    } else {
        std::snprintf(line, sizeof(line), "Battery: ---");
    }
    lcd_goto(lcd_fd, 0x00);
    lcd_write(lcd_fd, pad20(line));

    // Line 2: V, A, W
    if (battery.valid) {
        std::snprintf(line, sizeof(line), "%.1fV %+.1fA %3.0fW",
                      battery.voltage_v, battery.current_a, std::fabs(battery.power_w));
    } else {
        std::snprintf(line, sizeof(line), "---");
    }
    lcd_goto(lcd_fd, 0x40);
    lcd_write(lcd_fd, pad20(line));

    // Line 3: MUST show SSID or "Not connected"
    if (!wifi.ssid.empty()) {
        std::snprintf(line, sizeof(line), "SSID: %s", wifi.ssid.c_str());
    } else {
        std::snprintf(line, sizeof(line), "SSID: Not connected");
    }
    lcd_goto(lcd_fd, 0x14);
    lcd_write(lcd_fd, pad20(line));

    // Line 4: Show IP if present (even if SSID couldn't be read)
    if (!wifi.ip.empty()) {
        std::snprintf(line, sizeof(line), "IP: %s", wifi.ip.c_str());
    } else {
        std::snprintf(line, sizeof(line), "IP: Not connected");
    }
    lcd_goto(lcd_fd, 0x54);
    lcd_write(lcd_fd, pad20(line));
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    ::signal(SIGINT,  signal_handler);
    ::signal(SIGTERM, signal_handler);

    int lcd_fd = ::open(LCD_PORT, O_RDWR | O_NOCTTY);
    if (lcd_fd < 0) {
        std::fprintf(stderr, "Failed to open LCD (%s): %s\n", LCD_PORT, std::strerror(errno));
        return 1;
    }

    if (!setup_serial(lcd_fd)) {
        std::fprintf(stderr, "Failed to setup serial on %s\n", LCD_PORT);
        ::close(lcd_fd);
        return 1;
    }

    if (!lcd_init(lcd_fd)) {
        std::fprintf(stderr, "Failed to init LCD\n");
        ::close(lcd_fd);
        return 1;
    }

    auto interval    = std::chrono::milliseconds(1000 / UPDATE_RATE_HZ);
    auto next_update = std::chrono::steady_clock::now() + interval;

    while (g_running.load()) {
        BatteryStatus battery = read_battery();
        WiFiStatus wifi       = read_wifi();

        update_display(lcd_fd, battery, wifi);

        std::this_thread::sleep_until(next_update);
        next_update += interval;
    }

    // Exit message
    lcd_init(lcd_fd);
    lcd_goto(lcd_fd, 0x00);
    lcd_write(lcd_fd, pad20("LCD Service OFF"));

    ::close(lcd_fd);
    return 0;
}

