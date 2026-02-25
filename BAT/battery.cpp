/**
 * Battery Monitor for Berkshire Robot
 * 63V 30Ah LiFePO4 Pack (15S3P) with Vecure BMS
 * CAN1 @ 125kbps
 * 
 * Build:
 *   g++ -std=c++17 -O2 -o battery_monitor battery_monitor.cpp -lpthread
 * 
 * Run:
 *   sudo ./battery_monitor
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <utility>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>

// ─── Configuration ───────────────────────────────────────────────────────────

static constexpr const char* CAN_INTERFACE     = "can1";
static constexpr uint32_t    CAN_ID_PRIMARY    = 0x1B1;
static constexpr uint32_t    CAN_ID_TEMP       = 0x4B1;
static constexpr double      NOMINAL_CAPACITY  = 30.0;  // Ah
static constexpr int         SERIES_CELLS      = 15;

// ─── LiFePO4 Voltage-to-SOC Lookup Table ────────────────────────────────────

struct VoltageSOCPoint {
    double voltage;
    double soc;
};

static constexpr VoltageSOCPoint VOLTAGE_SOC_TABLE[] = {
    {3.65, 100.0},  // Fully charged
    {3.40,  99.0},  // Start of flat region
    {3.35,  90.0},
    {3.32,  70.0},
    {3.30,  40.0},
    {3.27,  30.0},
    {3.25,  20.0},
    {3.20,  10.0},
    {3.10,   5.0},
    {3.00,   0.0},  // Empty (don't discharge below this!)
    {2.50,   0.0},  // Dead
};

static constexpr size_t VOLTAGE_SOC_TABLE_SIZE = 
    sizeof(VOLTAGE_SOC_TABLE) / sizeof(VOLTAGE_SOC_TABLE[0]);

// ─── Global State ────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};

// ─── Signal Handler ──────────────────────────────────────────────────────────

static void signal_handler(int sig) {
    (void)sig;
    g_running.store(false);
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
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("CAN bind()");
            return false;
        }
        
        // Set receive timeout
        struct timeval tv = {1, 0};  // 1 second timeout
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        return true;
    }
    
    bool recv(struct can_frame& frame) {
        ssize_t nbytes = ::read(fd_, &frame, sizeof(frame));
        return nbytes == (ssize_t)sizeof(frame);
    }

private:
    int fd_;
};

// ─── LiFePO4 SOC Calculator ──────────────────────────────────────────────────

class LiFePO4_SOC {
public:
    /**
     * Convert cell voltage to State of Charge percentage
     * Uses linear interpolation between lookup table points
     */
    static double voltage_to_soc(double cell_voltage) {
        // Clamp to table range
        if (cell_voltage >= VOLTAGE_SOC_TABLE[0].voltage)
            return 100.0;
        if (cell_voltage <= VOLTAGE_SOC_TABLE[VOLTAGE_SOC_TABLE_SIZE - 1].voltage)
            return 0.0;
        
        // Linear interpolation between table points
        for (size_t i = 0; i < VOLTAGE_SOC_TABLE_SIZE - 1; ++i) {
            double v_high = VOLTAGE_SOC_TABLE[i].voltage;
            double soc_high = VOLTAGE_SOC_TABLE[i].soc;
            double v_low = VOLTAGE_SOC_TABLE[i + 1].voltage;
            double soc_low = VOLTAGE_SOC_TABLE[i + 1].soc;
            
            if (v_low <= cell_voltage && cell_voltage <= v_high) {
                double ratio = (cell_voltage - v_low) / (v_high - v_low);
                return soc_low + ratio * (soc_high - soc_low);
            }
        }
        
        return 0.0;
    }
    
    /**
     * Get human-readable status message and emoji
     */
    static std::pair<const char*, const char*> get_status_message(double soc) {
        if (soc > 80.0)
            return {"Excellent", "🟢"};
        else if (soc > 50.0)
            return {"Good", "🟡"};
        else if (soc > 20.0)
            return {"Low - Charge Soon", "🟠"};
        else if (soc > 10.0)
            return {"Critical - Charge Now!", "🔴"};
        else
            return {"EMERGENCY - Return to Base!", "⚠️"};
    }
};

// ─── Battery Data Structures ─────────────────────────────────────────────────

struct BatteryPrimary {
    double voltage_v        = 0.0;
    double current_a        = 0.0;
    double power_w          = 0.0;
    double soc_pct          = 0.0;
    double cell_voltage     = 0.0;
    double capacity_ah      = 0.0;
    double runtime_min      = 0.0;
    bool   has_runtime      = false;
    bool   valid            = false;
};

struct BatteryTemperature {
    double temp1_c  = 0.0;
    double temp2_c  = 0.0;
    int    cycles   = 0;
    bool   valid    = false;
};

// ─── Decode Battery Messages ─────────────────────────────────────────────────

static BatteryPrimary decode_primary_status(const struct can_frame& frame) {
    BatteryPrimary data;
    
    if (frame.can_dlc < 8)
        return data;
    
    // Bytes 0-1: Voltage in 1/256V units
    uint16_t voltage_raw = (uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8);
    data.voltage_v = voltage_raw / 256.0;
    
    // Bytes 4-5: Current in 0.1A (signed)
    int16_t current_raw = (int16_t)frame.data[4] | ((int16_t)frame.data[5] << 8);
    data.current_a = current_raw / 10.0;
    
    data.power_w = data.voltage_v * data.current_a;
    
    // Calculate SOC using proper LiFePO4 curve
    data.cell_voltage = data.voltage_v / SERIES_CELLS;
    data.soc_pct = LiFePO4_SOC::voltage_to_soc(data.cell_voltage);
    
    // Estimate remaining capacity
    data.capacity_ah = (data.soc_pct / 100.0) * NOMINAL_CAPACITY;
    
    // Estimate runtime (if discharging)
    if (data.current_a < -0.1) {
        double runtime_hours = data.capacity_ah / fabs(data.current_a);
        data.runtime_min = runtime_hours * 60.0;
        data.has_runtime = true;
    } else {
        data.has_runtime = false;
    }
    
    data.valid = true;
    return data;
}

static BatteryTemperature decode_temperature(const struct can_frame& frame) {
    BatteryTemperature data;
    
    if (frame.can_dlc < 8)
        return data;
    
    // Bytes 0-1: Temp1 in 0.01°C
    uint16_t temp1_raw = (uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8);
    data.temp1_c = temp1_raw / 100.0;
    
    // Bytes 2-3: Temp2 in 0.01°C
    uint16_t temp2_raw = (uint16_t)frame.data[2] | ((uint16_t)frame.data[3] << 8);
    data.temp2_c = temp2_raw / 100.0;
    
    // Bytes 4-5: Cycle count
    uint16_t cycles_raw = (uint16_t)frame.data[4] | ((uint16_t)frame.data[5] << 8);
    data.cycles = cycles_raw;
    
    data.valid = true;
    return data;
}

// ─── Display Functions ───────────────────────────────────────────────────────

static void clear_screen() {
    printf("\033[2J\033[H");  // ANSI clear screen and move cursor to home
    fflush(stdout);
}

static void display_battery_status(const BatteryPrimary& primary,
                                   const BatteryTemperature& temp) {
    clear_screen();
    
    printf("======================================================================\n");
    printf("                       BATTERY STATUS\n");
    printf("======================================================================\n");
    printf("\n");
    
    if (primary.valid) {
        auto [status_msg, emoji] = LiFePO4_SOC::get_status_message(primary.soc_pct);
        
        printf("  SOC:              %5.1f %%  %s\n", primary.soc_pct, emoji);
        printf("  Status:           %s\n", status_msg);
        printf("\n");
        printf("  Pack Voltage:     %5.2f V\n", primary.voltage_v);
        printf("  Cell Voltage:     %5.3f V (avg)\n", primary.cell_voltage);
        printf("  Current:          %5.1f A", primary.current_a);
        
        if (primary.current_a < -0.1)
            printf(" (discharging)\n");
        else if (primary.current_a > 0.1)
            printf(" (charging)\n");
        else
            printf(" (idle)\n");
        
        printf("  Power:            %5.0f W\n", fabs(primary.power_w));
        printf("\n");
        printf("  Capacity Left:    %5.1f Ah (of %.1f Ah)\n", 
               primary.capacity_ah, NOMINAL_CAPACITY);
        
        if (primary.has_runtime) {
            if (primary.runtime_min > 60.0)
                printf("  Est. Runtime:     %5.1f hours\n", primary.runtime_min / 60.0);
            else
                printf("  Est. Runtime:     %5.0f minutes\n", primary.runtime_min);
        }
        
        printf("\n");
    } else {
        printf("  [Waiting for battery data...]\n\n");
    }
    
    if (temp.valid) {
        printf("  Temperature 1:    %5.1f °C\n", temp.temp1_c);
        printf("  Temperature 2:    %5.1f °C\n", temp.temp2_c);
        printf("  Charge Cycles:    %5d\n", temp.cycles);
    }
    
    printf("\n");
    printf("======================================================================\n");
    
    // Charging recommendations
    if (primary.valid) {
        if (primary.soc_pct < 20.0)
            printf("⚠️  CHARGE NOW - Battery critically low!\n");
        else if (primary.soc_pct < 30.0)
            printf("⚠️  Battery low - Charge when convenient\n");
        else if (primary.soc_pct > 95.0)
            printf("✓  Battery fully charged\n");
    }
    
    printf("\n");
    printf("Press Ctrl+C to stop\n");
    fflush(stdout);
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    // Install signal handlers
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("======================================================================\n");
    printf("  BERKSHIRE ROBOT - BATTERY MONITOR\n");
    printf("  63V 30Ah LiFePO4 Pack (15S3P)\n");
    printf("======================================================================\n");
    printf("\n");
    
    // Open CAN socket
    CANSocket can;
    if (!can.open(CAN_INTERFACE)) {
        fprintf(stderr, "[ERR] Failed to open CAN interface '%s'\n", CAN_INTERFACE);
        fprintf(stderr, "      Make sure CAN1 is configured:\n");
        fprintf(stderr, "      sudo ip link set can1 type can bitrate 125000\n");
        fprintf(stderr, "      sudo ip link set can1 up\n");
        return 1;
    }
    
    printf("[CAN] Opened %s @ 125kbps\n", CAN_INTERFACE);
    printf("[INFO] Listening for battery messages...\n\n");
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    BatteryPrimary primary;
    BatteryTemperature temp;
    
    auto last_display = std::chrono::steady_clock::now();
    
    while (g_running.load()) {
        struct can_frame frame;
        
        if (can.recv(frame)) {
            // Decode relevant messages
            if (frame.can_id == CAN_ID_PRIMARY) {
                primary = decode_primary_status(frame);
            }
            else if (frame.can_id == CAN_ID_TEMP) {
                temp = decode_temperature(frame);
            }
            
            // Update display every second
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_display).count();
            
            if (elapsed >= 1000) {
                display_battery_status(primary, temp);
                last_display = now;
            }
        }
    }
    
    printf("\n\n✓ Shutdown complete\n");
    return 0;
}
