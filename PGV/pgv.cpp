// pgv_dashboard.cpp
// Build: g++ -std=c++17 -O2 -Wall -Wextra -o pgv_dashboard pgv_dashboard.cpp
// Run:   ./pgv_dashboard can1
//
// Requires: candump (can-utils) installed and accessible in PATH.

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
static constexpr uint8_t  NODE_ID   = 0x05;
static constexpr double   ANGLE_RES = 0.1;
static constexpr double   POS_RES   = 0.1;

static constexpr uint32_t TPDO2 = 0x280 + NODE_ID;
static constexpr uint32_t TPDO3 = 0x380 + NODE_ID;
static constexpr uint32_t TPDO4 = 0x480 + NODE_ID;
static constexpr uint32_t PDO5  = 0x680 + NODE_ID;

// ─────────────────────────────────────────
// Shared State (latest values)
// ─────────────────────────────────────────
struct State {
  double   x_position_mm = 0.0;
  uint32_t timestamp_ms  = 0;
  double   y_offset_mm   = 0.0;
  double   angle_deg     = 0.0;
  double   speed_mm_s    = 0.0;
  uint32_t tag_id        = 0;
};

static State g_state;

// ─────────────────────────────────────────
// Helpers: big-endian readers
// ─────────────────────────────────────────
static int16_t be_i16(const uint8_t* p) {
  return static_cast<int16_t>((p[0] << 8) | p[1]);
}

static uint16_t be_u16(const uint8_t* p) {
  return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

static int32_t be_i32(const uint8_t* p) {
  uint32_t u = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
  return static_cast<int32_t>(u);
}

static uint32_t be_u32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

static double round2(double x) {
  // mimic Python round(..., 2)
  return std::round(x * 100.0) / 100.0;
}

// ─────────────────────────────────────────
// Decoders (mirror your Python logic)
// ─────────────────────────────────────────
static void decode_tpdo2(const std::vector<uint8_t>& d) {
  if (d.size() < 6) return;
  int32_t y_raw     = be_i32(&d[0]);
  int16_t angle_raw = be_i16(&d[4]);
  g_state.y_offset_mm = round2(double(y_raw) * POS_RES);
  g_state.angle_deg   = round2(double(angle_raw) * ANGLE_RES);
}

static void decode_tpdo3(const std::vector<uint8_t>& d) {
  if (d.size() < 8) return;
  uint8_t tag = d[7];
  if (tag != 0) g_state.tag_id = tag;
}

static void decode_tpdo4(const std::vector<uint8_t>& d) {
  if (d.size() < 8) return;
  int32_t  x_raw = be_i32(&d[0]);
  uint32_t ts    = be_u32(&d[4]);
  g_state.x_position_mm = round2(double(x_raw) * POS_RES);
  g_state.timestamp_ms  = ts;
}

static void decode_pdo5(const std::vector<uint8_t>& d) {
  if (d.size() < 2) return;
  int16_t speed = be_i16(&d[0]);
  g_state.speed_mm_s = round2(double(speed) * 0.1);
}

// ─────────────────────────────────────────
// Dashboard
// ─────────────────────────────────────────
static void clear_screen() {
  // Like os.system("clear")
  std::cout << "\033[2J\033[H";  // ANSI clear + home
}

static void print_dashboard() {
  clear_screen();
  std::cout << "==============================================\n";
  std::cout << "      PGV100RS LIVE DASHBOARD\n";
  std::cout << "==============================================\n\n";

  std::cout << std::fixed << std::setprecision(2);

  std::cout << " X Position   : " << std::setw(8) << g_state.x_position_mm << " mm\n";
  std::cout << " Timestamp    : " << std::setw(8) << g_state.timestamp_ms  << " ms\n";
  std::cout << " Y Offset     : " << std::setw(8) << g_state.y_offset_mm   << " mm\n";
  std::cout << " Angle        : " << std::setw(8) << g_state.angle_deg     << " deg\n";
  std::cout << " Speed        : " << std::setw(8) << g_state.speed_mm_s    << " mm/s\n";
  std::cout << " Tag ID       : " << std::setw(8) << g_state.tag_id        << "\n";

  std::cout << "\nPress CTRL+C to stop.\n";
  std::cout.flush();
}

// ─────────────────────────────────────────
// candump line parsing
// Typical lines look like:
//   can1  385   [8]  00 00 00 00 00 00 00 69
// Sometimes with timestamps if candump -t* is used.
// We’ll robustly scan tokens and pick CAN ID token as the first hex-ish token after iface.
// ─────────────────────────────────────────
static bool is_hex_token(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s) {
    if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
  }
  return true;
}

static std::optional<uint32_t> parse_can_id_from_tokens(const std::vector<std::string>& toks) {
  // Prefer toks[1] if it looks like hex or decimal; otherwise search.
  auto parse_id = [](const std::string& t) -> std::optional<uint32_t> {
    // candump prints CAN IDs in hex without 0x (e.g., "385") but sometimes decimal-ish.
    // We'll treat it as hex if it contains A-F, else still parse as hex to match your Python int(...,16).
    // Python used int(parts[1],16).
    uint32_t id = 0;
    std::stringstream ss;
    ss << std::hex << t;
    ss >> id;
    if (ss.fail()) return std::nullopt;
    return id;
  };

  if (toks.size() >= 2) {
    auto id = parse_id(toks[1]);
    if (id.has_value()) return id;
  }
  // Fallback: search any token that is 3-8 hex digits and not "[8]"
  for (const auto& t : toks) {
    if (t.size() >= 2 && t.size() <= 8 && t != "[8]" && is_hex_token(t)) {
      auto id = parse_id(t);
      if (id.has_value()) return id;
    }
  }
  return std::nullopt;
}

static std::vector<uint8_t> parse_data_bytes(const std::vector<std::string>& toks) {
  // In your Python: bytes(int(x,16) for x in parts[3:])
  // candump format: iface, CANID, [DLC], b0 b1 ...
  // So we look for token like "[8]" then parse the following tokens as bytes.
  size_t start = std::string::npos;
  for (size_t i = 0; i < toks.size(); i++) {
    if (!toks[i].empty() && toks[i].front() == '[' && toks[i].back() == ']') {
      start = i + 1;
      break;
    }
  }
  if (start == std::string::npos) {
    // fallback: assume bytes start at index 3 like your script
    start = (toks.size() > 3) ? 3 : toks.size();
  }

  std::vector<uint8_t> data;
  for (size_t i = start; i < toks.size(); i++) {
    const auto& t = toks[i];
    if (t.size() != 2 || !is_hex_token(t)) continue;
    unsigned int v = 0;
    std::stringstream ss;
    ss << std::hex << t;
    ss >> v;
    if (!ss.fail() && v <= 0xFF) data.push_back(static_cast<uint8_t>(v));
  }
  return data;
}

static std::vector<std::string> split_ws(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> out;
  std::string tok;
  while (iss >> tok) out.push_back(tok);
  return out;
}

// ─────────────────────────────────────────
// Live dashboard
// ─────────────────────────────────────────
int main(int argc, char** argv) {
  std::string iface = "can1";
  if (argc > 1) iface = argv[1];

  // Start candump
  std::string cmd = "candump " + iface;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    std::cerr << "Failed to start candump. Is can-utils installed and in PATH?\n";
    return 1;
  }

  using clock = std::chrono::steady_clock;
  auto last_refresh = clock::now();

  char buf[4096];

  // Initial draw
  print_dashboard();

  while (std::fgets(buf, sizeof(buf), pipe)) {
    std::string line(buf);
    auto toks = split_ws(line);
    if (toks.size() < 3) continue;

    auto can_id_opt = parse_can_id_from_tokens(toks);
    if (!can_id_opt) continue;
    uint32_t can_id = *can_id_opt;

    auto data = parse_data_bytes(toks);

    if (can_id == TPDO2) {
      decode_tpdo2(data);
    } else if (can_id == TPDO3) {
      decode_tpdo3(data);
    } else if (can_id == TPDO4) {
      decode_tpdo4(data);
    } else if (can_id == PDO5) {
      decode_pdo5(data);
    }

    // Refresh at ~10 Hz
    auto now = clock::now();
    if (std::chrono::duration<double>(now - last_refresh).count() > 0.1) {
      print_dashboard();
      last_refresh = now;
    }
  }

  pclose(pipe);
  return 0;
}
