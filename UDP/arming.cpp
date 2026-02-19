#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <openssl/hmac.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ---------------- CONFIG ----------------
static const char* LISTEN_IP = "0.0.0.0";
static const int   LISTEN_PORT = 5000;

static const char* ALLOWED_SRC_IP = "192.168.0.20";

// IMPORTANT: change this to a long random secret and keep same on companion
static const std::string SECRET = "CHANGE_ME_TO_RANDOM_LONG_SECRET";

static const char* INIT_SCRIPT  = "/home/bg_bot/UDP/init_motors.sh";
static const char* START_SCRIPT = "/home/bg_bot/UDP/start_comm.sh";

// Anti-replay window (seconds)
static const int MAX_SKEW_SEC = 5;
// ---------------------------------------

static long unix_time_sec() {
  using namespace std::chrono;
  return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
}

static std::string hex_lower(const unsigned char* data, size_t len) {
  static const char* hexd = "0123456789abcdef";
  std::string out;
  out.resize(len * 2);
  for (size_t i = 0; i < len; i++) {
    out[2*i]   = hexd[(data[i] >> 4) & 0xF];
    out[2*i+1] = hexd[(data[i]     ) & 0xF];
  }
  return out;
}

static bool constant_time_eq(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  unsigned char r = 0;
  for (size_t i = 0; i < a.size(); i++) r |= (unsigned char)(a[i] ^ b[i]);
  return r == 0;
}

static std::string hmac_sha256_hex(const std::string& key, const std::string& msg) {
  unsigned int out_len = 0;
  unsigned char out[EVP_MAX_MD_SIZE];

  HMAC(EVP_sha256(),
       key.data(), (int)key.size(),
       reinterpret_cast<const unsigned char*>(msg.data()), msg.size(),
       out, &out_len);

  return hex_lower(out, out_len);
}

static std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> parts;
  std::string cur;
  std::stringstream ss(s);
  while (std::getline(ss, cur, delim)) parts.push_back(cur);
  return parts;
}

static int run_script(const char* path) {
  // Use /bin/sh explicitly (no need for exec bits)
  std::string cmd = std::string("/bin/sh ") + path;
  std::cout << "[RUN] " << cmd << std::endl;
  int rc = std::system(cmd.c_str());
  if (rc == -1) {
    std::cerr << "[ERR] system() failed: " << strerror(errno) << std::endl;
    return 1;
  }
  // system() returns shell status; normalize a bit
  int exit_code = WIFEXITED(rc) ? WEXITSTATUS(rc) : 1;
  std::cout << "[DONE] " << path << " rc=" << exit_code << std::endl;
  return exit_code;
}

int main() {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    std::cerr << "socket() failed: " << strerror(errno) << std::endl;
    return 1;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(LISTEN_PORT);
  if (inet_pton(AF_INET, LISTEN_IP, &addr.sin_addr) != 1) {
    std::cerr << "inet_pton(LISTEN_IP) failed\n";
    close(fd);
    return 1;
  }

  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "bind() failed: " << strerror(errno) << std::endl;
    close(fd);
    return 1;
  }

  std::cout << "[OK] Listening UDP " << LISTEN_IP << ":" << LISTEN_PORT << "\n";
  std::cout << "[OK] Allowing only src " << ALLOWED_SRC_IP << "\n";

  bool armed = false;

  while (true) {
    char buf[2048];
    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    ssize_t n = recvfrom(fd, buf, sizeof(buf)-1, 0, reinterpret_cast<sockaddr*>(&src), &slen);
    if (n < 0) {
      std::cerr << "[ERR] recvfrom(): " << strerror(errno) << std::endl;
      continue;
    }
    buf[n] = '\0';

    char src_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &src.sin_addr, src_ip, sizeof(src_ip));
    int src_port = ntohs(src.sin_port);

    std::string msg(buf);
    // trim trailing newlines/spaces
    while (!msg.empty() && (msg.back()=='\n' || msg.back()=='\r' || msg.back()==' ' || msg.back()=='\t'))
      msg.pop_back();

    if (std::string(src_ip) != ALLOWED_SRC_IP) {
      std::cout << "[DROP] from " << src_ip << ":" << src_port << " msg=" << msg << "\n";
      continue;
    }

    // Format: CMD|ts|hexsig
    auto parts = split(msg, '|');
    if (parts.size() != 3) {
      std::cout << "[BAD] format from " << src_ip << ": " << msg << "\n";
      continue;
    }

    const std::string& cmd = parts[0];
    const std::string& ts_s = parts[1];
    const std::string& sig = parts[2];

    if (!(cmd == "ARM" || cmd == "STOP")) {
      std::cout << "[BAD] cmd " << cmd << " from " << src_ip << "\n";
      continue;
    }

    long ts = 0;
    try {
      ts = std::stol(ts_s);
    } catch (...) {
      std::cout << "[BAD] ts " << ts_s << " from " << src_ip << "\n";
      continue;
    }

    long now = unix_time_sec();
    if (std::labs(now - ts) > MAX_SKEW_SEC) {
      std::cout << "[DROP] stale ts now=" << now << " ts=" << ts << "\n";
      continue;
    }

    std::string base = cmd + "|" + ts_s;
    std::string expected = hmac_sha256_hex(SECRET, base);

    if (!constant_time_eq(expected, sig)) {
      std::cout << "[DROP] bad hmac from " << src_ip << " cmd=" << cmd << "\n";
      continue;
    }

    if (cmd == "ARM") {
      if (armed) {
        std::cout << "[INFO] already armed; ignoring ARM\n";
        continue;
      }
      std::cout << "[ARM] Trigger accepted. Running init + start_comm...\n";

      int rc1 = run_script(INIT_SCRIPT);
      if (rc1 != 0) {
        std::cout << "[ERR] init_motors failed; not starting comm\n";
        continue;
      }
      int rc2 = run_script(START_SCRIPT);
      if (rc2 == 0) {
        armed = true;
        std::cout << "[ARM] Motors comm started.\n";
      } else {
        std::cout << "[ERR] start_comm failed.\n";
      }
    } else { // STOP
      std::cout << "[STOP] Trigger accepted. (No stop action implemented here.)\n";
      // If you have a stop script, call it here.
      // run_script("/home/bg_bot/UDP/stop_comm.sh");
    }
  }

  close(fd);
  return 0;
}
