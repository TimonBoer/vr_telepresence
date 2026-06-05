#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// latency_collector_node
//
// MVP stap 1 van het latency-meetsysteem.
//
// Ontvangt op UDP-poort 5007 meet-records van de Quest in de vorm:
//   PROBE id=<probe_id> t_start_ns=<..> t_end_ns=<..> rtt_ms=<..>
//
// De Quest zet ZOWEL t_start als t_end op zijn eigen monotone klok. Deze node
// rekent dus alleen het verschil (de transport-latency van de software-keten)
// en telt daar de apart gemeten HARDWARE-CONSTANTEN bij op om tot een schatting
// van de volledige motion-to-photon te komen.
//
// BELANGRIJK (voor het verslag):
//   - software_latency_ms  = t_end - t_start   → live gemeten keten-transport
//   - total_latency_ms     = software + som(constanten)
//   De constanten hieronder moet je APART meten en hier invullen. Ze staan nu
//   op 0.0 zodat je eerst de zuivere software-meting ziet.
// ─────────────────────────────────────────────────────────────────────────────

// ── Hardware-constanten (apart te meten, daarna hier invullen) ───────────────
//   1) laptop B → Arduino: seriële overdracht + Arduino-loop (excl. servo-beweging)
//   2) ZED-sensor → beschikbaar in zed_sender (camera-capture latency)
//   3) servo-commando → eerste ZICHTBARE beweging in beeld (servo-dode-tijd)
// Alle waarden in milliseconden.
static constexpr double CONST_LAPTOP_TO_ARDUINO_MS = 0.0;
static constexpr double CONST_ZED_CAPTURE_MS       = 0.0;
static constexpr double CONST_SERVO_VISIBLE_MS     = 0.0;
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int    COLLECTOR_PORT   = 5007;
static constexpr size_t WINDOW_SIZE      = 200;   // samples voor live-statistiek

static double percentile(std::vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    double idx = (p / 100.0) * (static_cast<double>(v.size()) - 1.0);
    size_t lo = static_cast<size_t>(idx);
    size_t hi = std::min(lo + 1, v.size() - 1);
    double frac = idx - static_cast<double>(lo);
    return v[lo] * (1.0 - frac) + v[hi] * frac;
}

class LatencyCollectorNode : public rclcpp::Node
{
public:
    LatencyCollectorNode() : Node("latency_collector_node")
    {
        const double const_sum =
            CONST_LAPTOP_TO_ARDUINO_MS + CONST_ZED_CAPTURE_MS + CONST_SERVO_VISIBLE_MS;

        // CSV openen met tijdstempel in de naam.
        char fname[128];
        std::time_t t = std::time(nullptr);
        std::strftime(fname, sizeof(fname), "latency_%Y%m%d_%H%M%S.csv",
                      std::localtime(&t));
        csv_.open(fname);
        csv_ << "probe_id,software_ms,const_sum_ms,total_ms,rtt_ms\n";
        csv_path_ = fname;

        // UDP-socket openen.
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(COLLECTOR_PORT);
        bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));

        timeval tv{0, 1000}; // 1 ms — non-blocking-achtig pollen
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        RCLCPP_INFO(this->get_logger(),
            "Latency collector op UDP %d. Constanten-som = %.2f ms. CSV: %s",
            COLLECTOR_PORT, const_sum, csv_path_.c_str());
        if (const_sum == 0.0) {
            RCLCPP_WARN(this->get_logger(),
                "Constanten staan op 0 — je ziet nu PURE software-latency. "
                "Vul de hardware-constanten in zodra je ze gemeten hebt.");
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&LatencyCollectorNode::poll, this));
    }

    ~LatencyCollectorNode() override
    {
        if (sock_ >= 0) close(sock_);
        if (csv_.is_open()) csv_.close();
    }

private:
    void poll()
    {
        char buf[256];
        for (int i = 0; i < 64; ++i) {  // leeg de socketbuffer per tick
            ssize_t n = recv(sock_, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;
            buf[n] = '\0';
            parse_record(buf);
        }
    }

    void parse_record(const char * line)
    {
        unsigned id = 0;
        unsigned long long t_start = 0, t_end = 0;
        double rtt = 0.0;
        // PROBE id=.. t_start_ns=.. t_end_ns=.. rtt_ms=..
        int matched = std::sscanf(line,
            "PROBE id=%u t_start_ns=%llu t_end_ns=%llu rtt_ms=%lf",
            &id, &t_start, &t_end, &rtt);
        if (matched < 3) return;  // geen geldig record

        double software_ms = static_cast<double>(t_end - t_start) / 1.0e6;
        double const_sum =
            CONST_LAPTOP_TO_ARDUINO_MS + CONST_ZED_CAPTURE_MS + CONST_SERVO_VISIBLE_MS;
        double total_ms = software_ms + const_sum;

        // CSV-regel
        csv_ << id << ',' << software_ms << ',' << const_sum << ','
             << total_ms << ',' << rtt << '\n';
        csv_.flush();

        // Live venster voor statistiek
        window_.push_back(total_ms);
        if (window_.size() > WINDOW_SIZE) window_.pop_front();

        std::vector<double> v(window_.begin(), window_.end());
        double mn = *std::min_element(v.begin(), v.end());
        double mx = *std::max_element(v.begin(), v.end());
        double p50 = percentile(v, 50);
        double p95 = percentile(v, 95);
        double p99 = percentile(v, 99);
        // jitter = standaarddeviatie van het venster
        double mean = 0.0; for (double x : v) mean += x; mean /= v.size();
        double var = 0.0;  for (double x : v) var += (x - mean) * (x - mean);
        double jitter = v.size() > 1 ? std::sqrt(var / v.size()) : 0.0;

        RCLCPP_INFO(this->get_logger(),
            "probe=%u  sw=%.2f  total=%.2f ms | n=%zu p50=%.2f p95=%.2f p99=%.2f "
            "min=%.2f max=%.2f jitter=%.2f",
            id, software_ms, total_ms, v.size(), p50, p95, p99, mn, mx, jitter);
    }

    int sock_ = -1;
    std::ofstream csv_;
    std::string csv_path_;
    std::deque<double> window_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatencyCollectorNode>());
    rclcpp::shutdown();
    return 0;
}
