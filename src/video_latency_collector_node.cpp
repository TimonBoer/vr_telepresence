#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// video_latency_collector_node
//
// Ontvangt op UDP 5009 de videoweg-records van de Quest:
//   VIDEO id=<id> t_start_ns=<..> t_end_ns=<..> rtt_ms=<..>
//
// rtt = t_end - t_start, beide op de Quest-klok (clock-free). Dit is de tijd
// van "Quest stuurt probe_id naar zed_sender" tot "Quest leest het terug uit
// het gerenderde beeld", dus de videoweg laptop B -> Quest.
//
// LET OP bij interpretatie:
//   - de sender tekent een binnengekomen probe_id pas in het VOLGENDE grab-
//     frame -> 0..33 ms frame-fase (uniform) bovenop de echte transportlatency;
//   - t_end is renderklaar, ~<1 frame vóór de fotonen;
//   - de kleine Quest->sender UDP-hop zit er ook in.
// De MINIMUM en p5 benaderen daarom de zuivere videoweg het best; de externe
// camera blijft de absolute grondwaarheid.
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int    COLLECTOR_PORT = 5009;
static constexpr size_t WINDOW_SIZE    = 200;

static double percentile(std::vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    double idx  = (p / 100.0) * (static_cast<double>(v.size()) - 1.0);
    size_t lo   = static_cast<size_t>(idx);
    size_t hi   = std::min(lo + 1, v.size() - 1);
    double frac = idx - static_cast<double>(lo);
    return v[lo] * (1.0 - frac) + v[hi] * frac;
}

class VideoLatencyCollectorNode : public rclcpp::Node
{
public:
    VideoLatencyCollectorNode() : Node("video_latency_collector_node")
    {
        char fname[128];
        std::time_t t = std::time(nullptr);
        std::strftime(fname, sizeof(fname), "video_latency_%Y%m%d_%H%M%S.csv",
                      std::localtime(&t));
        csv_.open(fname);
        csv_ << "probe_id,rtt_ms\n";
        csv_path_ = fname;

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(COLLECTOR_PORT);
        bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));

        timeval tv{0, 1000};
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        RCLCPP_INFO(this->get_logger(),
            "Video-latency collector op UDP %d. CSV: %s",
            COLLECTOR_PORT, csv_path_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Tip: kijk naar MIN/p5 voor de zuivere videoweg (p95/max bevat de "
            "0-33 ms frame-fase van de 30 fps-bron).");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&VideoLatencyCollectorNode::poll, this));
    }

    ~VideoLatencyCollectorNode() override
    {
        if (sock_ >= 0) close(sock_);
        if (csv_.is_open()) csv_.close();
    }

private:
    void poll()
    {
        char buf[256];
        for (int i = 0; i < 64; ++i) {
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
        double rtt_field = 0.0;
        int matched = std::sscanf(line,
            "VIDEO id=%u t_start_ns=%llu t_end_ns=%llu rtt_ms=%lf",
            &id, &t_start, &t_end, &rtt_field);
        if (matched < 3) return;

        double rtt_ms = static_cast<double>(t_end - t_start) / 1.0e6;

        csv_ << id << ',' << rtt_ms << '\n';
        csv_.flush();

        window_.push_back(rtt_ms);
        if (window_.size() > WINDOW_SIZE) window_.pop_front();

        std::vector<double> v(window_.begin(), window_.end());
        double mn  = *std::min_element(v.begin(), v.end());
        double mx  = *std::max_element(v.begin(), v.end());
        double p5  = percentile(v, 5);
        double p50 = percentile(v, 50);
        double p95 = percentile(v, 95);

        RCLCPP_INFO(this->get_logger(),
            "video id=%u  rtt=%.2f ms | n=%zu min=%.2f p5=%.2f p50=%.2f "
            "p95=%.2f max=%.2f",
            id, rtt_ms, v.size(), mn, p5, p50, p95, mx);
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
    rclcpp::spin(std::make_shared<VideoLatencyCollectorNode>());
    rclcpp::shutdown();
    return 0;
}
