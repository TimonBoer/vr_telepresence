#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// video_latency_collector_node  (bijgewerkte versie)
//
// Ontvangt op UDP 5009 records van zed_sender:
//   VIDEO id=<id> rtt_ms=<..>
//
// rtt = t_end - t_start, beide op de LAPTOP-klok in zed_sender.
// Dit meet: buffer-vertraging + encode + transport + decode + render +
//           Quest→laptop echo-UDP (~1-3ms, verwaarloosbaar).
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
        bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));

        timeval tv{0, 1000};
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        RCLCPP_INFO(this->get_logger(),
            "Video-latency collector op UDP %d. CSV: %s",
            COLLECTOR_PORT, csv_path_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Meet: buffer + encode + transport + decode + render (laptop-klok)");

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

    void parse_record(const char* line)
    {
        unsigned id = 0;
        double rtt_ms = 0.0;
        // Nieuw formaat: VIDEO id=.. rtt_ms=..
        if (std::sscanf(line, "VIDEO id=%u rtt_ms=%lf", &id, &rtt_ms) < 2)
            return;

        if (rtt_ms > 5000.0) return;  // stale probe

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
        double mean = 0.0; for (double x : v) mean += x; mean /= v.size();
        double var  = 0.0; for (double x : v) var += (x-mean)*(x-mean);
        double jitter = v.size() > 1 ? std::sqrt(var/v.size()) : 0.0;

        RCLCPP_INFO(this->get_logger(),
            "video id=%u  rtt=%.2f ms | n=%zu min=%.2f p5=%.2f "
            "p50=%.2f p95=%.2f max=%.2f jitter=%.2f",
            id, rtt_ms, v.size(), mn, p5, p50, p95, mx, jitter);
    }

    int sock_ = -1;
    std::ofstream csv_;
    std::string csv_path_;
    std::deque<double> window_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoLatencyCollectorNode>());
    rclcpp::shutdown();
    return 0;
}