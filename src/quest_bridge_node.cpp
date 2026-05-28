#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <regex>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// quest_bridge_node
//
// Opent een TCP server op poort 5005.
// De Meta Quest app verbindt en stuurt JSON regels in de vorm:
//   {"x": 0.0, "y": 0.1, "z": 0.0, "w": 0.99}
//
// De node publiceert de quaternion op:
//   /head/orientation  (geometry_msgs/QuaternionStamped)
// ─────────────────────────────────────────────────────────────────────────────

class QuestBridgeNode : public rclcpp::Node
{
public:
    QuestBridgeNode() : Node("quest_bridge_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "/head/orientation", 10);

        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(5005);

        bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
        listen(server_fd_, 1);

        RCLCPP_INFO(this->get_logger(),
            "TCP server luistert op poort 5005 — wachten op Quest verbinding...");

        // Poll elke 10 ms voor nieuwe TCP data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&QuestBridgeNode::receive_tcp, this));
    }

    ~QuestBridgeNode()
    {
        if (client_fd_ >= 0) { ::shutdown(client_fd_, SHUT_RDWR); close(client_fd_); }
        if (server_fd_ >= 0) { ::shutdown(server_fd_, SHUT_RDWR); close(server_fd_); }
    }

private:
    void receive_tcp()
    {
        // Accepteer nieuwe Quest verbinding (non-blocking)
        if (client_fd_ < 0) {
            sockaddr_in client_addr{};
            socklen_t len = sizeof(client_addr);
            client_fd_ = accept4(
                server_fd_,
                reinterpret_cast<sockaddr *>(&client_addr),
                &len,
                SOCK_NONBLOCK);
            if (client_fd_ < 0) return;
            RCLCPP_INFO(this->get_logger(), "Quest verbonden.");
        }

        char buf[1024];
        ssize_t n = recv(client_fd_, buf, sizeof(buf) - 1, MSG_DONTWAIT);

        if (n < 0) return;  // nog geen data

        if (n == 0) {
            RCLCPP_WARN(this->get_logger(), "Quest verbroken — wachten op herverbinding...");
            close(client_fd_);
            client_fd_ = -1;
            buffer_.clear();
            return;
        }

        buf[n] = '\0';
        buffer_ += buf;

        // Verwerk elke volledige JSON regel (gescheiden door \n)
        size_t pos;
        while ((pos = buffer_.find('\n')) != std::string::npos) {
            std::string line = buffer_.substr(0, pos);
            buffer_.erase(0, pos + 1);
            if (!line.empty()) process_json(line);
        }
    }

    void process_json(const std::string & json)
    {
        // Verwacht JSON: {"x": 0.0, "y": 0.1, "z": 0.0, "w": 0.99}
        double qx = extract(json, "x");
        double qy = extract(json, "y");
        double qz = extract(json, "z");
        double qw = extract(json, "w");

        geometry_msgs::msg::QuaternionStamped msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = "quest_imu";
        msg.quaternion.x    = qx;
        msg.quaternion.y    = qy;
        msg.quaternion.z    = qz;
        msg.quaternion.w    = qw;

        publisher_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(),
            "Gepubliceerd  x=%.4f  y=%.4f  z=%.4f  w=%.4f",
            qx, qy, qz, qw);
    }

    double extract(const std::string & json, const std::string & key)
    {
        std::regex pattern("\"" + key + "\"\\s*:\\s*(-?\\d+(\\.\\d+)?)");
        std::smatch match;
        if (std::regex_search(json, match, pattern))
            return std::stod(match[1]);
        return 0.0;
    }

    int server_fd_ = -1;
    int client_fd_ = -1;
    std::string buffer_;

    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuestBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
