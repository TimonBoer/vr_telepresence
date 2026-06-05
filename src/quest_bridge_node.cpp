#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <regex>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/empty.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// quest_bridge_node
//
// Opent een TCP server op poort 5005.
// De Meta Quest app verbindt en stuurt JSON regels in de vorm:
//   {"x": 0.0, "y": 0.1, "z": 0.0, "w": 0.99}
//
// De node publiceert de quaternion op:
//   orientation  (geometry_msgs/QuaternionStamped)
// ─────────────────────────────────────────────────────────────────────────────

class Calibrator
{
public:
    Calibrator() : pending_(false)
    {
        ref_q_.setRPY(0, 0, 0); // identity
        ref_q_ = ref_q_.inverse();
    }

    // Call this when the user presses the calibration button
    void request()
    {
        pending_ = true;
    }

    // Pass in the raw quaternion, get back the calibrated one.
    // If a calibration was requested, this sample becomes the new reference.
    tf2::Quaternion apply(double x, double y, double z, double w)
    {
        tf2::Quaternion q_raw(x, y, z, w);

        if (pending_)
        {
            ref_q_ = q_raw.inverse();
            pending_ = false;
        }

        return ref_q_ * q_raw;
    }

private:
    tf2::Quaternion ref_q_;
    bool pending_;
};

class QuestBridgeNode : public rclcpp::Node
{
public:
    QuestBridgeNode() : Node("quest_bridge_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("orientation", 10);

        calibrate_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "calibrate", 10,
            [this](const std_msgs::msg::Empty::SharedPtr)
            {
                calibrator_.request();
                RCLCPP_INFO(this->get_logger(), "Calibration requested");
            });

        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(5005);

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
        if (client_fd_ >= 0)
        {
            ::shutdown(client_fd_, SHUT_RDWR);
            close(client_fd_);
        }
        if (server_fd_ >= 0)
        {
            ::shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
        }
    }

private:
    void receive_tcp()
    {
        // Accepteer nieuwe Quest verbinding (non-blocking)
        if (client_fd_ < 0)
        {
            sockaddr_in client_addr{};
            socklen_t len = sizeof(client_addr);
            client_fd_ = accept4(
                server_fd_,
                reinterpret_cast<sockaddr *>(&client_addr),
                &len,
                SOCK_NONBLOCK);
            if (client_fd_ < 0)
                return;
            RCLCPP_INFO(this->get_logger(), "Quest verbonden.");
        }

        char buf[1024];
        ssize_t n = recv(client_fd_, buf, sizeof(buf) - 1, MSG_DONTWAIT);

        if (n < 0)
            return; // nog geen data

        if (n == 0)
        {
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
        while ((pos = buffer_.find('\n')) != std::string::npos)
        {
            std::string line = buffer_.substr(0, pos);
            buffer_.erase(0, pos + 1);
            if (!line.empty())
                process_json(line);
        }
    }

    void process_json(const std::string &json)
    {
        // Verwacht JSON: {"x": 0.0, "y": 0.1, "z": 0.0, "w": 0.99}
        double qx = extract(json, "x");
        double qy = extract(json, "y");
        double qz = extract(json, "z");
        double qw = extract(json, "w");
        long seq = static_cast<long>(extract(json, "seq"));
        // RCLCPP_WARN(this->get_logger(),"Ontvangen JSON: x=%.4f  y=%.4f  z=%.4f  w=%.4f", qx, qy, qz, qw);

        // swapping x/y/z to match ROS coordinate conventions (Quest's forward is ROS's x, etc.)
        // after swapping, we calibrate by applying the inverse of the reference orientation, which is set when the user presses the calibration button.
        tf2::Quaternion q_cal = calibrator_.apply(-qz, -qx, qy, qw);

        geometry_msgs::msg::QuaternionStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "quest_imu:" + std::to_string(seq);
        msg.quaternion.x = q_cal.x();
        msg.quaternion.y = q_cal.y();
        msg.quaternion.z = q_cal.z();
        msg.quaternion.w = q_cal.w();

        publisher_->publish(msg);

        RCLCPP_WARN(this->get_logger(),
                    "Gepubliceerd  x=%.4f  y=%.4f  z=%.4f  w=%.4f",
                    msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w);
    }

    double extract(const std::string &json, const std::string &key)
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

    Calibrator calibrator_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr calibrate_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuestBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
