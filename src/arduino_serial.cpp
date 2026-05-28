#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <boost/asio.hpp>

#include <algorithm>
#include <cmath>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// arduino_servo_node
//
// Luistert naar:
//   /head/orientation  (geometry_msgs/QuaternionStamped)
//
// Converteert quaternion → Euler (roll/pitch/yaw) in graden.
// Mapt de graden van [-180, +180] naar [0, 180] voor de servo's.
// Stuurt 4 bytes naar de Arduino via serieel (/dev/ttyACM0):
//   [roll_byte, pitch_byte, yaw_byte, 181]   (181 = end byte)
// ─────────────────────────────────────────────────────────────────────────────

// Quaternion → Euler omzetting (ZYX volgorde, uitvoer in graden)
struct Euler { double roll, pitch, yaw; };

static Euler quaternion_to_euler(double qx, double qy, double qz, double qw)
{
    // Roll (rotatie om X-as)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (rotatie om Y-as)
    double sinp = 2.0 * (qw * qy - qz * qx);
    double pitch;
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);  // Gimbal lock afvangen
    else
        pitch = std::asin(sinp);

    // Yaw (rotatie om Z-as)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    // Radialen → graden
    constexpr double RAD2DEG = 180.0 / M_PI;
    return { roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG };
}

// Mapt [-180, +180] graden naar [0, 180] voor de servo
static uint8_t to_servo_byte(double degrees)
{
    double mapped = std::clamp((degrees + 180.0) / 2.0, 0.0, 180.0);
    return static_cast<uint8_t>(mapped);
}

class ArduinoSerial : public rclcpp::Node
{
public:
    ArduinoSerial() : Node("arduino_serial"), io_(), serial_(io_)
    {
        // Probeer Arduino te openen — sla over als niet aangesloten
        try {
            serial_.open("/dev/ttyACM0");
            serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
            serial_connected_ = true;
            RCLCPP_INFO(this->get_logger(), "Arduino verbonden op /dev/ttyACM0");
        } catch (const std::exception & e) {
            serial_connected_ = false;
            RCLCPP_WARN(this->get_logger(),
                "Arduino NIET verbonden — alleen logging (geen serieel)");
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "orientation", 10,
            std::bind(&ArduinoSerial::on_orientation, this, std::placeholders::_1));
    }

private:
    void on_orientation(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
    {
        auto & q = msg->quaternion;
        Euler e  = quaternion_to_euler(q.x, q.y, q.z, q.w);

        uint8_t roll_byte  = to_servo_byte(e.roll);
        uint8_t pitch_byte = to_servo_byte(e.pitch);
        uint8_t yaw_byte   = to_servo_byte(e.yaw);

        RCLCPP_INFO(this->get_logger(),
            "Euler  roll=%.1f°  pitch=%.1f°  yaw=%.1f°  →  "
            "servo  roll=%d  pitch=%d  yaw=%d",
            e.roll, e.pitch, e.yaw,
            roll_byte, pitch_byte, yaw_byte);

        if (serial_connected_) {
            uint8_t data[4] = { roll_byte, pitch_byte, yaw_byte, 181 };
            boost::asio::write(serial_, boost::asio::buffer(data, 4));
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    bool serial_connected_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoSerial>());
    rclcpp::shutdown();
    return 0;
}
