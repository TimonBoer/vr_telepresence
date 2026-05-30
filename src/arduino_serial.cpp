#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <boost/asio.hpp>

#include <algorithm>
#include <cmath>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

// Mapt [-180, +180] graden naar [0, 180] voor de servo
static uint8_t to_servo_byte(double degrees)
{
    double mapped = std::clamp(degrees + 180.0, 0.0, 180.0);
    return static_cast<uint8_t>(mapped);
}

class ArduinoSerial : public rclcpp::Node
{
public:
    ArduinoSerial() : Node("arduino_serial"), io_(), serial_(io_)
    {
        try
        {
            serial_.open("/dev/ttyACM0");
            serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
            serial_connected_ = true;
        }
        catch (const std::exception &e)
        {
            serial_connected_ = false;
            RCLCPP_WARN(this->get_logger(), "Arduino not connected — logging only");
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "orientation", 10,
            [this](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
            {
                tf2::Quaternion quat;
                tf2::fromMsg(msg->quaternion, quat);

                // Create a rotation matrix from the quaternion
                tf2::Matrix3x3 mat(quat);

                double roll, pitch, yaw;

                // Extract RPY (in radians)
                mat.getRPY(roll, pitch, yaw);
                
                const double RAD_TO_DEG = 180.0 / M_PI;
                uint8_t roll_byte = to_servo_byte(roll * RAD_TO_DEG);
                uint8_t pitch_byte = to_servo_byte(pitch * RAD_TO_DEG);
                uint8_t yaw_byte = to_servo_byte(yaw * RAD_TO_DEG);

                RCLCPP_INFO(this->get_logger(),
                            "Euler  roll=%.1f°  pitch=%.1f°  yaw=%.1f°  →  "
                            "servo  roll=%d  pitch=%d  yaw=%d",
                            roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG,
                            roll_byte, pitch_byte, yaw_byte);

                if (serial_connected_)
                {
                    uint8_t data[4] = {roll_byte, pitch_byte, yaw_byte, 181};
                    boost::asio::write(serial_, boost::asio::buffer(data, 4));
                }
            });
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    bool serial_connected_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoSerial>());
    rclcpp::shutdown();
    return 0;
}
