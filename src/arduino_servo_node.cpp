#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <boost/asio.hpp>
#include <string>
#include <algorithm>
#include <cmath>

// ── Neck geometry (millimetres) ──────────────────────────────────────────────
constexpr double VERTEBRA_RADIUS = 52.0;
constexpr double VERTEBRA_HEIGHT = 7.0;
constexpr double CENTER_RADIUS = 5.0;
constexpr double ROPE_OFFSET = 20.0;
constexpr int N_VERTEBRAE = 6;
constexpr double SPINDLE_RADIUS = 20.0;
constexpr double NEUTRAL_ROPE_LENGTH = VERTEBRA_HEIGHT * 2.0 * N_VERTEBRAE;

class Rope
{
public:
    Rope(double angular_pos, int motor_neutral_angle)
        : angular_pos_(angular_pos), motor_neutral_angle_(motor_neutral_angle),
          angle_from_curve_centre_(0.0), dist_from_curve_centre_(0.0),
          rope_length_(0.0), angle_offset_(0.0), motor_angle_(motor_neutral_angle)
    {
        update(0.0, 0.0);
    }

    void update(double tilt_angle, double pan_angle)
    {
        updateRopeGeometry(pan_angle);
        updateRopeLength(tilt_angle);
        updateMotorAngle();
    }

    double getMotorAngle() const { return motor_angle_; }

private:
    void updateRopeGeometry(double pan_angle)
    {
        double x = ROPE_OFFSET * std::cos(pan_angle + angular_pos_) + CENTER_RADIUS;
        double y = VERTEBRA_RADIUS - VERTEBRA_HEIGHT;
        dist_from_curve_centre_ = std::sqrt(x * x + y * y);
        angle_from_curve_centre_ = -std::atan2(x, y);
    }

    void updateRopeLength(double tilt_angle)
    {
        double tilt_per_vertebra = tilt_angle / N_VERTEBRAE;
        double projected_offset = dist_from_curve_centre_ * std::cos(angle_from_curve_centre_ - tilt_per_vertebra);
        double length_per_vertebra = VERTEBRA_RADIUS - projected_offset;
        rope_length_ = length_per_vertebra * 2.0 * N_VERTEBRAE;
    }

    void updateMotorAngle()
    {
        constexpr double RAD_TO_DEG = 180.0 / M_PI;
        double length_delta = NEUTRAL_ROPE_LENGTH - rope_length_;
        angle_offset_ = (length_delta / SPINDLE_RADIUS) * RAD_TO_DEG;
        motor_angle_ = static_cast<int>(motor_neutral_angle_ + angle_offset_);
    }

    double angular_pos_;
    int motor_neutral_angle_;
    double angle_from_curve_centre_;
    double dist_from_curve_centre_;
    double rope_length_;
    double angle_offset_;
    double motor_angle_;
};

std::pair<double, double> quaternionToTiltPan(const geometry_msgs::msg::Quaternion &q)
{
    // ROS quaternion is (x, y, z, w)
    double x = q.x, y = q.y, z = q.z, w = q.w;

    // Rotate up-vector (0, 0, 1) by quaternion: v' = q * v * q^-1
    // Unrolled for v = (0, 0, 1) — third column of the rotation matrix
    double rx = 2.0 * (x * z + w * y);       // was: 2(xz + wy)
    double ry = 2.0 * (y * z - w * x);       // was: 2(yz − wx)
    double rz = 1.0 - 2.0 * (x * x + y * y); // was: 1 − 2(x² + y²)

    double tilt = std::acos(std::clamp(rz, -1.0, 1.0));
    double pan = -std::atan2(rx, ry) + M_PI;

    return {tilt, pan};
}

class ArduinoServoNode : public rclcpp::Node
{
public:
    ArduinoServoNode() : Node("arduino_servo_node"), io_(), serial_(io_),
                         ropes_({
                             Rope(0, 90 + TIGHTNESS),         // left
                             Rope(M_PI, 95 + TIGHTNESS),      // right
                             Rope(M_PI / 2, 95 + TIGHTNESS),  // front
                             Rope(-M_PI / 2, 90 + TIGHTNESS), // back
                         })
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
                auto [tilt, pan] = quaternionToTiltPan(msg->quaternion); // .quaternion added
                RCLCPP_INFO(this->get_logger(), "tilt: %.4f  pan: %.4f", tilt, pan);

                for (auto &rope : ropes_)
                    rope.update(tilt, pan);

                sendMotorAngles();
            });
    }

private:
    static constexpr int TIGHTNESS = 5;

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::vector<Rope> ropes_;
    bool serial_connected_ = false;
    
    void sendMotorAngles()
    {
        std::vector<uint8_t> motor_angles;
        motor_angles.reserve(ropes_.size() + 1);

        for (const auto& rope : ropes_)
        {
            uint8_t angle = static_cast<uint8_t>(std::clamp(rope.getMotorAngle(), 0.0, 180.0));
            motor_angles.push_back(angle);
            RCLCPP_INFO(this->get_logger(), "Motor angle: %d", angle);
        }

        motor_angles.push_back(181);
        if (serial_connected_)
            boost::asio::write(serial_, boost::asio::buffer(motor_angles));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoServoNode>());
    rclcpp::shutdown();
    return 0;
}