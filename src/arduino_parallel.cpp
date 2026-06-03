#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <boost/asio.hpp>
#include <string>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ── Neck geometry (millimetres) ──────────────────────────────────────────────
constexpr double VERTEBRA_RADIUS = 52.0;
constexpr double VERTEBRA_HEIGHT = 7.0;
constexpr double CENTER_RADIUS = 5.0;
constexpr double ROPE_OFFSET = 20.0;
constexpr int N_VERTEBRAE = 6;
constexpr double SPINDLE_RADIUS = 20.0;
constexpr double NEUTRAL_ROPE_LENGTH = VERTEBRA_HEIGHT * 2.0 * N_VERTEBRAE;

static constexpr int TIGHTNESS = 20;
static constexpr int LEFT_NEUTRAL =     90 + TIGHTNESS;
static constexpr int RIGHT_NEUTRAL =    90 + TIGHTNESS;
static constexpr int FRONT_NEUTRAL =    90 + TIGHTNESS;
static constexpr int BACK_NEUTRAL =     90 + TIGHTNESS;

static constexpr int YAW_NEUTRAL = 90;

class Rope
{
public:
    Rope(int motor_neutral_angle, double angular_pos, std::string name = "")
        : angular_pos_(angular_pos), motor_neutral_angle_(motor_neutral_angle),
          angle_from_curve_centre_(0.0), dist_from_curve_centre_(0.0),
          rope_length_(0.0), angle_offset_(0.0), motor_angle_(motor_neutral_angle), name_(name)
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

    std::string getName() const { return name_; }

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
    std::string name_;
};

void printVector3(const tf2::Vector3 v, const rclcpp::Logger &logger)
{
    RCLCPP_INFO(logger, "(%f, %f, %f)", v.x(), v.y(), v.z());
}

class ArduinoParallel : public rclcpp::Node
{
public:
    ArduinoParallel() : Node("arduino_parallel"), io_(), serial_(io_),
                        ropes_({
                            Rope(LEFT_NEUTRAL, 0, "left"),
                            Rope(RIGHT_NEUTRAL, M_PI, "right"),
                            Rope(FRONT_NEUTRAL, M_PI / 2, "front"),
                            Rope(BACK_NEUTRAL, -M_PI / 2, "back"),
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
                auto [tilt, pan, yaw] = quaternionToTiltPanYaw(msg);
                RCLCPP_INFO(this->get_logger(), "tilt: %.4f  pan: %.4f, yaw: %.4f", tilt, pan, yaw);
                RCLCPP_INFO(this->get_logger(), "Quaternion data (xyzw): (%.4f, %.4f, %.4f, %.4f)", msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);

                for (auto &rope : ropes_)
                    rope.update(tilt, pan);

                sendMotorAngles(yaw, ropes_);
            });
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::vector<Rope> ropes_;
    bool serial_connected_ = false;

    void sendMotorAngles(double yaw, const std::vector<Rope> &ropes)
    {
        std::vector<uint8_t> motor_angles;
        motor_angles.reserve(ropes.size() + 2);

        for (const auto &rope : ropes)
        {
            uint8_t angle = static_cast<uint8_t>(std::clamp(rope.getMotorAngle(), 0.0, 180.0));
            motor_angles.push_back(angle);
            RCLCPP_INFO(this->get_logger(), "%s: %d", rope.getName().c_str(), angle);
        }

        auto yaw_angle = static_cast<uint8_t>(std::clamp(yaw * 180.0 / M_PI + YAW_NEUTRAL, 0.0, 180.0));
        motor_angles.push_back(yaw_angle);
        RCLCPP_INFO(this->get_logger(), "Yaw: %d", yaw_angle);

        motor_angles.push_back(181);
        if (serial_connected_)
            boost::asio::write(serial_, boost::asio::buffer(motor_angles));
    }

    std::tuple<double, double, double> quaternionToTiltPanYaw(const geometry_msgs::msg::QuaternionStamped::SharedPtr q_msg)
    {
        tf2::Quaternion q;
        tf2::fromMsg(q_msg->quaternion, q);

        // get normal vector by applying rotation to vector pointing up (0, 0, 1)
        tf2::Vector3 v_norm = quatRotate(q, tf2::Vector3(0, 0, 1));

        double tilt = std::clamp(acos(std::clamp(v_norm.z(), -1.0, 1.0)), 0.0, M_PI/2);
        double pan = -atan2(v_norm.x(), v_norm.y()) + M_PI;

        // get forward vector by applying rotation to the vector pointing forward (1, 0, 0)
        tf2::Vector3 v_forward = quatRotate(q, tf2::Vector3(1, 0, 0));

        // get quaternion of only the tilt and pan rotation, having 0 yaw rotation
        tf2::Vector3 up(0, 0, 1);
        tf2::Quaternion tilt_pan_rot = tf2::shortestArcQuatNormalize2(up, v_norm);

        // rotating vector (1, 0, 0) results in the vector with 0 yaw
        tf2::Vector3 expected_forward = quatRotate(tilt_pan_rot, tf2::Vector3(1, 0, 0));

        // calculate angle between expected forward and actual forward vector.
        double yaw = std::acos(std::clamp(v_forward.dot(expected_forward), -1.0, 1.0));

        // determine the sign of the yaw angle by checking the direction of the cross product
        tf2::Vector3 cross = expected_forward.cross(v_forward);
        if (cross.dot(v_norm) < 0)
            yaw = -yaw;

        return {tilt, pan, yaw};
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoParallel>());
    rclcpp::shutdown();
    return 0;
}