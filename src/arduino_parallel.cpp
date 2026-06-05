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

// ── Latency-echo naar de Quest (UDP) ─────────────────────────────────────────
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>

// ── Neck geometry (millimetres) ──────────────────────────────────────────────
constexpr double VERTEBRA_RADIUS = 52.0;
constexpr double VERTEBRA_HEIGHT = 7.0;
constexpr double CENTER_RADIUS = 5.0;
constexpr double ROPE_OFFSET = 20.0;
constexpr int N_VERTEBRAE = 6;
constexpr double SPINDLE_RADIUS = 20.0;
constexpr double NEUTRAL_ROPE_LENGTH = VERTEBRA_HEIGHT * 2.0 * N_VERTEBRAE;

static constexpr int TIGHTNESS = 30;
static constexpr int LEFT_NEUTRAL = 85 + TIGHTNESS;
static constexpr int RIGHT_NEUTRAL = 90 + TIGHTNESS;
static constexpr int FRONT_NEUTRAL = 85 + TIGHTNESS;
static constexpr int BACK_NEUTRAL = 100 + TIGHTNESS;

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
            serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
            serial_connected_ = true;
        }
        catch (const std::exception &e)
        {
            serial_connected_ = false;
            RCLCPP_WARN(this->get_logger(), "Arduino not connected — logging only");
        }

        // ── Latency-echo opzetten ────────────────────────────────────────────
        // Stuurt het seq-nummer (uit frame_id) als UDP-pakketje terug naar de
        // Quest op ECHO_PORT, zodat die de round-trip-latency kan meten.
        // Quest-IP komt uit de omgevingsvariabele QUEST_IP (export QUEST_IP=...).
        echo_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        const char *quest_ip = std::getenv("QUEST_IP");
        if (echo_sock_ >= 0 && quest_ip)
        {
            std::memset(&echo_addr_, 0, sizeof(echo_addr_));
            echo_addr_.sin_family = AF_INET;
            echo_addr_.sin_port = htons(ECHO_PORT);
            if (inet_pton(AF_INET, quest_ip, &echo_addr_.sin_addr) == 1)
            {
                echo_enabled_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "Latency-echo aan: seq -> %s:%d (UDP)", quest_ip, ECHO_PORT);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "QUEST_IP ongeldig — echo uit");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "QUEST_IP niet gezet — latency-echo uit (export QUEST_IP=<ip bril>)");
        }

        // ── QoS: keep-last-1 best-effort ─────────────────────────────────────
        // MOET matchen met de publisher in de bridge. Een reliable subscriber
        // op een best-effort publisher is INCOMPATIBEL (dan komt er niets door).
        // Keep-last-1 is bovendien de queue-fix: alleen de NIEUWSTE pose telt.
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();

        subscription_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "orientation", 10,
            [this](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
            {
                // Echo het seq DIRECT terug, vóór alle servo-berekeningen, zodat
                // de meting de keten-/netwerklatency weergeeft en niet de rekentijd.
                echo_seq(msg->header.frame_id);

                auto [tilt, pan, yaw] = quaternionToTiltPanYaw(msg);
                RCLCPP_INFO(this->get_logger(), "tilt: %.4f  pan: %.4f, yaw: %.4f", tilt, pan, yaw);
                RCLCPP_INFO(this->get_logger(), "Quaternion data (xyzw): (%.4f, %.4f, %.4f, %.4f)", msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);

                for (auto &rope : ropes_)
                    rope.update(tilt, pan);

                sendMotorAngles(yaw, ropes_);
            });
    }

    ~ArduinoParallel() override
    {
        if (echo_sock_ >= 0)
            close(echo_sock_);
    }

private:
    static constexpr int ECHO_PORT = 5006; // moet matchen met HMD_ECHO_PORT in de Quest-app

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::vector<Rope> ropes_;
    bool serial_connected_ = false;

    // ── Echo-state ───────────────────────────────────────────────────────────
    int echo_sock_ = -1;
    bool echo_enabled_ = false;
    sockaddr_in echo_addr_{};

    // Haalt het seq-nummer uit frame_id ("quest_imu:<seq>") en stuurt het als
    // 32-bit network-order integer terug naar de Quest. Geen seq -> niets doen.
    void echo_seq(const std::string &frame_id)
    {
        if (!echo_enabled_)
            return;
        auto pos = frame_id.find(':');
        if (pos == std::string::npos)
            return;
        try
        {
            uint32_t seq = static_cast<uint32_t>(std::stoul(frame_id.substr(pos + 1)));
            uint32_t seq_net = htonl(seq);
            sendto(echo_sock_, &seq_net, sizeof(seq_net), 0,
                   reinterpret_cast<sockaddr *>(&echo_addr_), sizeof(echo_addr_));
        }
        catch (...)
        {
            // geen geldig seq in frame_id -> overslaan
        }
    }

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

    // ── Acceleration limiter state ───────────────────────────────────────────
    tf2::Vector3 v_norm_prev_ = tf2::Vector3(0, 0, 1); // previous normal vector
    tf2::Vector3 v_norm_vel_ = tf2::Vector3(0, 0, 0);  // current velocity of normal vector
    tf2::Quaternion quat_prev_ = tf2::Quaternion(0, 0, 0, 1);                             // previous orientation quaternion
    rclcpp::Time last_update_time_;
    bool first_update_ = true;

    static constexpr double MAX_ACCELERATION = 0.5; // units/s², tune this

    tf2::Vector3 limitAcceleration(const tf2::Vector3 &v_norm_target, double dt)
    {
        // Desired velocity to reach target in one step
        tf2::Vector3 desired_vel = (v_norm_target - v_norm_prev_) / dt;

        // Acceleration needed to reach desired velocity
        tf2::Vector3 accel = (desired_vel - v_norm_vel_) / dt;

        // Clamp acceleration magnitude
        double accel_mag = accel.length();
        if (accel_mag > MAX_ACCELERATION)
        {
            RCLCPP_WARN(this->get_logger(), "Acceleration limited from %f to %f", accel_mag, MAX_ACCELERATION);
            accel = accel * (MAX_ACCELERATION / accel_mag);
        }

        // Integrate
        v_norm_vel_ = v_norm_vel_ + accel * dt;
        tf2::Vector3 v_norm_limited = v_norm_prev_ + v_norm_vel_ * dt;

        // Keep on unit sphere
        v_norm_limited.normalize();
        v_norm_prev_ = v_norm_limited;

        return v_norm_limited;
    }

    std::tuple<double, double, double> quaternionToTiltPanYaw(const geometry_msgs::msg::QuaternionStamped::SharedPtr q_msg)
    {
        tf2::Quaternion q_raw;
        tf2::fromMsg(q_msg->quaternion, q_raw);

        tf2::Quaternion q = tf2::slerp(q_raw, quat_prev_, 0.01); // smooth interpolation to prevent jumps
        quat_prev_ = q_raw;

        // get normal vector by applying rotation to vector pointing up (0, 0, 1)
        tf2::Vector3 v_norm = quatRotate(q, tf2::Vector3(0, 0, 1));

        // // ── Acceleration limiting ────────────────────────────────────────────
        // rclcpp::Time now = this->get_clock()->now();
        // tf2::Vector3 v_norm;
        // if (first_update_)
        // {
        //     v_norm = v_norm_raw;
        //     v_norm_prev_ = v_norm_raw;
        //     first_update_ = false;
        // }
        // else
        // {
        //     double dt = (now - last_update_time_).seconds();
        //     v_norm = limitAcceleration(v_norm_raw, dt);
        // }
        // last_update_time_ = now;
        // // ────────────────────────────────────────────────────────────────────

        double tilt = std::clamp(acos(std::clamp(v_norm.z(), -1.0, 1.0)), 0.0, M_PI / 2);
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
