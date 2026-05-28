#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <termios.h>
#include <unistd.h>

// ─────────────────────────────────────────────────────────────────────────────
// emotion_input_node
//
// Leest toetsenbordinput en publiceert de gekozen emotie op:
//   /robot/expression  (std_msgs/String)
//
// Toetsen:
//   i → blij
//   d → verdrietig
//   j → boos
//   f → bang
//   q → stoppen
// ─────────────────────────────────────────────────────────────────────────────

class EmotionInputNode : public rclcpp::Node
{
public:
    EmotionInputNode() : Node("emotion_input_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/expression", 10);

        RCLCPP_INFO(this->get_logger(), "Emotie-input node gestart.");
        RCLCPP_INFO(this->get_logger(),
            "Druk [i=blij  d=verdrietig  j=boos  f=bang]  q=stoppen");

        key_loop();
    }

private:
    char getch()
    {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0) perror("tcgetattr");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN]  = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ~ICANON");
        if (read(0, &buf, 1) < 0) perror("read");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ICANON");
        return buf;
    }

    void key_loop()
    {
        while (rclcpp::ok()) {
            char key = getch();

            if (key == 'i' || key == 'd' || key == 'j' || key == 'f') {
                auto message  = std_msgs::msg::String();
                message.data  = std::string(1, key);
                RCLCPP_INFO(this->get_logger(),
                    "Emotie '%c' gepubliceerd op /robot/expression", key);
                publisher_->publish(message);
            } else if (key == 'q') {
                RCLCPP_INFO(this->get_logger(), "Stoppen...");
                break;
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmotionInputNode>();
    rclcpp::shutdown();
    return 0;
}
