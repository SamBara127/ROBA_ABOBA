#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TextToCmdVel : public rclcpp::Node {
public:
    TextToCmdVel() : Node("text_to_cmd_vel") {
        // Подписываемся на топик cmd_text
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "cmd_text", 10, std::bind(&TextToCmdVel::command_callback, this, std::placeholders::_1));

        // Публикуем команды в топик /turtle1/cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

private:
    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto twist = geometry_msgs::msg::Twist();

        // Определяем команды для черепахи в зависимости от полученной команды
        if (msg->data == "move_forward") {
            twist.linear.x = 1.0;  // 1 м/с вперёд
        } else if (msg->data == "move_backward") {
            twist.linear.x = -1.0;  // 1 м/с назад
        } else if (msg->data == "turn_right") {
            twist.angular.z = -1.5;  // 1.5 рад/с вправо
        } else if (msg->data == "turn_left") {
            twist.angular.z = 1.5;  // 1.5 рад/с влево
        } else {
            RCLCPP_WARN(this->get_logger(), "Неизвестная команда: '%s'", msg->data.c_str());
            return;
        }

        publisher_->publish(twist);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TextToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
