#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_turtle_commands/action/message_turtle_commands.hpp"

using namespace std::chrono_literals;
using TurtleAction = action_turtle_commands::action::MessageTurtleCommands;

class ActionTurtleClient : public rclcpp::Node {
public:
    ActionTurtleClient() : Node("action_turtle_client") {
        client_ = rclcpp_action::create_client<TurtleAction>(this, "MessageTurtleCommands");
        execute_sequence();
    }

private:
    rclcpp_action::Client<TurtleAction>::SharedPtr client_;

    void execute_sequence() {
        // Последовательно отправляем команды
        if (!send_goal("forward", 2, 0)) return;
        if (!send_goal("turn_right", 0, 90)) return;
        if (!send_goal("forward", 1, 0)) return;

        RCLCPP_INFO(this->get_logger(), "Все команды успешно выполнены!");
    }

    bool send_goal(const std::string &command, int s, int angle) {
        auto goal_msg = TurtleAction::Goal();
        goal_msg.command = command;
        goal_msg.s = s;
        goal_msg.angle = angle;

        // Отправка цели
        RCLCPP_INFO(this->get_logger(), "Отправка команды: %s, s: %d, angle: %d", command.c_str(), s, angle);
        auto send_goal_options = rclcpp_action::Client<TurtleAction>::SendGoalOptions();

        // Указание обратного вызова для feedback
        send_goal_options.feedback_callback = [this](auto, const std::shared_ptr<const TurtleAction::Feedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "Обратная связь: пройдено %d метров", feedback->odom);
        };

        auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось отправить команду: %s", command.c_str());
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Сервер отклонил команду: %s", command.c_str());
            return false;
        }

        // Ожидание завершения цели
        auto result_future = client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка при ожидании результата команды: %s", command.c_str());
            return false;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Команда выполнена успешно: %s", command.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Команда не выполнена: %s", command.c_str());
            return false;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionTurtleClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
