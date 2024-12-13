#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include "action_turtle_commands/action/message_turtle_commands.hpp"

using namespace std::placeholders;

class ActionTurtleServer : public rclcpp::Node {
public:
    using TurtleAction = action_turtle_commands::action::MessageTurtleCommands;
    using GoalHandleTurtleAction = rclcpp_action::ServerGoalHandle<TurtleAction>;

    ActionTurtleServer() : Node("action_turtle_server") {
        action_server_ = rclcpp_action::create_server<TurtleAction>(
            this,
            "MessageTurtleCommands",
            std::bind(&ActionTurtleServer::handle_goal, this, _1, _2),
            std::bind(&ActionTurtleServer::handle_cancel, this, _1),
            std::bind(&ActionTurtleServer::handle_accepted, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&ActionTurtleServer::pose_callback, this, _1));

        current_pose_ = std::make_shared<turtlesim::msg::Pose>();
    }

private:
    rclcpp_action::Server<TurtleAction>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    std::shared_ptr<turtlesim::msg::Pose> current_pose_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = msg;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TurtleAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal: command=%s, s=%d, angle=%d",
                    goal->command.c_str(), goal->s, goal->angle);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTurtleAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTurtleAction> goal_handle) {
        std::thread{std::bind(&ActionTurtleServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleTurtleAction> goal_handle) {
        auto feedback = std::make_shared<TurtleAction::Feedback>();
        auto result = std::make_shared<TurtleAction::Result>();

        const auto goal = goal_handle->get_goal();
        if (goal->command == "forward") {
            result->result = move_forward(goal->s, feedback, goal_handle);
        } else if (goal->command == "turn_left" || goal->command == "turn_right") {
            result->result = turn(goal->command, goal->angle);
        } else {
            result->result = false;
        }

        goal_handle->succeed(result);
    }

    bool move_forward(int distance, std::shared_ptr<TurtleAction::Feedback> feedback,
                      const std::shared_ptr<GoalHandleTurtleAction> goal_handle) {
        double start_x = current_pose_->x;
        double start_y = current_pose_->y;
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 1.0;

        while (rclcpp::ok()) {
            double current_distance = std::hypot(current_pose_->x - start_x, current_pose_->y - start_y);
            // feedback->odom = static_cast<int>(current_distance);
            // goal_handle->publish_feedback(feedback);
            if (current_distance >= distance) 
            {
                distance_buff += current_distance;
                feedback->odom = static_cast<int>(distance_buff);
                goal_handle->publish_feedback(feedback);
                break;
            }
            cmd_vel_publisher_->publish(cmd_vel);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        cmd_vel.linear.x = 0.0;
        cmd_vel_publisher_->publish(cmd_vel);
        return true;
    }

    bool turn(const std::string &direction, int angle) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.angular.z = (direction == "turn_left") ? 1.0 : -1.0;
        double target_angle = angle * M_PI / 180.0;

        double start_theta = current_pose_->theta;
        while (rclcpp::ok()) {
            double current_angle = std::fabs(current_pose_->theta - start_theta);
            if (current_angle >= target_angle) break;

            cmd_vel_publisher_->publish(cmd_vel);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        cmd_vel.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel);
        return true;
    }

    int32_t distance_buff = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionTurtleServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
