#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>

class MoveToGoal : public rclcpp::Node {
public:
    MoveToGoal() : Node("move_to_goal") {
        this->declare_parameter("goal_x", 5.0);
        this->declare_parameter("goal_y", 5.0);
        this->declare_parameter("goal_theta", 0.0);

        this->get_parameter("goal_x", goal_x_);
        this->get_parameter("goal_y", goal_y_);
        this->get_parameter("goal_theta", goal_theta_);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&MoveToGoal::poseCallback, this, std::placeholders::_1));
    }

private:
    double goal_x_, goal_y_, goal_theta_;
    double tolerance_ = 0.1;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
        double distance = std::sqrt(std::pow(goal_x_ - msg->x, 2) + std::pow(goal_y_ - msg->y, 2));
        double angle_to_goal = std::atan2(goal_y_ - msg->y, goal_x_ - msg->x);
        double angle_diff = normalizeAngle(angle_to_goal - msg->theta);

        geometry_msgs::msg::Twist cmd;

        if (distance > tolerance_) {
            if (std::abs(angle_diff) > 0.1) {
                cmd.angular.z = angle_diff;
            } else {
                cmd.linear.x = distance;
            }
        } else if (std::abs(normalizeAngle(goal_theta_ - msg->theta)) > 0.1) {
            cmd.angular.z = normalizeAngle(goal_theta_ - msg->theta);
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(cmd);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToGoal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
