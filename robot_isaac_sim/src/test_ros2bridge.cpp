#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class TestROS2Bridge : public rclcpp::Node
{
public:
    TestROS2Bridge()
        : Node("test_ros2bridge"), time_start_(this->now().seconds())
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

        joint_state_.name = {"neck_dx_joint", "dx_tilt_joint"};

        int num_joints = joint_state_.name.size();

        joint_state_.position.resize(num_joints, 0.0);
        default_joints_ = {0.0, 0.0};

        max_joints_ = {default_joints_[0] + 20, default_joints_[1] + 50};
        min_joints_ = {default_joints_[0] - 20, default_joints_[1] - 50};

        timer_ = this->create_wall_timer(50ms, std::bind(&TestROS2Bridge::timer_callback, this));
    }

private:
    void timer_callback()
    {
        joint_state_.header.stamp = this->now();

        double current_time = this->now().seconds();
        current_time /= 10;
        std::vector<double> joint_position = {
            std::sin(current_time - time_start_) * (max_joints_[0] - min_joints_[0]) * 0.5 + default_joints_[0],
            std::sin(current_time - time_start_) * (max_joints_[1] - min_joints_[1]) * 0.5 + default_joints_[1]
        };

        joint_state_.position = joint_position;

        publisher_->publish(joint_state_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<double> default_joints_;
    std::vector<double> max_joints_;
    std::vector<double> min_joints_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_start_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestROS2Bridge>());
    rclcpp::shutdown();
    return 0;
}
