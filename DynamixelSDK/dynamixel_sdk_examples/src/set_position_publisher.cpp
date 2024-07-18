#include <memory>
#include <chrono>
#include <functional>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "camera_custom_interfaces/msg/detection.hpp"
#include "camera_custom_interfaces/msg/detections.hpp"

using namespace std::chrono_literals;

class SetPositionPublisher : public rclcpp::Node
{
public:
    SetPositionPublisher() : Node("set_position_publisher"),
        center_x(320),  // Initialize to center of the image (640/2)
        center_y(240),  // Initialize to center of the image (480/2)
        current_tf_angle_1(0),
        current_tf_angle_2(0),
        Kp(1.0), Ki(0.0), Kd(0.0),
        previous_error_x(0), previous_error_y(0),
        integral_x(0), integral_y(0),
        min_velocity(0.2), max_velocity(12.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_command", 10);

        detection_subscriber_ = this->create_subscription<camera_custom_interfaces::msg::Detections>(
            "detection_info", 10, std::bind(&SetPositionPublisher::detection_callback, this, std::placeholders::_1));

        tf_angle_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&SetPositionPublisher::tf_angle_callback, this, std::placeholders::_1));

        //timer_ = this->create_wall_timer(20ms, std::bind(&SetPositionPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        const int image_width = 640;
        const int image_height = 480;

        // Calculate differences between detection center and image center
        int delta_x = center_x - image_width / 2;  // 320 is the center of the image width (640/2)
        int delta_y = center_y - image_height / 2;  // 240 is the center of the image height (480/2)

        double tol = 0.0;
        double ratio_x = static_cast<double>(delta_x) / image_width;
        double ratio_y = static_cast<double>(delta_y) / image_height;

        int target_angle_x = current_tf_angle_1 + static_cast<int>((180.0 / M_PI) * atan(-ratio_x));
        int target_angle_y = current_tf_angle_2 + static_cast<int>((180.0 / M_PI) * atan(-ratio_y));

        // Clamp the target angles between min_angle and max_angle
        target_angle_x = std::clamp(target_angle_x, -20, 20);
        target_angle_y = std::clamp(target_angle_y, -50, 50);

        // PID control calculations
        double error_x = std::abs(target_angle_x - current_tf_angle_1);
        double error_y = std::abs(target_angle_y - current_tf_angle_2);

        integral_x += error_x;
        integral_y += error_y;

        double derivative_x = error_x - previous_error_x;
        double derivative_y = error_y - previous_error_y;

        double velocity_x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
        double velocity_y = Kp * error_y + Ki * integral_y + Kd * derivative_y;

        // Clamp the velocities between min_velocity and max_velocity
        velocity_x = std::clamp(velocity_x, min_velocity, max_velocity);
        velocity_y = std::clamp(velocity_y, min_velocity, max_velocity);

        previous_error_x = error_x;
        previous_error_y = error_y;

        auto message = std::make_shared<sensor_msgs::msg::JointState>();
        message->header.stamp = this->now();
        message->name.resize(2);
        message->position.resize(2);
        message->velocity.resize(2);
        
        message->name[0] = "neck_dx_joint";
        message->name[1] = "dx_tilt_joint";
               
        message->position[0] = target_angle_x;
        message->position[1] = target_angle_y;
        
        message->velocity[0] = velocity_x;
        message->velocity[1] = velocity_y;

        publisher_->publish(*message);
    }

    void detection_callback(const camera_custom_interfaces::msg::Detections::SharedPtr msg)
    {
        if (!msg->detections.empty()) {
            for (const auto& detection : msg->detections)
            {
                // Access individual detection data
                std::string class_name = detection.class_name;
                int32_t class_id = detection.class_id;
                float confidence = detection.confidence;
                int32_t x = detection.x;
                int32_t y = detection.y;
                int32_t width = detection.width;
                int32_t height = detection.height;

                if (detection.class_id == 64)
                {
                    center_x = detection.x + detection.width / 2;
                    center_y = detection.y + detection.height / 2;
                    timer_callback();
                    break;
                }
            }
            // RCLCPP_INFO(this->get_logger(), "Detection center: (%d, %d)", center_x, center_y);
        }
        else {
            // RCLCPP_WARN(this->get_logger(), "No detections received");
        }
    }

    void tf_angle_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_tf_angle_1 = msg->position[0];
        current_tf_angle_2 = msg->position[1];
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<camera_custom_interfaces::msg::Detections>::SharedPtr detection_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tf_angle_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    int center_x;
    int center_y;
    const int tolerance = 10;  // Tolerance to avoid too sensitive adjustments

    int current_tf_angle_1;
    int current_tf_angle_2;

    // PID controller parameters
    double Kp, Ki, Kd;
    double previous_error_x, previous_error_y;
    double integral_x, integral_y;

    // Velocity limits (in RPM)
    const double min_velocity;
    const double max_velocity;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetPositionPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
