#include <chrono>
#include <vector>
#include <cmath>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define VELOCITY_SCAN 2 // Velocity of the Dynamixel Motors in revolutions per minute (RPM)

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
        : Node("joint_state_publisher"),
          current_index_(0),
          publish_interval_(1.0) // Intervalo inicial en segundos
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(publish_interval_ * 1000)),
            std::bind(&JointStatePublisher::timer_callback, this));

        joint_state_.name = {"neck_dx_joint", "dx_tilt_joint"};

        int num_joints = joint_state_.name.size();

        joint_state_.position.resize(num_joints, 0.0);
        joint_state_.velocity.resize(num_joints, 0.0);

        default_joints_ = {0.0, 0.0};

        max_joints_ = {default_joints_[0] + 20, default_joints_[1] + 50};
        min_joints_ = {default_joints_[0] - 20, default_joints_[1] - 50};

        // Inicializa la secuencia de posiciones
        position_sequence_ = {
            {default_joints_[0], default_joints_[1]},
            {min_joints_[0], default_joints_[1]},
            {max_joints_[0], default_joints_[1]},
            {default_joints_[0], default_joints_[1]},

            {default_joints_[0], min_joints_[1]/2},
            {min_joints_[0], min_joints_[1]/2},
            {max_joints_[0], min_joints_[1]/2},
            {default_joints_[0], min_joints_[1]/2},

            {default_joints_[0], min_joints_[1]},
            {min_joints_[0], min_joints_[1]},
            {max_joints_[0], min_joints_[1]},
            {default_joints_[0], min_joints_[1]},
        };

    }

private:
    void timer_callback()
    {
        joint_state_.header.stamp = this->now();
        joint_state_.position = position_sequence_[current_index_];
        joint_state_.velocity = {VELOCITY_SCAN, VELOCITY_SCAN};
        

        // Calcula el nuevo intervalo
        publish_interval_ = calculate_new_interval();
        // Ajusta el temporizador si el intervalo es variable (ejemplo de cambio dinámico)
        timer_->cancel();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(publish_interval_ * 1000)),
            std::bind(&JointStatePublisher::timer_callback, this));

        // Publica mensaje
        publisher_->publish(joint_state_);

        // Avanza al siguiente índice en la secuencia
        current_index_ = (current_index_ + 1) % position_sequence_.size();

    }

    // Ejemplo de función para calcular un nuevo intervalo
    double calculate_new_interval()
    {
        // Lógica para calcular el nuevo intervalo
        size_t previous_index_ = (current_index_ == 0) ? position_sequence_.size() - 1 : current_index_ - 1;
        double displacement_joint_1 = std::abs(position_sequence_[previous_index_][0] - position_sequence_[current_index_][0]);
        double displacement_joint_2 = std::abs(position_sequence_[previous_index_][1] - position_sequence_[current_index_][1]);
        double max_displacement_degrees = (displacement_joint_1 > displacement_joint_2) ? displacement_joint_1 : displacement_joint_2;
        double max_velocity = (joint_state_.velocity[0] > joint_state_.velocity[1]) ? joint_state_.velocity[0] : joint_state_.velocity[1];   
        double max_displacement_radians = max_displacement_degrees * M_PI / 180.0;
        double angular_velocity =  max_velocity * 2 * M_PI / 60.0;
        double time = max_displacement_radians / angular_velocity;
        RCLCPP_INFO(this->get_logger(), "Received time: %f", time);
        return time;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<std::vector<double>> position_sequence_;
    std::vector<double> default_joints_;
    std::vector<double> max_joints_;
    std::vector<double> min_joints_;
    size_t current_index_;
    double publish_interval_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}

