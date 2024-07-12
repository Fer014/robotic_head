#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class DigitalTwinBridge : public rclcpp::Node
{
public:
    DigitalTwinBridge()
        : Node("digital_twin_bridge")
    {
        // Suscribirse al topic de los estados de las articulaciones simuladas
        subscription_simulated_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&DigitalTwinBridge::simulated_joint_states_callback, this, std::placeholders::_1));

        // Suscribirse al topic de los comandos de las articulaciones
        subscription_joint_command_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_command", 10, std::bind(&DigitalTwinBridge::joint_command_callback, this, std::placeholders::_1));

        // Publicar en el topic de los comandos simulados
        publisher_simulated_joint_command_ = this->create_publisher<sensor_msgs::msg::JointState>("simulated_joint_command", 10);
    }

private:
    void simulated_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received simulated_joint_states: '%f, %f'", msg->position[0] * M_PI / 180.0, msg->position[1] * M_PI / 180.0);

        // Aquí puedes procesar los estados de las articulaciones simuladas si es necesario
    }

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received joint_command: '%f, %f'", msg->position[0], msg->position[1]);

        // Crear un mensaje JointState para el comando simulado
        auto simulated_joint_command_msg = sensor_msgs::msg::JointState();
        simulated_joint_command_msg.header.stamp = this->get_clock()->now();
        simulated_joint_command_msg.name = msg->name;
        
        // Convertir las posiciones de grados a radianes
        std::vector<double> positions_in_radians;
        for (const auto& position : msg->position) {
            positions_in_radians.push_back(position * M_PI / 180.0);
        }
        simulated_joint_command_msg.position = positions_in_radians;

        // Publicar el mensaje JointState
        publisher_simulated_joint_command_->publish(simulated_joint_command_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_simulated_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_command_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_simulated_joint_command_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigitalTwinBridge>());
    rclcpp::shutdown();
    return 0;
}

