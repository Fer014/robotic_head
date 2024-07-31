#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <math.h>

class TFPublisherNode : public rclcpp::Node
{
public:
    TFPublisherNode() : Node("static_tf_publisher")
    {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Subscribe to the present tf angles topic
        tf_angles_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
            std::bind(&TFPublisherNode::angleCallback, this, std::placeholders::_1));

        // Initialize present positions
        angle1 = 0;
        angle2 = 0;

        // Publish static transforms
        publish_static_transforms();
    }

private:
	void angleCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
		angle1 = msg->position[0];
		angle2 = msg->position[1];
		RCLCPP_INFO(this->get_logger(), "angle1: %d, angle2: %d", angle1, angle2);
		
		// Republish static transforms when positions are updated
        publish_static_transforms();
	}

    void publish_static_transforms()
    {
        // Clear previous static transforms
        static_transforms_.clear();

        // Static transform 1 - Neck to DX Link
        geometry_msgs::msg::TransformStamped static_transform_stamped1;
        static_transform_stamped1.header.stamp = this->now();
        static_transform_stamped1.header.frame_id = "neck_link";
        static_transform_stamped1.child_frame_id = "dx_link";
        static_transform_stamped1.transform.translation.x = -0.001621;
        static_transform_stamped1.transform.translation.z = 0.1195;
        // Set dynamic rotation around Z-axis using angle_1
        tf2::Quaternion quat1;
        quat1.setRPY(0, 0, angle1*(M_PI/180));
        static_transform_stamped1.transform.rotation.x = quat1.x();
        static_transform_stamped1.transform.rotation.y = quat1.y();
        static_transform_stamped1.transform.rotation.z = quat1.z();
        static_transform_stamped1.transform.rotation.w = quat1.w();
        static_transforms_.push_back(static_transform_stamped1);

        // Static transform 2 - DX to Tilt Link
        geometry_msgs::msg::TransformStamped static_transform_stamped2;
        static_transform_stamped2.header.stamp = this->now();
        static_transform_stamped2.header.frame_id = "dx_link";
        static_transform_stamped2.child_frame_id = "tilt_link";
        static_transform_stamped2.transform.translation.x = 0.024;
        // Set dynamic rotation around Z-axis using angle_2
        tf2::Quaternion quat2;
        quat2.setRPY(0, -angle2*(M_PI/180), 0); // Roll, Pitch, Yaw
        static_transform_stamped2.transform.rotation.x = quat2.x();
        static_transform_stamped2.transform.rotation.y = quat2.y();
        static_transform_stamped2.transform.rotation.z = quat2.z();
        static_transform_stamped2.transform.rotation.w = quat2.w();
        static_transforms_.push_back(static_transform_stamped2);

        // Static transform 3 - Tilt to Head Link
        geometry_msgs::msg::TransformStamped static_transform_stamped3;
        static_transform_stamped3.header.stamp = this->now();
        static_transform_stamped3.header.frame_id = "tilt_link";
        static_transform_stamped3.child_frame_id = "head_link";
        static_transform_stamped3.transform.translation.x = 0.028;
        // Set rotation around Z-axis using present_position_2
        tf2::Quaternion quat3;
        quat3.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        static_transform_stamped3.transform.rotation.x = quat3.x();
        static_transform_stamped3.transform.rotation.y = quat3.y();
        static_transform_stamped3.transform.rotation.z = quat3.z();
        static_transform_stamped3.transform.rotation.w = quat3.w();
        static_transforms_.push_back(static_transform_stamped3);

        // Static transform 4 - Head to Camera Optical Link (D435_head)
	    geometry_msgs::msg::TransformStamped static_transform_stamped4;
	    static_transform_stamped4.header.stamp = this->now();
	    static_transform_stamped4.header.frame_id = "head_link";
	    static_transform_stamped4.child_frame_id = "D435_head_optical_link";
        // Set translation and rotation obtained from eye-in-hand camera calibration
	    static_transform_stamped4.transform.translation.x = 0.0180;
	    static_transform_stamped4.transform.translation.y = 0.0336;
	    static_transform_stamped4.transform.translation.z = 0.0297;
	    static_transform_stamped4.transform.rotation.x = -0.4931;
	    static_transform_stamped4.transform.rotation.y = 0.4960;
	    static_transform_stamped4.transform.rotation.z = -0.5029;
	    static_transform_stamped4.transform.rotation.w = 0.5079;
	    static_transforms_.push_back(static_transform_stamped4);

        // Static transform 5 - Camera Optical to Camera ROS2 Link (D435_head)
        geometry_msgs::msg::TransformStamped static_transform_stamped5;
	    static_transform_stamped5.header.stamp = this->now();
	    static_transform_stamped5.header.frame_id = "D435_head_optical_link";
	    static_transform_stamped5.child_frame_id = "D435_head_link";
        // Set rotation from optical frame to ROS2 frame
        tf2::Quaternion quat5;
        quat5.setRPY(90*(M_PI/180), -90*(M_PI/180), 0); // Roll, Pitch, Yaw
	    static_transform_stamped5.transform.rotation.x = quat5.x();
        static_transform_stamped5.transform.rotation.y = quat5.y();
        static_transform_stamped5.transform.rotation.z = quat5.z();
        static_transform_stamped5.transform.rotation.w = quat5.w();
	    static_transforms_.push_back(static_transform_stamped5);

        // Static transform 6 - Neck to Camera Optical Link (D435_ceiling)
        geometry_msgs::msg::TransformStamped static_transform_stamped6;
	    static_transform_stamped6.header.stamp = this->now();
	    static_transform_stamped6.header.frame_id = "neck_link";
	    static_transform_stamped6.child_frame_id = "D435_ceiling_optical_link";
        // Set translation and rotation obtained from ArUco camera calibration
        static_transform_stamped6.transform.translation.x = 0.3307;
        static_transform_stamped6.transform.translation.y = -0.1169;
        static_transform_stamped6.transform.translation.z = 0.9008;
        tf2::Quaternion quat6;
        quat6.setRPY(179.1301*(M_PI/180), -0.3724*(M_PI/180), -90.1271*(M_PI/180)); // Roll, Pitch, Yaw 
	    static_transform_stamped6.transform.rotation.x = quat6.x();
        static_transform_stamped6.transform.rotation.y = quat6.y();
        static_transform_stamped6.transform.rotation.z = quat6.z();
        static_transform_stamped6.transform.rotation.w = quat6.w();
	    static_transforms_.push_back(static_transform_stamped6);

        /*
        Marker ID: [9]
          Translation (x, y, z): ([0.131322], [-0.32451127], [0.90106753])
          Rotation (roll, pitch, yaw): (179.6256826590625, -0.8690713942656215, -0.12712198754221418)
         In Matlab:
        translation_neck2cam = 0.3307, -0.1169, 0.9008
        eul_neck2cam_degrees = 179.1301   -0.3724  -90.1271
        */

        // Static transform 7 - Camera Optical to Camera ROS2 Link (D435_ceiling)
        geometry_msgs::msg::TransformStamped static_transform_stamped7;
	    static_transform_stamped7.header.stamp = this->now();
	    static_transform_stamped7.header.frame_id = "D435_ceiling_optical_link";
	    static_transform_stamped7.child_frame_id = "D435_ceiling_link";
        // Set rotation from optical frame to ROS2 frame
        tf2::Quaternion quat7;
        quat7.setRPY(90*(M_PI/180), -90*(M_PI/180), 0); // Roll, Pitch, Yaw 
	    static_transform_stamped7.transform.rotation.x = quat7.x();
        static_transform_stamped7.transform.rotation.y = quat7.y();
        static_transform_stamped7.transform.rotation.z = quat7.z();
        static_transform_stamped7.transform.rotation.w = quat7.w();
	    static_transforms_.push_back(static_transform_stamped7);

        // Publish all static transforms
        for (const auto &static_transform : static_transforms_)
        {
            static_broadcaster_->sendTransform(static_transform);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tf_angles_subscriber_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms_;
    int angle1;
    int angle2;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
