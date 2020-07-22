
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class Axes : public rclcpp::Node {
public:
    Axes(std::string parent, std::string child) : 
        Node("axes_node"), parent_(parent), child_(child) {

            geometry_msgs::msg::TransformStamped tf_msg;
            tf2::Quaternion quat;

            // Euler (z-y-x) to Quaternion.
            quat.setRPY(0.0, 0.0, 1.5607);

            // Create the message as specified by the documentation: 
            // http://docs.ros.org/diamondback/api/geometry_msgs/html/msg/TransformStamped.html
            // Note that the header.seq is deprecated as of ROS2.
            // https://github.com/ros2/common_interfaces/pull/2
            tf_msg.header.stamp = this->now();
            tf_msg.transform.translation.x = 2.0;
            tf_msg.transform.translation.y = 0.0;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation.x = quat.x();
            tf_msg.transform.rotation.y = quat.y();
            tf_msg.transform.rotation.z = quat.z();
            tf_msg.transform.rotation.w = quat.w();
            tf_msg.header.frame_id = parent_;
            tf_msg.child_frame_id = child_;

            broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

            // send transform
            broadcaster_->sendTransform(tf_msg);
        }

private:
    std::string parent_;
    std::string child_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // We create an instance of our node that creates a `new_frame` relative
    // to the frame `map`. The transformation between them is hardcoded above.
    auto axes_node = std::make_shared<Axes>("map", "new_frame");
    rclcpp::spin(axes_node);
    rclcpp::shutdown();
    return 0;
}

