#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TFMapBodyPublisher : public rclcpp::Node
{
public:
    TFMapBodyPublisher()
            : Node("tf_publisher_map_body")
    {
        // Declare and acquire `turtlename` parameter
        parent_frame_name_ = this->declare_parameter<std::string>("parent_frame", "map");
        child_frame_name_ = "base_link";
        // Initialize the transform broadcaster
        tf_broadcaster_ =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odometry", 10,
                std::bind(&TFMapBodyPublisher::odometryCallback, this, std::placeholders::_1));
        got_odometry_information_ = false;
        initial_tf_thread = std::thread(&TFMapBodyPublisher::publishInitialTF, this);
    }

private:
    void publishInitialTF(){
        geometry_msgs::msg::TransformStamped t;
        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame_name_;
        t.child_frame_id = child_frame_name_;

        t.transform.translation.x = 0;
        t.transform.translation.y = 0;
        t.transform.translation.z = 0;

        t.transform.rotation.w = 1.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        rclcpp::Clock tmp_clock(RCL_SYSTEM_TIME);
        auto stamp = tmp_clock.now();
        double freq = 20;
        // Send the transformation
        while (rclcpp::ok() && !got_odometry_information_){
            tf_broadcaster_->sendTransform(t);
            while ((tmp_clock.now() - stamp).seconds() < 1 / freq){

            }
        }
    }

    void odometryCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
        if (!got_odometry_information_){
            got_odometry_information_ = true;
        }
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame_name_;
        t.child_frame_id = child_frame_name_;

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        t.transform.rotation = msg->pose.pose.orientation;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string parent_frame_name_;
    std::string child_frame_name_;
    std::thread initial_tf_thread;
    bool got_odometry_information_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFMapBodyPublisher>());
    rclcpp::shutdown();
    return 0;
}