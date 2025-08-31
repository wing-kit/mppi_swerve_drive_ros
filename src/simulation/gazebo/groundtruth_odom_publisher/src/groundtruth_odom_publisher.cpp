#include "groundtruth_odom_publisher/groundtruth_odom_publisher.hpp"
using std::placeholders::_1;

namespace gazebo
{

// constructor
GroundTruthOdomPublisher::GroundTruthOdomPublisher()
    : rclcpp::Node("groundtruth_odom_publisher")
{
    // declare and get parameters
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("groundtruth_odom_topic", "/groundtruth_odom");

    base_frame_name = this->get_parameter("base_frame").as_string();
    odom_frame_name = this->get_parameter("odom_frame").as_string();

    // subscribing topic name
    std::string groundtruth_odom_topic = this->get_parameter("groundtruth_odom_topic").as_string();

    // initialize subscriber
    sub_groundtruth_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        groundtruth_odom_topic, rclcpp::QoS(10), std::bind(&GroundTruthOdomPublisher::groundTruthOdomCallback, this, _1));

    // initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

// destructor
GroundTruthOdomPublisher::~GroundTruthOdomPublisher()
{
    // No Contents
}

// /odom topic callback
void GroundTruthOdomPublisher::groundTruthOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // get current time
    rclcpp::Time time_now_ = this->now();

    // publish tf (avoiding publishing tf with same timestamp repeatedly)
    if(prev_tf_timestamp_ != time_now_)
    {
        // update cache
        prev_tf_timestamp_ = time_now_;

        // publish odom tf
        odom_tf_.header.stamp = time_now_.to_builtin_time();
        odom_tf_.header.frame_id = odom_frame_name;
        odom_tf_.child_frame_id = base_frame_name;
        odom_tf_.transform.translation.x = msg->pose.pose.position.x;
        odom_tf_.transform.translation.y = msg->pose.pose.position.y;
        odom_tf_.transform.translation.z = msg->pose.pose.position.z;
        odom_tf_.transform.rotation.x = msg->pose.pose.orientation.x;
        odom_tf_.transform.rotation.y = msg->pose.pose.orientation.y;
        odom_tf_.transform.rotation.z = msg->pose.pose.orientation.z;
        odom_tf_.transform.rotation.w = msg->pose.pose.orientation.w;
        tf_broadcaster_->sendTransform(odom_tf_);
    }
}

} // namespace gazebo
