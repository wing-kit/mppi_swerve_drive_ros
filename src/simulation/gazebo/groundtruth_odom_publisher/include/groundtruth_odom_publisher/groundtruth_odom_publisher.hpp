#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gazebo
{
    class GroundTruthOdomPublisher : public rclcpp::Node
    {
        public:
            GroundTruthOdomPublisher();
            ~GroundTruthOdomPublisher();
            void groundTruthOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        private:
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_groundtruth_odom_;
            rclcpp::Time prev_tf_timestamp_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            geometry_msgs::msg::TransformStamped odom_tf_;
            std::string base_frame_name, odom_frame_name;
    };
}