#include "groundtruth_odom_publisher/groundtruth_odom_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gazebo::GroundTruthOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
