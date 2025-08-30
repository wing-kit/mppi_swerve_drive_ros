#include "vel_driver/vel_driver.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gazebo::VelDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
