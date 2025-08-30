#include "joy_controller/joy_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<operation::JoyController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
