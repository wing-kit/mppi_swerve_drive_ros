#include "world_handler/world_handler.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gazebo::WorldHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
