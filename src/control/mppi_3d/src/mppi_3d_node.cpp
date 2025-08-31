#include "mppi_3d/mppi_3d.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller::MPPI>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
