#include "mppi_4d/mppi_4d.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller::MPPI>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
