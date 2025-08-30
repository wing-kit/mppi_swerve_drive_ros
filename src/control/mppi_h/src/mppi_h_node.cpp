#include "mppi_h/mppi_h.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller_mppi_h::MPPI>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
