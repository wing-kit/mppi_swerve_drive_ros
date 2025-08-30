#include "joy_controller/joy_controller.hpp"
using std::placeholders::_1;

namespace operation
{

//constructor
JoyController::JoyController()
    : rclcpp::Node("joy_controller")
{
    // declare parameters 
    this->declare_parameter<int>("joy_top_left_button_idx", 4);
    this->declare_parameter<int>("joy_top_right_button_idx", 5);
    this->declare_parameter<int>("joy_left_stick_x_idx", 0);
    this->declare_parameter<int>("joy_left_stick_y_idx", 1);
    this->declare_parameter<int>("joy_right_stick_x_idx", 3);
    this->declare_parameter<int>("joy_right_stick_y_idx", 4);
    this->declare_parameter<double>("abs_max_linear_vel_x", 1.5);
    this->declare_parameter<double>("abs_max_linear_vel_y", 1.5);
    this->declare_parameter<double>("abs_max_angular_vel_z", 3.14);
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("control_cmd_vel_topic", "/cmd_vel");

    // get parameters
    joy_top_left_button_idx = this->get_parameter("joy_top_left_button_idx").as_int();
    joy_top_right_button_idx = this->get_parameter("joy_top_right_button_idx").as_int();
    joy_left_stick_x_idx = this->get_parameter("joy_left_stick_x_idx").as_int();
    joy_left_stick_y_idx = this->get_parameter("joy_left_stick_y_idx").as_int();
    joy_right_stick_x_idx = this->get_parameter("joy_right_stick_x_idx").as_int();
    joy_right_stick_y_idx = this->get_parameter("joy_right_stick_y_idx").as_int();

    abs_max_linear_vel_x = static_cast<float>(this->get_parameter("abs_max_linear_vel_x").as_double());
    abs_max_linear_vel_y = static_cast<float>(this->get_parameter("abs_max_linear_vel_y").as_double());
    abs_max_angular_vel_z = static_cast<float>(this->get_parameter("abs_max_angular_vel_z").as_double());

    // subscribing topic name
    std::string joy_topic = this->get_parameter("joy_topic").as_string();

    // publishing topic name
    std::string control_cmd_vel_topic = this->get_parameter("control_cmd_vel_topic").as_string();

    // initialize subscriber
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, rclcpp::QoS(10), std::bind(&JoyController::joyCallback, this, _1));

    // initialize publisher
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(control_cmd_vel_topic, rclcpp::QoS(10));
}

// destructor
JoyController::~JoyController()
{
    // No Contents
}

// /joy topic callback
void JoyController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // parse received joy message
    float scale = 1.0f;
    if (msg->buttons[joy_top_left_button_idx]) // scale down the command (0.5x)
    {
        scale = scale * 0.5f;
    }
    if (msg->buttons[joy_top_right_button_idx]) // scale up the command (2.0x)
    {
        scale = scale * 2.0f;
    }
    float val_left_stick_x  = scale * msg->axes[joy_left_stick_x_idx] * (-1.0f);
    float val_left_stick_y  = scale * msg->axes[joy_left_stick_y_idx];
    float val_right_stick_x = scale * msg->axes[joy_right_stick_x_idx];
    float val_right_stick_y = scale * msg->axes[joy_right_stick_y_idx];

    // [for debug] annouce received joy message
    ROS_DEBUG("Received Joy Command: Lstick_x = %+5.1f, Lstick_y = %+5.1f, Rstick_x = %+5.1f, Rstick_y = %+5.1f", val_left_stick_x, val_left_stick_y, val_right_stick_x, val_right_stick_y);

    // calculate Twist message to publish
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x =  abs_max_linear_vel_x * val_left_stick_y; // forward : positive, backward : negative
    cmd_vel.linear.y = -abs_max_linear_vel_y * val_left_stick_x; // left    : positive, right    : negative
    cmd_vel.linear.z =  0.0f;
    cmd_vel.angular.x = 0.0f;
    cmd_vel.angular.y = 0.0f;
    cmd_vel.angular.z = abs_max_angular_vel_z * val_right_stick_x; // left    : positive, right    : negative

    // [for debug] annouce publishing Twist message
    ROS_DEBUG("Send Twist Command: linear_x = %+5.1f, linear_y = %+5.1f, angular_z = %+5.1f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // publish Twist message
    pub_cmd_vel_->publish(cmd_vel);
}

} // namespace operation