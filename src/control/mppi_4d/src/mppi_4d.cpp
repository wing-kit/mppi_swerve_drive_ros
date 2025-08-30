#include "mppi_4d/mppi_4d.hpp"
using std::placeholders::_1;

namespace controller
{

// constructor
MPPI::MPPI()
    : rclcpp::Node("mppi_4d")
{
    // load parameters
    param::Param param;
    //// navigation
    this->declare_parameter<double>("navigation/xy_goal_tolerance", 0.5);
    this->declare_parameter<double>("navigation/yaw_goal_tolerance", 0.5);
    param.navigation.xy_goal_tolerance = this->get_parameter("navigation/xy_goal_tolerance").as_double();
    param.navigation.yaw_goal_tolerance = this->get_parameter("navigation/yaw_goal_tolerance").as_double();
    
    //// target_system
    this->declare_parameter<double>("target_system/l_f", 0.5);
    this->declare_parameter<double>("target_system/l_r", 0.5);
    this->declare_parameter<double>("target_system/d_l", 0.5);
    this->declare_parameter<double>("target_system/d_r", 0.5);
    this->declare_parameter<double>("target_system/tire_radius", 0.2);
    param.target_system.l_f = this->get_parameter("target_system/l_f").as_double();
    param.target_system.l_r = this->get_parameter("target_system/l_r").as_double();
    param.target_system.d_l = this->get_parameter("target_system/d_l").as_double();
    param.target_system.d_r = this->get_parameter("target_system/d_r").as_double();
    param.target_system.tire_radius = this->get_parameter("target_system/tire_radius").as_double();

    //// controller
    this->declare_parameter<std::string>("controller/name", "mppi_4d");
    this->declare_parameter<double>("controller/control_interval", 0.05);
    this->declare_parameter<int>("controller/num_samples", 3000);
    this->declare_parameter<int>("controller/prediction_horizon", 30);
    this->declare_parameter<double>("controller/step_len_sec", 0.033);
    this->declare_parameter<double>("controller/param_exploration", 0.1);
    this->declare_parameter<double>("controller/param_lambda", 0.1);
    this->declare_parameter<double>("controller/param_alpha", 0.1);
    this->declare_parameter<std::vector<double>>("controller/sigma", {1.0, 1.0, 0.78});
    this->declare_parameter<bool>("controller/reduce_computation", false);
    this->declare_parameter<std::vector<double>>("controller/weight_cmd_change", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("controller/weight_vehicle_cmd_change", {1.4, 1.4, 1.4, 1.4, 0.1, 0.1, 0.1, 0.1});
    this->declare_parameter<double>("controller/ref_velocity", 2.0);
    this->declare_parameter<double>("controller/weight_velocity_error", 10.0);
    this->declare_parameter<double>("controller/weight_angular_error", 30.0);
    this->declare_parameter<double>("controller/weight_collision_penalty", 50.0);
    this->declare_parameter<double>("controller/weight_distance_error_penalty", 40.0);
    this->declare_parameter<double>("controller/weight_terminal_state_penalty", 50.0);
    this->declare_parameter<bool>("controller/use_sg_filter", true);
    this->declare_parameter<int>("controller/sg_filter_half_window_size", 10);
    this->declare_parameter<int>("controller/sg_filter_poly_order", 3);

    param.controller.name = this->get_parameter("controller/name").as_string();
    param.controller.control_interval = this->get_parameter("controller/control_interval").as_double();
    param.controller.num_samples = this->get_parameter("controller/num_samples").as_int();
    param.controller.prediction_horizon = this->get_parameter("controller/prediction_horizon").as_int();
    param.controller.step_len_sec = this->get_parameter("controller/step_len_sec").as_double();
    param.controller.param_exploration = this->get_parameter("controller/param_exploration").as_double();
    param.controller.param_lambda = this->get_parameter("controller/param_lambda").as_double();
    param.controller.param_alpha = this->get_parameter("controller/param_alpha").as_double();
    param.controller.sigma = this->get_parameter("controller/sigma").as_double_array();
    param.controller.reduce_computation = this->get_parameter("controller/reduce_computation").as_bool();
    param.controller.weight_cmd_change = this->get_parameter("controller/weight_cmd_change").as_double_array();
    param.controller.weight_vehicle_cmd_change = this->get_parameter("controller/weight_vehicle_cmd_change").as_double_array();
    param.controller.ref_velocity = this->get_parameter("controller/ref_velocity").as_double();
    param.controller.weight_velocity_error = this->get_parameter("controller/weight_velocity_error").as_double();
    param.controller.weight_angular_error = this->get_parameter("controller/weight_angular_error").as_double();
    param.controller.weight_collision_penalty = this->get_parameter("controller/weight_collision_penalty").as_double();
    param.controller.weight_distance_error_penalty = this->get_parameter("controller/weight_distance_error_penalty").as_double();
    param.controller.weight_terminal_state_penalty = this->get_parameter("controller/weight_terminal_state_penalty").as_double();
    param.controller.use_sg_filter = this->get_parameter("controller/use_sg_filter").as_bool();
    param.controller.sg_filter_half_window_size = this->get_parameter("controller/sg_filter_half_window_size").as_int();
    param.controller.sg_filter_poly_order = this->get_parameter("controller/sg_filter_poly_order").as_int();

    //// subscribing topic names
    std::string odom_topic, ref_path_topic, collision_costmap_topic, distance_error_map_topic, ref_yaw_map_topic;
    this->declare_parameter<std::string>("odom_topic", "/groundtruth_odom");
    this->declare_parameter<std::string>("ref_path_topic", "/move_base/NavfnROS/plan");
    this->declare_parameter<std::string>("collision_costmap_topic", "/move_base/local_costmap/costmap");
    this->declare_parameter<std::string>("distance_error_map_topic", "/distance_error_map");
    this->declare_parameter<std::string>("ref_yaw_map_topic", "/ref_yaw_map");
    odom_topic = this->get_parameter("odom_topic").as_string();
    ref_path_topic = this->get_parameter("ref_path_topic").as_string();
    collision_costmap_topic = this->get_parameter("collision_costmap_topic").as_string();
    distance_error_map_topic = this->get_parameter("distance_error_map_topic").as_string();
    ref_yaw_map_topic = this->get_parameter("ref_yaw_map_topic").as_string();

    //// publishing topic names
    std::string control_cmd_vel_topic, mppi_absvel_topic, mppi_vx_topic, mppi_vy_topic, mppi_omega_topic, \
    calc_time_topic, mppi_optimal_traj_topic, mppi_sampled_traj_topic, mppi_overlay_text_topic, mppi_eval_msg_topic;
    this->declare_parameter<std::string>("control_cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("mppi_absvel_topic", "/mppi/cmd/absvel");
    this->declare_parameter<std::string>("mppi_vx_topic", "/mppi/cmd/vx");
    this->declare_parameter<std::string>("mppi_vy_topic", "/mppi/cmd/vy");
    this->declare_parameter<std::string>("mppi_omega_topic", "/mppi/cmd/omega");
    this->declare_parameter<std::string>("calc_time_topic", "/mppi/calc_time");
    this->declare_parameter<std::string>("mppi_overlay_text_topic", "/mppi/overlay_text");
    this->declare_parameter<std::string>("mppi_optimal_traj_topic", "/mppi/optimal_traj");
    this->declare_parameter<std::string>("mppi_sampled_traj_topic", "/mppi/sampled_traj");
    this->declare_parameter<std::string>("mppi_eval_msg_topic", "/mppi/eval_info");
    control_cmd_vel_topic = this->get_parameter("control_cmd_vel_topic").as_string();
    mppi_absvel_topic = this->get_parameter("mppi_absvel_topic").as_string();
    mppi_vx_topic = this->get_parameter("mppi_vx_topic").as_string();
    mppi_vy_topic = this->get_parameter("mppi_vy_topic").as_string();
    mppi_omega_topic = this->get_parameter("mppi_omega_topic").as_string();
    calc_time_topic = this->get_parameter("calc_time_topic").as_string();
    mppi_overlay_text_topic = this->get_parameter("mppi_overlay_text_topic").as_string();
    mppi_optimal_traj_topic = this->get_parameter("mppi_optimal_traj_topic").as_string();
    mppi_sampled_traj_topic = this->get_parameter("mppi_sampled_traj_topic").as_string();
    mppi_eval_msg_topic = this->get_parameter("mppi_eval_msg_topic").as_string();

    // initialize subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10), std::bind(&MPPI::odomCallback, this, _1));
    odom_received_ = false;
    sub_ref_path_ = this->create_subscription<nav_msgs::msg::Path>(ref_path_topic, rclcpp::QoS(10), std::bind(&MPPI::refPathCallback, this, _1));
    ref_path_received_ = false;
    sub_collision_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(collision_costmap_topic, rclcpp::QoS(10), std::bind(&MPPI::collisionCostmapCallback, this, _1));
    collision_costmap_received_ = false;
    sub_distance_error_map_ = this->create_subscription<grid_map_msgs::msg::GridMap>(distance_error_map_topic, rclcpp::QoS(10), std::bind(&MPPI::distanceErrorMapCallback, this, _1));
    distance_error_map_received_ = false;
    sub_ref_yaw_map_ = this->create_subscription<grid_map_msgs::msg::GridMap>(ref_yaw_map_topic, rclcpp::QoS(10), std::bind(&MPPI::refYawMapCallback, this, _1));
    ref_yaw_map_received_ = false;

    // initialize publishers
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(control_cmd_vel_topic, rclcpp::QoS(10));
    pub_cmd_absvel_ = this->create_publisher<std_msgs::msg::Float32>(mppi_absvel_topic, rclcpp::QoS(10));
    pub_cmd_vx_ = this->create_publisher<std_msgs::msg::Float32>(mppi_vx_topic, rclcpp::QoS(10));
    pub_cmd_vy_ = this->create_publisher<std_msgs::msg::Float32>(mppi_vy_topic, rclcpp::QoS(10));
    pub_cmd_omega_ = this->create_publisher<std_msgs::msg::Float32>(mppi_omega_topic, rclcpp::QoS(10));
    pub_mppi_calc_time_ = this->create_publisher<std_msgs::msg::Float32>(calc_time_topic, rclcpp::QoS(10));
    pub_mppi_overlay_text_ = this->create_publisher<visualization_msgs::msg::Marker>(mppi_overlay_text_topic, rclcpp::QoS(10));
    pub_mppi_optimal_traj_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(mppi_optimal_traj_topic, rclcpp::QoS(10));
    pub_mppi_sampled_traj_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(mppi_sampled_traj_topic, rclcpp::QoS(10));
    pub_mppi_eval_msg_ = this->create_publisher<mppi_eval_msgs::msg::MPPIEval>(mppi_eval_msg_topic, rclcpp::QoS(10));

    // initialize timer
    timer_control_interval_ = this->create_wall_timer(std::chrono::duration<double>(param.controller.control_interval), std::bind(&MPPI::calcControlCommand, this));

    // instantiate MPPICore class
    mppi_core_ = new controller::MPPICore(param);
}

// destructor
MPPI::~MPPI()
{
    // No Contents
}

// callback to update odometry (global vehicle pose)
void MPPI::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_received_ = true;
    latest_odom_ = *msg;

    // get latest vehicle pose
    double latest_x = latest_odom_.pose.pose.position.x;
    double latest_y = latest_odom_.pose.pose.position.y;
    double latest_yaw = tf2::getYaw(latest_odom_.pose.pose.orientation);

    // check if NaN is included
    if (std::isnan(latest_x) || std::isnan(latest_y) || std::isnan(latest_yaw))
    {
        RCLCPP_WARN(this->get_logger(), "NaN is included in the received odometry");
        return;
    }

    // update observed state
    observed_state_.x = latest_x;
    observed_state_.y = latest_y;
    observed_state_.yaw = latest_yaw;
    observed_state_.unwrap();
}

// callback to update reference path
void MPPI::refPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    ref_path_received_ = true;
    latest_ref_path_ = *msg;

    // update goal state
    int goal_idx = latest_ref_path_.poses.size() - 1;
    goal_state_.x = latest_ref_path_.poses[goal_idx].pose.position.x;
    goal_state_.y = latest_ref_path_.poses[goal_idx].pose.position.y;
    goal_state_.yaw = tf2::getYaw(latest_ref_path_.poses[goal_idx].pose.orientation);
}

// callback to update local costmap callback
void MPPI::collisionCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    collision_costmap_received_ = true;

    // convert subscribed OccupancyGrid to GridMap type
    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "collision_cost", collision_costmap_);
}

// callback to update distance error map
void MPPI::distanceErrorMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
    distance_error_map_received_ = true;

    // convert subscribed GridMap to GridMap type
    grid_map::GridMapRosConverter::fromMessage(*msg, distance_error_map_);
}

// callback to update reference yaw map
void MPPI::refYawMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
    ref_yaw_map_received_ = true;

    // convert subscribed GridMap to GridMap type
    grid_map::GridMapRosConverter::fromMessage(*msg, ref_yaw_map_);
}

// callback to calculate control command
void MPPI::calcControlCommand()
{

    // check if all necessary data are received, and return if not.
    if (!odom_received_ || !ref_path_received_ || !collision_costmap_received_ || !distance_error_map_received_ || !ref_yaw_map_received_)
    {
        RCLCPP_WARN(this->get_logger(), "[MPPI] not all necessary data are received, odom: %d, ref_path: %d, collision_costmap: %d, distance_error_map: %d, ref_yaw_map: %d", \
        odom_received_, ref_path_received_, collision_costmap_received_, distance_error_map_received_, ref_yaw_map_received_);
        return;
    }

    // calculate optimal control command
    common_type::VxVyOmega optimal_cmd = 
        mppi_core_->solveMPPI(
            observed_state_,
            collision_costmap_,
            distance_error_map_,
            ref_yaw_map_,
            goal_state_
    );

    // publish optimal control command as Twist message
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = optimal_cmd.vx;
    cmd_vel.linear.y = optimal_cmd.vy;
    cmd_vel.angular.z = optimal_cmd.omega;
    pub_cmd_vel_->publish(cmd_vel);

    // publish rviz markers for visualization
    //// publish optimal trajectory
    publishOptimalTrajectory(mppi_core_->getOptimalTrajectory());
    //// publish sampled trajectories
    // publishSampledTrajectories(mppi_core_->getFullSampledTrajectories()); // visualize all sampled trajectories
    publishSampledTrajectories(mppi_core_->getEliteSampledTrajectories(100)); // visualize top 100 sampled trajectories

    // publish mppi calculation time
    std_msgs::msg::Float32 calc_time;
    calc_time.data = mppi_core_->getCalcTime();
    pub_mppi_calc_time_->publish(calc_time);

    // publish overlay text
    publishOverlayText(mppi_core_->getControllerName());

    // publish velocity command info
    std_msgs::msg::Float32 absvel, vx, vy, omega;
    absvel.data = sqrt(pow(optimal_cmd.vx, 2) + pow(optimal_cmd.vy, 2));
    vx.data = optimal_cmd.vx;
    vy.data = optimal_cmd.vy;
    omega.data = optimal_cmd.omega;
    pub_cmd_absvel_->publish(absvel);
    pub_cmd_vx_->publish(vx);
    pub_cmd_vy_->publish(vy);
    pub_cmd_omega_->publish(omega);

    // publish mppi evaluation info
    mppi_eval_msgs::msg::MPPIEval mppi_eval_msg;
    mppi_eval_msg.header.stamp = this->now().to_builtin_time();
    mppi_eval_msg.header.frame_id = mppi_core_->getControllerName();
    mppi_eval_msg.state_cost = mppi_core_->getStateCost();
    mppi_eval_msg.global_x = observed_state_.x;
    mppi_eval_msg.global_y = observed_state_.y;
    mppi_eval_msg.global_yaw = observed_state_.yaw;
    mppi_eval_msg.cmd_vx = optimal_cmd.vx;
    mppi_eval_msg.cmd_vy = optimal_cmd.vy;
    mppi_eval_msg.cmd_yawrate = optimal_cmd.omega;
    common_type::VehicleCommand8D optimal_vehicle_cmd = mppi_core_->getOptimalVehicleCommand();
    mppi_eval_msg.cmd_steer_fl = optimal_vehicle_cmd.steer_fl;
    mppi_eval_msg.cmd_steer_fr = optimal_vehicle_cmd.steer_fr;
    mppi_eval_msg.cmd_steer_rl = optimal_vehicle_cmd.steer_rl;
    mppi_eval_msg.cmd_steer_rr = optimal_vehicle_cmd.steer_rr;
    mppi_eval_msg.cmd_rotor_fl = optimal_vehicle_cmd.rotor_fl;
    mppi_eval_msg.cmd_rotor_fr = optimal_vehicle_cmd.rotor_fr;
    mppi_eval_msg.cmd_rotor_rl = optimal_vehicle_cmd.rotor_rl;
    mppi_eval_msg.cmd_rotor_rr = optimal_vehicle_cmd.rotor_rr;
    mppi_eval_msg.calc_time_ms = mppi_core_->getCalcTime();
    mppi_eval_msg.goal_reached = mppi_core_->isGoalReached();
    pub_mppi_eval_msg_->publish(mppi_eval_msg);
}

// publish rviz markers to visualize optimal trajectory with arrow markers
void MPPI::publishOptimalTrajectory(const std::vector<common_type::XYYaw>& optimal_xyyaw_sequence)
{
    // constant params
    //// arrow height
    double MARKER_POS_Z = 0.41; // [m]
    //// arrow scale (x, y, z)
    double arrow_scale[3] = {0.10, 0.03, 0.03};
    //// arrow color (red, green, blue, alpha)
    double arrow_color[4] = {1.0, 0.0, 0.0, 1.0};

    // create marker array
    visualization_msgs::msg::MarkerArray marker_array;
    // get number of time steps
    int T = optimal_xyyaw_sequence.size();
    marker_array.markers.resize(T);

    // for each time step, add an arrow marker
    for (int t = 0; t < T; t++)
    {
        double x = optimal_xyyaw_sequence[t].x;
        double y = optimal_xyyaw_sequence[t].y;
        double yaw = optimal_xyyaw_sequence[t].yaw;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw); // Note: assume roll and pitch angles are zero

        marker_array.markers[t].header.frame_id = "map";
        marker_array.markers[t].header.stamp = this->now().to_builtin_time();
        marker_array.markers[t].ns = "optimal_trajectory";
        marker_array.markers[t].id = t;
        marker_array.markers[t].type = visualization_msgs::msg::Marker::ARROW;
        marker_array.markers[t].action = visualization_msgs::msg::Marker::ADD;
        marker_array.markers[t].pose.position.x = x;
        marker_array.markers[t].pose.position.y = y;
        marker_array.markers[t].pose.position.z = MARKER_POS_Z;
        marker_array.markers[t].pose.orientation.x = q.x();
        marker_array.markers[t].pose.orientation.y = q.y();
        marker_array.markers[t].pose.orientation.z = q.z();
        marker_array.markers[t].pose.orientation.w = q.w();
        marker_array.markers[t].scale.x = arrow_scale[0];
        marker_array.markers[t].scale.y = arrow_scale[1];
        marker_array.markers[t].scale.z = arrow_scale[2];
        marker_array.markers[t].color.r = arrow_color[0];
        marker_array.markers[t].color.g = arrow_color[1];
        marker_array.markers[t].color.b = arrow_color[2];
        marker_array.markers[t].color.a = arrow_color[3];
    }

    // publish rviz markers
    pub_mppi_optimal_traj_->publish(marker_array);
}

// publish rviz markers to visualize sampled trajectories
void MPPI::publishSampledTrajectories(const std::vector<std::vector<common_type::XYYaw>>& sampled_state_sequences)
{
    // constant params
    //// arrow height
    double MARKER_POS_Z = 0.39; // [m]
    //// line lifetime [s]
    double line_lifetime = 0.1;
    //// arrow scale (x, y, z)
    double line_width = 0.01;
    //// line color (red, green, blue, alpha)
    double line_color[4] = {0.0, 0.35, 1.0, 0.5};

    // get number of samples
    int K = sampled_state_sequences.size();
    int T = sampled_state_sequences[0].size();

    // create marker array
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(K);

    // for each sampled state sequence, add an line strip marker
    for (int k = 0; k < K; k++)
    {
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now().to_builtin_time();
        line.ns = "sampled_trajectories";
        line.id = k;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.x = 0.0;
        line.pose.orientation.y = 0.0;
        line.pose.orientation.z = 0.0;
        line.pose.orientation.w = 1.0;
        line.scale.x = line_width;
        line.color.r = line_color[0];
        line.color.g = line_color[1];
        line.color.b = line_color[2];
        line.color.a = line_color[3];
        line.lifetime = rclcpp::Duration::from_seconds(line_lifetime);
        line.points.resize(T);

        // for each time step, add a point to the line strip marker
        for (int t = 0; t < T; t++)
        {
            // add a point to the line strip marker
            line.points[t].x = sampled_state_sequences[k][t].x;
            line.points[t].y = sampled_state_sequences[k][t].y;
            line.points[t].z = MARKER_POS_Z;
        }

        marker_array.markers[k] = line;
    }

    // publish rviz markers
    pub_mppi_sampled_traj_->publish(marker_array);
}

// publish overlay text for visualization on rviz
void MPPI::publishOverlayText(const std::string& text)
{
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = this->now().to_builtin_time();
    text_marker.ns = "mppi_text";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 0.0;
    text_marker.pose.position.y = 0.0;
    text_marker.pose.position.z = 1.5;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 25.0 / 255;
    text_marker.color.g = 255.0 / 255;
    text_marker.color.b = 240.0 / 255;
    text_marker.color.a = 0.8;
    text_marker.text = text;
    pub_mppi_overlay_text_->publish(text_marker);
}

} // namespace controller
