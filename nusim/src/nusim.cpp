/// PARAMETERS:
///     rate (double): frequency of the timer, in Hz
///     x0 (double): starting x location of the turtlebot (m)
///     y0 (double): starting y location of the turtlebot (m)
///     theta0 (double): starting theta location of the turtlebot (rad)
///     obstacles/x (double[]): list of x coordinates of cylindrical obstacles (m)
///     obstacles/y (double[]): list of r coordinates of cylindrical obstacles (m)
///     obstacles/r (double): radius of cylindrical obstacles (m)
///     arena_x_length : X length of rectangular arena (m)
///     arena_y_length : Y length of rectangular arena (m)
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): current timestep of simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker objects representing cylinders
///     ~/walls (visualization_msgs::msg::MarkerArray): marker objects representing walls of arena
///     ~/red/path (nav_msgs::msg::Path): path of the turtlebot
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets the simulation to the initial state
///     ~/teleport (nusim::srv::Teleport): teleports the turtle to a given x, y, theta value
/// CLIENTS:
///     none
/// BROADCASTS:
///    nusim/world -> red/base_footprint
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/lidar.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <random>

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    ///create a timer
    declare_parameter("rate", 200.);
    rate_hz = get_parameter("rate").as_double();
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

    declare_parameter("rate_fakesensor", 5);
    rate_fakesensor_hz = get_parameter("rate_fakesensor").as_int();
    std::chrono::milliseconds rate_fakesensor = (std::chrono::milliseconds) ((int)(1000. / rate_fakesensor_hz));

    declare_parameter("x", 0.);
    x_i = get_parameter("x").as_double();

    declare_parameter("y", 0.);
    y_i = get_parameter("y").as_double();

    declare_parameter("theta", 0.);
    theta_i = get_parameter("theta").as_double();

    declare_parameter("arena_x_length", 0.);
    arena_x_length = get_parameter("arena_x_length").as_double();

    declare_parameter("arena_y_length", 0.);
    arena_y_length = get_parameter("arena_y_length").as_double();

    declare_parameter("obstacles/x", std::vector<double>{});
    obx = get_parameter("obstacles/x").as_double_array();

    declare_parameter("obstacles/y", std::vector<double>{});
    oby = get_parameter("obstacles/y").as_double_array();

    declare_parameter("obstacles/r", 0.0);
    obr = get_parameter("obstacles/r").as_double();

    declare_parameter("collision_radius", 0.0);
    collision_radius = get_parameter("collision_radius").as_double();

    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();

    declare_parameter("wheel_radius", -1.);
    wheel_radius = get_parameter("wheel_radius").as_double();
    if(wheel_radius == 0 || wheel_radius < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Wheel_radius error" << 4);
    }

    declare_parameter("track_width", -1.);
    track_width = get_parameter("track_width").as_double();
    if(track_width == 0 || track_width < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Track_width error" << 4);
    }

    declare_parameter("encoder_ticks_per_rad", -1.);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    if(encoder_ticks_per_rad == 0 || encoder_ticks_per_rad < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "encoder_ticks_per_rad error" << 4);
    }
    ///LIDAR PARAMETERS
    // lidar_angle_increment_
    declare_parameter("lidar_angle_increment_", 0.0174533);
    lidar_angle_increment_ = get_parameter("lidar_angle_increment_").as_double();

    // lidar_max_detect_range QUESTION: CHANGE RANGE PARAMETER LATER!!!!!!!
    declare_parameter("lidar_max_range_", 3.0);
    lidar_max_range_ = get_parameter("lidar_max_range_").as_double();

    declare_parameter("lidar_min_range_", 0.12);
    lidar_min_range_ = get_parameter("lidar_min_range_").as_double();
    ///QUESTION HOW WE GONNA USE THIS PARAMETER??? 5 scans per second
    declare_parameter("lidar_num_samples_", 0);
    lidar_num_samples_ = get_parameter("lidar_num_samples_").as_int();
    ///QUESTION resolution corresponding to angle increament???
    declare_parameter("resolution", 0.0174533);
    resolution = get_parameter("resolution").as_double();

    declare_parameter("noise_level", 0.0);
    noise_level = get_parameter("noise_level").as_double();




    declare_parameter("draw_only", false);
    draw_only = get_parameter("draw_only").as_bool();
    ///gausion noise with variance input_noise
    declare_parameter("input_noise", 0.0);
    input_noise = get_parameter("input_noise").as_double();
    ///slip fraction
    declare_parameter("slip_fraction", 0.0);
    slip_fraction = get_parameter("slip_fraction").as_double();
    ///gausion noise for sensor_variance
    declare_parameter("basic_sensor_variance", 0.01);
    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();

    



    initial = true;
    x0 = 0.0;
    y0 = 0.0;
    theta0 = 0.0;
    /// timer for normal publishing wall stuff
    timer_ = create_wall_timer(
      rate, std::bind(&Nusim::timer_callback, this));
    /// timer for fake sensor publishing
    timer2_ = create_wall_timer(
      rate_fakesensor, std::bind(&Nusim::timer_callback_slow, this));

    //timestep publisher
    time_step_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    //service reset
    reset_ =
      create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));

    //broadcaster the transform
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //teleport service
    teleport_ =
      create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));

    wheel_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::wheel_callback, this, std::placeholders::_1));
// sensor data publisher
    sensor_data_pub = this->create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
      
    path_pub = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    
    
    //wall publisher
    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    wall_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_policy);
    obs_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos_policy);
    fake_sensor_cyl_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", qos_policy);
    
    //laser publisher
    laser_pub = create_publisher<sensor_msgs::msg::LaserScan>("~/scan", 10);
  }

private:
  void timer_callback()
  {
    // publish timetsep
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    time_step_publisher_->publish(message);
    timestep_++;
    

    wallPub();
    obstacle_publisher();

    if(!draw_only){
      //if its the first time calling initial, set the initial location
      if (initial) {
        x0 = x_i;
        y0 = y_i;
        theta0 = theta_i;
        initial = false;
      }
      ///update the robot configuration
      auto delta_left_angle = left_velocity/rate_hz;///velocity(rad/s)*time to get rad
      auto delta_right_angle = right_velocity/rate_hz;
      ///QUESTIONS?? WHY LEFT ARE USING U*(1+slip_fraction)???
      delta_left_angle = delta_left_angle * (1 + slip_noise);
      delta_right_angle = delta_right_angle * (1 + slip_noise);

      auto new_right_angle = delta_right_angle + robot.get_wheel_state().r;
      auto new_left_angle = delta_left_angle + robot.get_wheel_state().l;

      ///use delta wheel degree to update wheel state
      robot.forwardKinematics(turtlelib::WheelState{delta_left_angle, delta_right_angle});
      // x = robot.get_current_config().translation().x;
      // y = robot.get_current_config().translation().y;
      // theta = robot.get_current_config().rotation();
      // send_transform(x, y, theta);
      ///if no collision, update the robot's location
      // if(!collide){
      //   ///get the robot's new location, 
      //   x = robot.get_current_config().translation().x;
      //   y = robot.get_current_config().translation().y;
      //   theta = robot.get_current_config().rotation();
      //   send_transform(x, y, theta);
      // }
      // else{
      //   send_transform(x, y, theta);
      // }
      collision();
      x = robot.get_current_config().translation().x;
      y = robot.get_current_config().translation().y;
      theta = robot.get_current_config().rotation();
      send_transform(x, y, theta);
      //update the sensor call back
      sensor_data.left_encoder = static_cast<int>(new_left_angle*encoder_ticks_per_rad);
      sensor_data.right_encoder = static_cast<int>(new_right_angle*encoder_ticks_per_rad);
      sensor_data.stamp = get_clock()->now();
      sensor_data_pub->publish(sensor_data);

      //publish the nav_msgs::msg::Path
      red_path.header.frame_id = "nusim/world";
      red_path.header.stamp = get_clock()->now();
      path_pub->publish(red_path);
    }
  }

  void timer_callback_slow()
  {
    //publish the sensor data on fake obstacles
    if(!draw_only){
      fake_sensor_cyl_publisher();
      laser_scan();
      // RCLCPP_INFO_STREAM(get_logger(), "Fake sensor publishing");
    }


  }
  void reset(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RESET IS GOING ON!");
    timestep_ = 0;
    // RCLCPP_INFO_STREAM(get_logger(), "reset x=" << x << " y=" << y << " theta=" << theta);
    x = x_i;
    y = y_i;
    theta = theta_i;
  }

  //broadcast the location by giving x y w
  //also update the robot path with x y w
  void send_transform(double x, double y, double w)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, w);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    //update the robot path
    robot_pose.header.frame_id = "nusim/world";
    robot_pose.header.stamp = get_clock()->now();
    robot_pose.pose.position.x = x;
    robot_pose.pose.position.y = y;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = q.x();
    robot_pose.pose.orientation.y = q.y();
    robot_pose.pose.orientation.z = q.z();
    robot_pose.pose.orientation.w = q.w();
    // RCLCPP_INFO_STREAM(
      // get_logger(), "Send path pose as x=" << x << " y=" << y << " theta=" << w);
    red_path.poses.push_back(robot_pose);

  }

  void teleport(
    std::shared_ptr<nusim::srv::Teleport::Request> req,
    std::shared_ptr<nusim::srv::Teleport::Response> res)
  {
    x = req->x;
    y = req->y;
    theta = req->theta;
    RCLCPP_INFO_STREAM(
      get_logger(), "Teleporting service as x=" << x << " y=" << y << " theta=" << theta0);
    res->success = true;
  }

  void wallPub()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;

    for (int i = 0; i < 4; i++) {
      m.header.stamp = this->get_clock()->now();
      m.header.frame_id = "nusim/world";
      m.id = i;
      m.type = 1;
      m.action = 0;

      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      //wall scale
      m.scale.x = 0.0;
      m.scale.y = 0.0;
      m.scale.z = 0.25;
      m.pose.position.x = 0.0;
      m.pose.position.y = 0.0;
      m.pose.position.z = 0.125;
      arr.markers.push_back(m);
    }
    const auto wall_thickness = 0.1;

    arr.markers.at(0).scale.x = wall_thickness;
    arr.markers.at(0).scale.y = arena_y_length + 2 * wall_thickness;
    arr.markers.at(0).pose.position.x = 0.5 * (arena_x_length + wall_thickness);

    arr.markers.at(1).scale.x = arena_x_length + 2 * wall_thickness;
    arr.markers.at(1).scale.y = wall_thickness;
    arr.markers.at(1).pose.position.y = 0.5 * (arena_y_length + wall_thickness);

    arr.markers.at(2).scale.x = wall_thickness;
    arr.markers.at(2).scale.y = arena_y_length + 2 * wall_thickness;
    arr.markers.at(2).pose.position.x = -0.5 * (arena_x_length + wall_thickness);

    arr.markers.at(3).scale.x = arena_x_length + 2 * wall_thickness;
    arr.markers.at(3).scale.y = wall_thickness;
    arr.markers.at(3).pose.position.y = -0.5 * (arena_y_length + wall_thickness);

    wall_pub->publish(arr);
  }

  void obstacle_publisher()
  {
    if (oby.size() == obx.size()) {
      RCLCPP_INFO_ONCE(get_logger(), "Valid Marker Input!");
      cyl_num = oby.size();
    } else {
      RCLCPP_DEBUG(get_logger(), "Invalid Marker Input!");
      cyl_num = 0;
      return;
    }
    visualization_msgs::msg::MarkerArray arr_obstacle;
    for (size_t i = 0; i < cyl_num; i++) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = this->get_clock()->now();
      m.header.frame_id = "nusim/world";
      m.id = i;
      m.type = 3;
      m.action = 0;

      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      //wall scale
      m.scale.x = this->obr * 2;
      m.scale.y = this->obr * 2;
      m.scale.z = 0.25;
      m.pose.position.x = this->obx[i];
      m.pose.position.y = this->oby[i];
      m.pose.position.z = 0.125;
      arr_obstacle.markers.push_back(m);
    }
    obs_pub->publish(arr_obstacle);
  }

  void fake_sensor_cyl_publisher(){
    if (oby.size() == obx.size()) {
      RCLCPP_INFO_ONCE(get_logger(), "Valid Marker Input!");
      cyl_num = oby.size();
    } else {
      RCLCPP_DEBUG(get_logger(), "Invalid Marker Input!");
      cyl_num = 0;
      return;
    }
    visualization_msgs::msg::MarkerArray arr_fake_sensor_obstacle;
    // RCLCPP_INFO_STREAM(get_logger(), "I get into fake sensor publisher");
    for (size_t i = 0; i < cyl_num; i++) {
      
      // RCLCPP_INFO_STREAM(get_logger(), "FOR LOOP TIME" << i);
      visualization_msgs::msg::Marker m;
      m.header.stamp = this->get_clock()->now();
      m.header.frame_id = "red/base_footprint";
      m.id = i;
      if(dist_obs_robot(i, range)){
        m.type = 3;
        m.action = 0;

        ///create gaussian noise on the obstacle location
        std::normal_distribution<double> d(0, basic_sensor_variance);
        auto obs_noise = d(get_random());
        // RCLCPP_INFO_STREAM(get_logger(), "obs_noise" << obs_noise);

        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
        //wall scale
        m.scale.x = this->obr * 2 + obs_noise;
        m.scale.y = this->obr * 2 + obs_noise;
        m.scale.z = 0.25;
        const turtlelib::Transform2D column_location{turtlelib::Vector2D {this->obx[i], this->oby[i]}, 0};
        // RCLCPP_INFO_STREAM(get_logger(), "COLUMN LOCATION TRC as" << column_location.translation().x << " " << column_location.translation().y << " " << column_location.rotation());
        // const auto robot_cur_location = robot.get_current_config(); ///also a Transform2D
        const turtlelib::Transform2D robot_cur_location{turtlelib::Vector2D{x, y}, theta};  /// x, y, theta represents the red robot's current location
        // RCLCPP_INFO_STREAM(get_logger(), "ROBOT CUR LOCATION TRC as" << robot_cur_location.translation().x << " " << robot_cur_location.translation().y << " " << robot_cur_location.rotation());
        ///return a vector of the translation, Trc = Trs * Tsc = Tsr.inv() * Tsc
        const auto Trc_location = (robot_cur_location.inv() * column_location).translation();
        m.pose.position.x = Trc_location.x + obs_noise;
        m.pose.position.y = Trc_location.y + obs_noise;
        m.pose.position.z = 0.125;
        // RCLCPP_INFO_STREAM(get_logger(), "new obstacle added");

        ///updating if the robot is colliding the obstacle
        // if(if_collide(i)){
        //   collide = true;
        // }
      }
      else{
        m.action = visualization_msgs::msg::Marker::DELETE;
        
        // RCLCPP_INFO_STREAM(get_logger(), "delete the obstacle marker!");
        }
      arr_fake_sensor_obstacle.markers.push_back(m);
    }
    fake_sensor_cyl_pub->publish(arr_fake_sensor_obstacle);
  
  }

  void wheel_callback(const nuturtlebot_msgs::msg::WheelCommands & wc)
  {
    // just store left and right velocity and do this update in the timer
    left_velocity = wc.left_velocity * motor_cmd_per_rad_sec;   //wc in ticks, get the velocity in rad/s
    right_velocity = wc.right_velocity * motor_cmd_per_rad_sec;
    if(left_velocity != 0){
      left_velocity += gaussian_noise_velocity;
    }
    if(right_velocity != 0){
      right_velocity += gaussian_noise_velocity;
    }
    // RCLCPP_INFO_STREAM(
    //   get_logger(), "Teleporting service as x=" << left_velocity << " y=" << right_velocity);
  }

  std::mt19937 & get_random()
  {
      // static variables inside a function are created once and persist for the remainder of the program
      static std::random_device rd{}; 
      static std::mt19937 mt{rd()};
      // we return a reference to the pseudo-random number genrator object. This is always the
      // same object every time get_random is called
      return mt;
  }
  void generate_noise(){
    //generate the noise
    std::normal_distribution<double> d(0, input_noise);
    gaussian_noise_velocity = d(get_random());
    std::uniform_real_distribution<> u(-slip_fraction, slip_fraction);
    slip_noise = u(get_random());
  }

  /// @brief  return true if the robot is collidig the obstacle, false if not
  /// @param i the index of the obstacle
  /// @return bool
  bool if_collide(int i){
    double cyl_x = obx[i];
    double cyl_y = oby[i];
    auto robot_x = robot.get_current_config().translation().x;
    auto robot_y = robot.get_current_config().translation().y;
    auto dist = std::sqrt(std::pow(cyl_x - robot_x, 2) + std::pow(cyl_y - robot_y, 2));
    if((dist - obr - collision_radius) > 0){
      return false;
    }
    else{
      return true;
    }
  }
  /// @brief  update robot position once hit column
  /// @return nothing
  void collision()
    {
      for (size_t i = 0; i < obx.size(); ++i) {
        auto rs = robot.get_current_config();

        double dx = obx[i] - rs.translation().x;
        double dy = oby[i] - rs.translation().y;
        double d = std::sqrt(dx * dx + dy * dy);

        double total_radius = collision_radius + obr;

        if (d < total_radius) {
          RCLCPP_INFO_STREAM(get_logger(), "Collision detected with obstacle " << i);

          double vx = dx / d;
          double vy = dy / d;

          double tx = obx[i] - total_radius * vx;
          double ty = oby[i] - total_radius * vy;

          robot.q.linear.x = tx;
          robot.q.linear.y = ty;
          robot.q.angular = rs.rotation();
        }
      }
    }


  bool detect_and_simulate_collision(turtlelib::WheelState predicted_delta_wheels_)
  {    
    // Predicted robot motion
    turtlelib::DiffDrive predicted_turtle_ = robot;
    predicted_turtle_.forwardKinematics(predicted_delta_wheels_);   

    turtlelib::Transform2D T_world_robot_{{predicted_turtle_.get_current_config().translation().x, predicted_turtle_.get_current_config().translation().y}, predicted_turtle_.get_current_config().rotation()};
    turtlelib::Transform2D T_robot_world_ = T_world_robot_.inv(); 

    for (size_t i = 0; i < obx.size(); i++)
    {
      // Find local coordinates of obstacle
      turtlelib::Point2D obstacle_pos_world_{obx.at(i), oby.at(i)};
      turtlelib::Point2D obstacle_pos_robot_ = T_robot_world_(obstacle_pos_world_);

      // Detect collision
      if (std::sqrt(std::pow(obstacle_pos_robot_.x, 2) + std::pow(obstacle_pos_robot_.y, 2)) < (collision_radius + obr)) 
      {
        // If colliding, calculate shift of robot frame, in robot frame
        turtlelib::Vector2D robotshift_robot_{
        -((collision_radius + obr) * cos(atan2(obstacle_pos_robot_.y, obstacle_pos_robot_.x)) - obstacle_pos_robot_.x),
        -((collision_radius + obr) * sin(atan2(obstacle_pos_robot_.y, obstacle_pos_robot_.x)) - obstacle_pos_robot_.y)
        };

        // Calculate transform corresponding to this shift
        turtlelib::Transform2D T_robot_newrobot_{robotshift_robot_};

        // Define this transformation in the world frame
        turtlelib::Transform2D T_world_newrobot_ = T_world_robot_ * T_robot_newrobot_;

        if(lie_group_collision_)
        {
          x = T_world_newrobot_.translation().x;
          y = T_world_newrobot_.translation().y;
          theta = T_world_newrobot_.rotation();
        }
          
        robot.w.l = turtlelib::normalize_angle(robot.get_wheel_state().l + predicted_delta_wheels_.l); // TODO: wheel rotation not working properly
        robot.w.r = turtlelib::normalize_angle(robot.get_wheel_state().l + predicted_delta_wheels_.r);

        RCLCPP_DEBUG(this->get_logger(), "turtle: %f B: %f", obstacle_pos_robot_.x, obstacle_pos_robot_.y);
        return true; // Colliding with one obstacle, therefore, ignore other obstacles
      } 
    }
    return false; // Not colliding

    throw std::runtime_error("Invalid collision! Check collision simulation!");
  }
  ///return true if the obstacle is inside the range of the fake sensor, false if out of range
  bool dist_obs_robot(int i, double range){
    double cyl_x = obx[i];
    double cyl_y = oby[i];
    auto robot_x = robot.get_current_config().translation().x;
    auto robot_y = robot.get_current_config().translation().y;
    auto dist = std::sqrt(std::pow(cyl_x - robot_x, 2) + std::pow(cyl_y - robot_y, 2));
    if((range == 0) && ((dist - obr - collision_radius) < range)){
      // RCLCPP_INFO_STREAM(get_logger(), "return true, situation 1, the distance is inside range" << (dist - obr - collision_radius));
      return true;
    }
    else if((dist - obr - collision_radius) < range){
      // RCLCPP_INFO_STREAM(get_logger(), "return true, the distance is inside range" << (dist - obr - collision_radius));
      return true;
    }
    // RCLCPP_INFO_STREAM(get_logger(), "return false" << (dist - obr - collision_radius));
    return false;
    
  }

  void laser_scan(){
    ///calculate the lidar num samples
    lidar_num_samples_ = static_cast<int> (turtlelib::deg2rad(360 - 0)/lidar_angle_increment_+1);///360
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.frame_id = "red/base_scan";
    scan_msg.header.stamp = get_clock()->now();
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = turtlelib::deg2rad(360.0); // convert degrees to radians
    scan_msg.angle_increment = lidar_angle_increment_; // in radians
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 1.0 / rate_fakesensor_hz;
    scan_msg.range_min = lidar_min_range_;
    scan_msg.range_max = lidar_max_range_;
    scan_msg.ranges.resize(lidar_num_samples_);


    for (int i = 0; i < lidar_num_samples_; i++) {
      ///Solution 1: in robot frame
      // const turtlelib::Transform2D column_location{turtlelib::Vector2D {this->obx[0], this->oby[0]}, 0};
      // const turtlelib::Transform2D robot_cur_location{turtlelib::Vector2D{x, y}, theta};  /// x, y, theta represents the red robot's current location
      // const auto Trc_location = (robot_cur_location.inv() * column_location).translation();
      // turtlelib::Point2D obst_location{Trc_location.x, Trc_location.y};///remember to change!!!
      // turtlelib::Point2D robot_location{0, 0};
      // double angle = -(i * lidar_angle_increment_ - turtlelib::PI/2); 
      bool wall_measured = false;
      ///Solution 2: in world frame
    ///Problem with lidar, it initially predicts but with the rotation, the lidar goes off
    for(size_t column_index = 0; column_index < cyl_num; column_index++){
      turtlelib::Point2D obst_location{obx[column_index], oby[column_index]};///remember to change!!!
      turtlelib::Point2D robot_location{x, y}; ///x, y represents the red robot's current location
      ///the way I wrote my function is not follow convention, so convert them into the real angle 
      double angle = -(i * lidar_angle_increment_ + theta - turtlelib::PI/2); 
      turtlelib::Point2D scan_point 
      = turtlelib::intersectPoint(obst_location, robot_location, lidar_max_range_, angle, obr);


      turtlelib::Transform2D lidar_pose_{turtlelib::Vector2D{
                (robot.get_current_config().translation().x - 0.032*cos(robot.get_current_config().angular)), 
                (robot.get_current_config().translation().y - 0.032*sin(robot.get_current_config().angular))},
                robot.get_current_config().angular};
      turtlelib::Point2D limit{
                            lidar_pose_.linear.x + lidar_max_range_ * cos(i * lidar_angle_increment_ + lidar_pose_.angular),
                            lidar_pose_.linear.y + lidar_max_range_ * sin(i * lidar_angle_increment_ + lidar_pose_.angular)
                          };
      double lidar_reading = 3.5;
      double slope = (limit.y - lidar_pose_.linear.y) / (limit.x - lidar_pose_.linear.x  + 1e-7);


      if (scan_point.x == 10000 && !wall_measured)
      {
        // Estimate where laser hits the wall, if not already estimated
        
          // North Wall reading
          if(limit.y > arena_y_length/2.0)
          {
            turtlelib::Vector2D laser_vector{ 0, arena_y_length/2.0 - lidar_pose_.linear.y};
            laser_vector.x = laser_vector.y / (slope  + 1e-7);

            if(lidar_reading > turtlelib::magnitude(laser_vector))
            {
              lidar_reading = turtlelib::magnitude(laser_vector);
            }
          }
          // West Wall reading
          if(limit.x < -arena_x_length/2.0)
          {
            turtlelib::Vector2D laser_vector{ -arena_x_length/2.0 - lidar_pose_.linear.x, 0};
            laser_vector.y = laser_vector.x * slope;

            if(lidar_reading > turtlelib::magnitude(laser_vector))
            {
              lidar_reading = turtlelib::magnitude(laser_vector);
            }
          }
          // South Wall reading
          if(limit.y < -arena_y_length/2.0)
          {
            turtlelib::Vector2D laser_vector{0, -arena_y_length/2.0 - lidar_pose_.linear.y};
            laser_vector.x = laser_vector.y / (slope  + 1e-7);

            if(lidar_reading > turtlelib::magnitude(laser_vector))
            {
              lidar_reading = turtlelib::magnitude(laser_vector);
            }
          }
          // East Wall reading
          if(limit.x > arena_x_length/2.0)
          {
            turtlelib::Vector2D laser_vector{ arena_x_length/2.0 - lidar_pose_.linear.x, 0};
            laser_vector.y = laser_vector.x * slope;

            if(lidar_reading > turtlelib::magnitude(laser_vector))
            {
              lidar_reading = turtlelib::magnitude(laser_vector);
            }
          }
          scan_msg.ranges.at(i) = lidar_reading;
          wall_measured = true;
      }

      if(scan_point.x < 10000 && scan_point.y < 10000){
        scan_msg.ranges.at(i) = turtlelib::magnitude(scan_point - robot_location);
        break;
      }
    }
  }
    
    laser_pub->publish(scan_msg);
  }

  bool lie_group_collision_ = true;
  double x_i, y_i, theta_i;
  double x, y, theta, rate_hz, encoder_ticks_per_rad, rate_fakesensor_hz;
  double arena_x_length, arena_y_length;
  double track_width, wheel_radius, motor_cmd_per_rad_sec;
  std::vector<double> obx, oby;
  double obr, collision_radius, basic_sensor_variance;
  bool initial;
  double x0, y0, theta0;
  size_t timestep_;
  size_t cyl_num;
  double left_velocity=0.0; //rad/s
  double right_velocity=0.0;
  nav_msgs::msg::Path red_path; //the path of the robot
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_cyl_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_subscriber_;
  turtlelib::DiffDrive robot{0.033, 0.16};
  nuturtlebot_msgs::msg::SensorData sensor_data;
  geometry_msgs::msg::PoseStamped robot_pose = geometry_msgs::msg::PoseStamped();
  bool draw_only = false;
  double input_noise = 0.0;
  double slip_fraction = 0.0;
  double gaussian_noise_velocity = 0.0;
  double slip_noise = 0.0;
  double range = 3.0; //maximum range of the fake sensor to detect the obstacle
  // bool collide = false;
  double lidar_angle_increment_, lidar_max_range_, lidar_min_range_, resolution, noise_level;
  int lidar_num_samples_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
