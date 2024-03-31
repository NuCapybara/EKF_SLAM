#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"


using namespace std::chrono_literals;


turtlelib::DiffDrive robot{0.033, 0.16};
/// Twist message for testing cmd_callback
geometry_msgs::msg::Twist cmd;
/// SensorData message for testing sensor_data_callback
nuturtlebot_msgs::msg::SensorData sensor_msg;
void test_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands msg) {
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I heard Wheel Command as (WC: left velocity, right velocity): " << msg.left_velocity << " " << msg.right_velocity);
  turtlelib::Twist2D twist{cmd.angular.z, cmd.linear.x, 0.0};
  turtlelib::WheelState ws = robot.inverseKinematics(twist);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "!!!wl, wr calcualted "<< ws.l << " " << ws.r);
  double left_velocity_calculated = int(ws.l / 0.024); ///0.024 as ticks per radian
  double right_velocity_calculated = int(ws.r / 0.024); //wheel command is in int so we need to convert it to int
  

  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "left velocity calculated, right velocity calculated): " << left_velocity_calculated << " " << right_velocity_calculated);

  REQUIRE_THAT(right_velocity_calculated, Catch::Matchers::WithinAbs(msg.right_velocity, 1e-1));
  REQUIRE_THAT(left_velocity_calculated, Catch::Matchers::WithinAbs(msg.left_velocity, 1e-1));
}

void test_joint_state_callback(const sensor_msgs::msg::JointState msg) {
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I heard Joint State as (JS: left position, right position): " << msg.position.at(0) << " " << msg.position.at(1));
  double left_wheel_position = msg.position.at(0);
  double right_wheel_position = msg.position.at(1);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "left position, right position: " << left_wheel_position << " " <<  right_wheel_position);
  REQUIRE_THAT(left_wheel_position, Catch::Matchers::WithinAbs(0.15339, 1e-1));
  REQUIRE_THAT(right_wheel_position, Catch::Matchers::WithinAbs(0.092, 1e-1));
}


TEST_CASE("cmd_vel_test_pure_translation", "turtle_control_test") {
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  /// cmd_vel publisher
  auto cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  /// wheel_cmd_subscriber
  auto wheel_cmd_subscriber_ = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, test_cmd_callback);
  ///test duration parameter declaration
  node->declare_parameter<double>("test_duration", 2.0);
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  rclcpp::Time start_time = rclcpp::Clock().now();

  cmd.linear.x = 0.1;
  cmd.angular.z = 0;
  cmd_vel_publisher_->publish(cmd);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I published cmd_vel");

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    rclcpp::spin_some(node);
  }
}


TEST_CASE("cmd_vel test pure rotation", "turtle_control_test") {
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  /// cmd_vel publisher
  /// wheel_cmd_subscriber
  auto cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_subscriber_ = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, test_cmd_callback);
  
  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration", 2.0);

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  rclcpp::Time start_time = rclcpp::Clock().now();

  cmd.linear.x = 0.;
  cmd.angular.z = 1.0;
  cmd_vel_publisher_->publish(cmd);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I published cmd_vel");


  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    rclcpp::spin_some(node);
  }
}



TEST_CASE("sensor_data_conversion_on_joint_stats", "turtle_control_test") {
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  sensor_msg.left_encoder = 100.0;
  sensor_msg.right_encoder = 60.0;
  /// sensor msg publisher
  auto sensor_msg_publisher_ = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
  ///joint state subscriber
  auto joint_state_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, test_joint_state_callback);



  // ///compute joint state by sensor data
  // auto left_encoder = sensor_msg.left_encoder;
  // auto right_encoder = sensor_msg.right_encoder;

  // auto radians_left = left_encoder/651.8986;
  // auto radians_right = right_encoder/651.8986;

  
  node->declare_parameter<double>("test_duration", 2.0);
  const auto TEST_DURATION =
  node->get_parameter("test_duration").get_parameter_value().get<double>();

  rclcpp::Time start_time = rclcpp::Clock().now();

  sensor_msg_publisher_->publish(sensor_msg);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I published sensor_msg!" << sensor_msg.left_encoder << " " << sensor_msg.right_encoder);


  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    rclcpp::spin_some(node);
  }
}