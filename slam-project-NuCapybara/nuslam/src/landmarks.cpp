/// \file odometry.cpp
/// \brief Publish odom and tf messages of the robot by subscribing to joint states
///
/// PARAMETERS:
///    \param body_id (string): body id of the robot
///    \param odom_id (string): odometry id of the robot
///    \param rate (double): rate at which the control loop runs
///    \param velocity (double): angular velocity of the turtle
///    \param radius (double): radius of the circle
///    \param wheel_radius (double): radius of the wheel
///    \param track_width (double): width of the track

/// PUBLISHES:
///     \param odom (nav_msgs::msg::Odometry): odometry of the turtle
///     \param TransformBroadcaster (tf2_ros::TransformBroadcaster): broadcast transform of the robot
/// SERVICES:
///     \param initial_pose (nuturtle_control::srv::InitialPose): sets the initial pose of the turtle
/// SUBSCRIBES:
///     \param joint_states (sensor_msgs::msg::JointState): joint states of the turtle
/// CLIENTS:
///     none
/// BROADCASTS:
///   \param braodcaster odom -> blue/base_footprint


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf_slam.hpp"
#include "turtlelib/geometry2d.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;


// ############################ Begin_Citation [1] ############################
// Code that is directly influence by the citation [1]
// ############################ End_Citation [1]  #############################


class Landmarks: public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    laser_scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Landmarks::laser_scan_callback, this, std::placeholders::_1));
    detect_landmark_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", 10);
   
  }
private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan & laser_msg)
    {
     RCLCPP_INFO_STREAM(get_logger(), "heard laser msg" << msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;


};

/// \brief main function to create and run Odometry node
/// \param argc
/// \param argv
/// \return int
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
