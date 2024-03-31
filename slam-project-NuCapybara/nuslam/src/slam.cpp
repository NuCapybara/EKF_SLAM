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

/// \brief subscribes to joint states and publishes odometry of the robot
/// \param body_id - body id of the robot
/// \param odom_id - odometry id of the robot
/// \param wheel_left - left wheel joint name
/// \param wheel_right - right wheel joint name
/// \param wheel_radius - radius of the wheels[m]
/// \param track_width - distance between the wheels[m]


// ############################ Begin_Citation [1] ############################
// Code that is directly influence by the citation [1]
// ############################ End_Citation [1]  #############################


class Nuslam: public rclcpp::Node
{
public:
  Nuslam()
  : Node("nuslam")
  {


    ///parameter delcaration
    declare_parameter("body_id", "green/base_footprint");
    body_id = get_parameter("body_id").as_string();
    if(body_id.empty()){
        RCLCPP_DEBUG_STREAM(get_logger(), "BodyID not specified" << 4);
    }
    
    declare_parameter("odom_id", odom_id);
    odom_id = get_parameter("odom_id").as_string();

    // declare_parameter("wheel_left", "green/wheel_left_link");
    declare_parameter("wheel_left", wheel_left);
    wheel_left = get_parameter("wheel_left").as_string();
    if(wheel_left.empty()){
        RCLCPP_DEBUG_STREAM(get_logger(), "left wheel joint name not specified" << 4);
    }

    // declare_parameter("wheel_right", "green/wheel_right_link");
    declare_parameter("wheel_right", wheel_right);
    wheel_right = get_parameter("wheel_right").as_string();
    if(wheel_right.empty()){
        RCLCPP_DEBUG_STREAM(get_logger(), "right wheel joint name not specified" << 4);
    }

    declare_parameter("wheel_radius", -1.);
    wheel_radius = get_parameter("wheel_radius").as_double();
    if(wheel_radius == -1.0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Wheel_radius error" << 4);
    }

    declare_parameter("track_width", -1.);
    track_width = get_parameter("track_width").as_double();
    if(track_width == -1.0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Track_width error" << 4);
    }
    
    declare_parameter("basic_sensor_variance", -1.);
    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();
    if(basic_sensor_variance == -1.0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Basic sensor variance error" << 4);
    }

    declare_parameter("obstacles/r", -1.);
    obstacles_r = get_parameter("obstacles/r").as_double();
    if(obstacles_r == -1.0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Obstacle r error" << 4);
    }

    declare_parameter("max_range", -1.);
    max_range = get_parameter("max_range").as_double();
    if(max_range == -1.0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Max range error" << 4);
    }

    //odom id defination
    odom.header.frame_id = odom_id; //relatively world id?
    odom.child_frame_id = body_id;
    transformStamped.header.frame_id = odom_id;
    transformStamped.child_frame_id = body_id;  
    robot = turtlelib::DiffDrive(wheel_radius, track_width);
    ekf = std::make_unique<turtlelib::EKFSlam>(robot.get_current_config());
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Nuslam::js_callback, this, std::placeholders::_1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("green/odom", 10);
    path_pub_green = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    fake_sensor_subscriber_ =  create_subscription<visualization_msgs::msg::MarkerArray>(
    "nusim/fake_sensor", 10, std::bind(
        &Nuslam::fake_sensor_callback,
        this, std::placeholders::_1));
    /// odom to base_footprint
    br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    /// odom to map
    tf_br2 = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ///
    initial_pose_=create_service<nuturtle_control::srv::InitialPose>("initial_pose",
    std::bind(&Nuslam::initial_pose, this, std::placeholders::_1, std::placeholders::_2));
  }

private:

    void js_callback(const sensor_msgs::msg::JointState & js){
        // RCLCPP_INFO_STREAM(get_logger(), "getinto js");

        std::string js_frameid = js.header.frame_id;
        // const auto time_stamp = js.header.stamp;
        const auto joint_name_list = js.name;
        const auto joint_position_list = js.position;
        const auto joint_velocity_list = js.velocity;
        

        if(first){
            first = false;

        }
        else{
            auto left_index = -1;
            auto right_index = -1;
            for (long unsigned int i = 0; i < js.name.size(); i++){
                if (js.name[i] == wheel_left){
                    left_index = i;
                }
                else if (js.name[i] == wheel_right){
                    right_index = i;
                }
            }

            if (left_index != -1 && right_index != -1)  
            {
                // RCLCPP_INFO_STREAM(get_logger(), "left index" << left_index);
                // RCLCPP_INFO_STREAM(get_logger(), "wheel right:" << right_index);
                const auto dl = joint_position_list.at(left_index) - last_joint_state.position.at(left_index);
                const auto dr = joint_position_list.at(right_index) - last_joint_state.position.at(right_index);
                const auto currTime = js.header.stamp.sec  + 1e-9 * js.header.stamp.nanosec;
                const auto lastTime = last_joint_state.header.stamp.sec + 1e-9 * last_joint_state.header.stamp.nanosec;
                const auto dt = currTime - lastTime;

                const auto prev_x = robot.get_current_config().translation().x;
                const auto prev_y = robot.get_current_config().translation().y;
                const auto prev_phi = robot.get_current_config().rotation();

                // RCLCPP_INFO_STREAM(get_logger(), "dl" << dl << "dr" << dr);

                // update robot config and everything
                robot.forwardKinematics(turtlelib::WheelState{dl, dr});

                ///ODOM POSE PART
                ///define odom pose/pose
                odom.header.stamp = get_clock()->now();
                odom.pose.pose.position.x = robot.get_current_config().translation().x;
                odom.pose.pose.position.y = robot.get_current_config().translation().y;
                ///define odom pose/orientation(quaternion)
                tf2::Quaternion q;
                q.setRPY(0, 0, robot.get_current_config().rotation());
                odom.pose.pose.orientation.x = q.x();
                odom.pose.pose.orientation.y = q.y();
                odom.pose.pose.orientation.z = q.z();
                odom.pose.pose.orientation.w = q.w();
            
                ///ODOM VELOCITY PART
                ///define odom twist/twist
                odom.twist.twist.linear.x = (robot.get_current_config().translation().x - prev_x) / dt;
                odom.twist.twist.linear.y = (robot.get_current_config().translation().y - prev_y) / dt;
                odom.twist.twist.angular.z = turtlelib::normalize_angle((robot.get_current_config().rotation() - prev_phi) / dt);



                ///publish odom
                odom_publisher_->publish(odom);
               
               ///TF PART
                transformStamped.header.stamp = get_clock()->now();
                transformStamped.transform.translation.x = robot.get_current_config().translation().x;

                transformStamped.transform.translation.y = robot.get_current_config().translation().y;
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                br->sendTransform(transformStamped);

                ///Path publishing on blue robot
                robot_path.header.stamp = get_clock()->now();
                robot_path.header.frame_id = "map";
                ///initialize the pose inside of the path
                geometry_msgs::msg::PoseStamped path_pose;
                path_pose.header.stamp = get_clock()->now();
                path_pose.header.frame_id = "map";
                path_pose.pose.position.x = robot.get_current_config().translation().x;
                path_pose.pose.position.y = robot.get_current_config().translation().y;
                path_pose.pose.orientation.x = q.x();
                path_pose.pose.orientation.y = q.y();
                path_pose.pose.orientation.z = q.z();
                path_pose.pose.orientation.w = q.w();
                robot_path.poses.push_back(path_pose);
                path_pub_green->publish(robot_path);

                broadcast_map_odom_transform();
                publish_seen_obstacle();


            }
            else{
                ///JOINT STATE ERROR
                RCLCPP_DEBUG_STREAM(get_logger(), "Wheel ID not found" << 4);
            } 
        }
        last_joint_state = js;

    }

    void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
    {
        // RCLCPP_INFO_STREAM(get_logger(), "getinto fake sensor");
        T_or_now= robot.get_current_config();
        turtlelib::Transform2D T_rr_prime = T_or_prev.inv() * T_or_now;
        ekf->predict(turtlelib::differentiate_transform(T_rr_prime));
        // RCLCPP_INFO_STREAM(get_logger(), "ekf current rs" << ekf->get_robot_state().translation().x << " " << ekf->get_robot_state().translation().y << " " << ekf->get_robot_state().rotation());
        T_or_prev = T_or_now;
        // RCLCPP_INFO_STREAM(get_logger(), "Tor prev"<< T_or_prev.translation().x << " " << T_or_prev.translation().y << " " << T_or_prev.rotation());
        
        for(size_t j = 1; j <= msg.markers.size(); j++){
            // RCLCPP_INFO_STREAM(get_logger(), "marker size" << msg.markers.size());
            // RCLCPP_INFO_STREAM(get_logger(), "getinto fake sensor for loop" << j);
            if(msg.markers[j-1].action == visualization_msgs::msg::Marker::ADD){
                // RCLCPP_INFO_STREAM(get_logger(), "CCCCCCCC" << j);
                ekf->correct(msg.markers[j-1].pose.position.x, msg.markers[j-1].pose.position.y, j);
            }
        }
    }

    void broadcast_map_odom_transform(){
        ///ekf predicted turtle position
        T_m_r = ekf->get_robot_state();
        // RCLCPP_INFO_STREAM(get_logger(), "T_m_r: " << T_m_r.translation().x << " " << T_m_r.translation().y << " " << T_m_r.rotation());
        T_o_r = turtlelib::Transform2D{turtlelib::Vector2D{robot.get_current_config().translation().x, robot.get_current_config().translation().y}, robot.get_current_config().rotation()};
        T_m_o = T_m_r * T_o_r.inv();
        // RCLCPP_INFO_STREAM(get_logger(), "T_m_o: " << T_m_o.translation().x << " " << T_m_o.translation().y << " " << T_m_o.rotation());

        geometry_msgs::msg::TransformStamped T_m_o_msg;
        T_m_o_msg.header.stamp = get_clock()->now();
        T_m_o_msg.header.frame_id = "map";
        T_m_o_msg.child_frame_id = odom_id;
        T_m_o_msg.transform.translation.x = T_m_o.translation().x;
        T_m_o_msg.transform.translation.y = T_m_o.translation().y;
        tf2::Quaternion q2;
        q2.setRPY(0, 0, T_m_o.rotation());
        T_m_o_msg.transform.rotation.x = q2.x();
        T_m_o_msg.transform.rotation.y = q2.y();
        T_m_o_msg.transform.rotation.z = q2.z();
        T_m_o_msg.transform.rotation.w = q2.w();

        tf_br2->sendTransform(T_m_o_msg);
    }

    void publish_seen_obstacle(){
        auto obs_array = ekf->get_map();
        ///how many landmarks in the map
        auto num_landmarks_map = obs_array.size()/2.0;
        visualization_msgs::msg::MarkerArray obstacle_arr;
        for(size_t lm = 0; lm < num_landmarks_map; lm++){
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = get_clock()->now();
            marker.id = lm;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = obs_array[2*lm];
            marker.pose.position.y = obs_array[2*lm+1];
            marker.pose.position.z = 0.125;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = obstacles_r*2;
            marker.scale.y = obstacles_r*2;
            marker.scale.z = 0.25;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            obstacle_arr.markers.push_back(marker);
            seen_object = true;
        }
        if(seen_object){
            obstacles_publisher_->publish(obstacle_arr);
        }

    }

    void initial_pose(
    std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response> response){
        turtlelib::Transform2D currConfig(turtlelib::Vector2D{request->x, request->y}, request->theta);
        robot = turtlelib::DiffDrive(currConfig, wheel_radius, track_width);
        response->success = true;
    }
    
    
    
    std::string wheel_left, wheel_right, body_id, odom_id;
    double wheel_radius, track_width;
    double basic_sensor_variance, obstacles_r, max_range;
    sensor_msgs::msg::JointState last_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_green;
    nav_msgs::msg::Path robot_path;
    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_;
    turtlelib::DiffDrive robot;
    nav_msgs::msg::Odometry odom;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br2;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_subscriber_;
    std::unique_ptr<turtlelib::EKFSlam> ekf;
    geometry_msgs::msg::TransformStamped transformStamped;
    bool first = true;
    bool seen_object = false;
    turtlelib::Transform2D T_or_now, T_or_prev, green_turtle, T_m_r, T_o_r, T_m_o;
    


};

/// \brief main function to create and run Odometry node
/// \param argc
/// \param argv
/// \return int
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nuslam>());
  rclcpp::shutdown();
  return 0;
}
