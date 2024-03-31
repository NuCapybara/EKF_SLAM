/// \file turtle_control.cpp
/// \brief Publishes wheel commands and joint states of the robot
/// \brief Subscribes to twist and calculates wheel commands.
/// \brief Subscribes to sensor data and calculates joint states.
///
/// PARAMETERS:
///     wheel_radius (double): radius of the wheel
///     track_width (double): width of the track
///     motor_cmd_max (double): maximum motor command
///     motor_cmd_per_rad_sec (double): motor command per radian per second
///     encoder_ticks_per_rad (double): encoder ticks per radian
///     collision_radius (double): radius of the collision
/// SUBSCRIBES:
///     ~/cmd_vel (geometry_msgs::msg::Twist): velocity command
///     ~/sensor_data (nuturtlebot_msgs::msg::SensorData): sensor data
/// PUBLISHES:
///     ~/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): wheel commands
///     ~/joint_states (sensor_msgs::msg::JointState): joint states
/// SERVICES:
///     none
/// CLIENTS:
///     none
/// BROADCASTS:
///     none


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
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;

/// @brief class to control the turtle
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    ///parameter delcaration
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

    this->declare_parameter("motor_cmd_max", -1.);
    motor_cmd_max = this->get_parameter("motor_cmd_max").as_double();
    if(motor_cmd_max == 0 || motor_cmd_max < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Motor_cmd_max error" << 4);
    }

    declare_parameter("motor_cmd_per_rad_sec", -1.);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    if(motor_cmd_per_rad_sec == 0 || motor_cmd_per_rad_sec < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Motor_cmd_per_rad_sec error" << 4);
    }

    declare_parameter("encoder_ticks_per_rad", -1.);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    if(encoder_ticks_per_rad == 0 || encoder_ticks_per_rad < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Encoder_ticks_per_rad error" << 4);
    }

    declare_parameter("collision_radius", -1.);
    collision_radius = get_parameter("collision_radius").as_double();
    if(collision_radius == 0 || collision_radius < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Collision_radius error" << 4);
    }

    ///Cmd subscription and publisher
    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    ///subscribe to the sensor data
    sensor_data_subscription_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
        "sensor_data",10, std::bind(&TurtleControl::sensor_callback, this, std::placeholders::_1));

    ///provide the angle (in radians) and velocity (in rad/sec) of each wheel
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    robot = turtlelib::DiffDrive(wheel_radius, track_width);
    // RCLCPP_INFO_STREAM(this->get_logger(), "track width and radius "<< track_width << " "<< wheel_radius);
    
  }

private:

    void cmd_vel_callback(const geometry_msgs::msg::Twist & msg) 
    {   turtlelib::Twist2D twist{msg.angular.z, msg.linear.x, 0.0};


        turtlelib::WheelState ws = robot.inverseKinematics(twist);


        double left_velocity = ws.l / motor_cmd_per_rad_sec;
        double right_velocity = ws.r / motor_cmd_per_rad_sec;
        if(left_velocity > motor_cmd_max){
            left_velocity = motor_cmd_max;
        }
        else if(left_velocity < -motor_cmd_max){
            left_velocity = -motor_cmd_max;
        }
        if(right_velocity > motor_cmd_max){
            right_velocity = motor_cmd_max;
        }
        else if(right_velocity < -motor_cmd_max){
            right_velocity = -motor_cmd_max;
        }
        nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
        wheel_cmd.left_velocity = static_cast<int>(left_velocity);
        wheel_cmd.right_velocity = static_cast<int>(right_velocity);
        wheel_cmd_publisher_->publish(wheel_cmd);

    }

    void sensor_callback(const nuturtlebot_msgs::msg::SensorData & msg) 
        {
        double left_encoder = msg.left_encoder;
        double right_encoder = msg.right_encoder;

        left_wheel_joint = static_cast<double> (left_encoder) / encoder_ticks_per_rad;
        right_wheel_joint  = static_cast<double> (right_encoder) / encoder_ticks_per_rad;


        if(initial_js == true){
            initial_js = false;
            joint_state.name = {"wheel_left_joint", "wheel_right_joint"};
            joint_state.header.stamp =  msg.stamp;
            joint_state.position = {left_wheel_joint, right_wheel_joint};
            joint_state.velocity = {0.0, 0.0};
            

        }
        else{
            auto del_t = msg.stamp.sec + msg.stamp.nanosec * 1e-9 - joint_state.header.stamp.sec -
            joint_state.header.stamp.nanosec * 1e-9;

            joint_state.position = {left_wheel_joint, right_wheel_joint};

            double left_wheel_velocity =(left_wheel_joint - joint_state.position.at(0)) / del_t;
            double right_wheel_velocity = (right_wheel_joint - joint_state.position.at(1)) / del_t;

            joint_state.header.stamp = msg.stamp;
            joint_state.velocity = {left_wheel_velocity, right_wheel_velocity};
            joint_state_publisher_->publish(joint_state);
        }
        
    }

    
    
    
    turtlelib::Twist2D twist;
    bool initial_js = true;
    sensor_msgs::msg::JointState joint_state, last_jointstate;
    double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
    double left_wheel_joint, right_wheel_joint;
    turtlelib::DiffDrive robot;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

};

/// @brief main function to start the node
/// @param argc
/// @param argv
/// @return int
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
