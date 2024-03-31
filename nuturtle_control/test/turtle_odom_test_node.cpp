// #include "catch_ros2/catch_ros2.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include <chrono>
// #include <cmath>
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/u_int64.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "nuturtlebot_msgs/msg/wheel_commands.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "nuturtlebot_msgs/msg/sensor_data.hpp"
// #include "turtlelib/se2d.hpp"
// #include "turtlelib/diff_drive.hpp"
// #include "turtlelib/geometry2d.hpp"
// #include "nuturtle_control/srv/initial_pose.hpp"
// #include "nuturtle_control/srv/control.hpp"


// using namespace std::chrono_literals;


// turtlelib::DiffDrive robot{0.033, 0.16};
// /// Twist message for testing cmd_callback
// geometry_msgs::msg::Twist cmd;
// /// SensorData message for testing sensor_data_callback
// nuturtlebot_msgs::msg::SensorData sensor_msg;

// TEST_CASE("initial_pose_testing", "odom_test") {
//     ///decleare a node
//     auto node = rclcpp::Node::make_shared("initial_pose");
//     ///test duration parameter declaration
//     node->declare_parameter<double>("test_duration", 1.0);
//     const auto TEST_DURATION =
//     node->get_parameter("test_duration").get_parameter_value().get<double>();


//     auto init_client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

//     auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
//     request->x = 5.;
//     request->y = 5.;
//     request->theta = 1/2;

//     auto result = init_client->async_send_request(request);
//     REQUIRE(result.get()->success == true);

//     while (!init_client->wait_for_service(1s)) {
//         if (!rclcpp::ok()) {
//             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//         }
        
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//     }

//     rclcpp::Time start_time = rclcpp::Clock().now();


//     RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I published cmd_vel");

//     while (
//         rclcpp::ok() &&
//         ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
//     )
//     {
//     rclcpp::spin_some(node);
//     }
// }
