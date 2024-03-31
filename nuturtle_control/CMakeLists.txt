cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# This CMakeLists.txt only include the lines needed to set up this test
# in your own custom ROS 2 package. Other necessary CMake commands
# (such as the boilerplate ROS 2 items added when using ros2 pkg create)
# are omitted.

# THIS LINE SHOULD BE UNCOMMENTED - it is only commented here
# so this file can be included in the top level CMakeLists.txt
# find_package(catch_ros2 REQUIRED) # UNCOMMENT
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtle_description REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/Control.srv"
  "srv/InitialPose.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

##########################
## INTEGRATION AUX NODE ##
##########################
# This could be any node in your ROS 2 package
# Likely you are looking to test some functionality of this node

find_package(rosidl_default_generators REQUIRED)

add_executable(turtle_control_node
  src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)
  
ament_target_dependencies(turtle_control_node rclcpp std_srvs geometry_msgs turtlelib nuturtle_description nuturtlebot_msgs sensor_msgs)
ament_target_dependencies(odometry rclcpp geometry_msgs turtlelib  nav_msgs tf2 tf2_ros sensor_msgs)
ament_target_dependencies(circle rclcpp geometry_msgs turtlelib std_srvs sensor_msgs)

target_link_libraries(turtle_control_node turtlelib::turtlelib ${cpp_typesupport_target})
target_link_libraries(odometry turtlelib::turtlelib ${cpp_typesupport_target})
target_link_libraries(circle turtlelib::turtlelib ${cpp_typesupport_target})

install(TARGETS
  turtle_control_node
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  srv
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  include(CTest)
  find_package(catch_ros2 REQUIRED)
  ###########################
  ## INTEGRATION TEST NODE ##
  ###########################
  # This is the node in which integration tests occur
  add_executable(turtle_control_test_node
    test/turtle_control_test.cpp
  )
  add_executable(turtle_odom_test_node test/turtle_odom_test_node.cpp)
  add_executable(turtle_circle_test test/circle_test.cpp)
  # The link libraries call links this node with catch_ros2::catch_ros2_with_node_main
  # to get the default integration test node main function
  target_link_libraries(turtle_control_test_node
    catch_ros2::catch_ros2_with_node_main turtlelib::turtlelib ${cpp_typesupport_target}
  )
  target_link_libraries(turtle_odom_test_node catch_ros2::catch_ros2_with_node_main turtlelib::turtlelib ${cpp_typesupport_target})
  target_link_libraries(turtle_circle_test catch_ros2::catch_ros2_with_node_main turtlelib::turtlelib)

  ament_target_dependencies(turtle_control_test_node
    rclcpp std_srvs geometry_msgs turtlelib nuturtlebot_msgs sensor_msgs)
  ament_target_dependencies(turtle_odom_test_node rclcpp nuturtlebot_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros turtlelib std_srvs)
  ament_target_dependencies(turtle_circle_test rclcpp geometry_msgs)
  
  install(TARGETS
    turtle_control_test_node
    turtle_odom_test_node
    turtle_circle_test
    DESTINATION lib/${PROJECT_NAME}
  )


  install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME})


  install(FILES
    test/turtle_control_test.launch.xml
    test/turtle_odom_test.launch.xml
    test/turtle_circle_test.launch.xml
    DESTINATION share/${PROJECT_NAME}/test)

  catch_ros2_add_integration_test(turtle_control_test
  LAUNCH_FILE turtle_control_test.launch.xml
  )
  catch_ros2_add_integration_test(turtle_odom_test
  LAUNCH_FILE turtle_odom_test.launch.xml
  )

  catch_ros2_add_integration_test(turtle_circle_test
  LAUNCH_FILE turtle_circle_test.launch.xml
  )

endif()

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" ON)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
  # Turn the README.md into the homepage of the doxygen docs
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen include/ src/ README.md ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

ament_package()
