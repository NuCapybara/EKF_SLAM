from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    PythonExpression
)
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, GroupAction
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.events import Shutdown


# use Xacro files to make life easier
def generate_launch_description():

    return LaunchDescription(
        [
            GroupAction(
                actions=[
                    DeclareLaunchArgument(
                        name="use_jsp",
                        default_value="true",
                        description="if true, jsp is used to publish joint states",
                    ),
                    DeclareLaunchArgument(
                        name="use_rviz",
                        default_value="true",
                        description="control whether rviz is launched",
                    ),
                    DeclareLaunchArgument(
                        name="color",
                        default_value="purple",
                        description="the color of the baselink",
                        choices=["purple", "red", "green", "blue"],
                    ),
                    DeclareLaunchArgument(
                        name="x",
                        default_value="0",
                        description="the x of robot",
                    ),
                    DeclareLaunchArgument(
                        name="y",
                        default_value="0",
                        description="the y of robot",
                    ),
                    SetLaunchConfiguration(
                        name="rviz_config",
                        value=["basic_", LaunchConfiguration("color"), ".rviz"],
                    ),
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        name="robot_state_publisher",
                        namespace=LaunchConfiguration("color"),
                        parameters=[
                            {
                                "robot_description": Command(
                                    [
                                        ExecutableInPackage("xacro", "xacro"),
                                        " ",
                                        PathJoinSubstitution(
                                            [
                                                FindPackageShare(
                                                    "nuturtle_description"
                                                ),
                                                "turtlebot3_burger.urdf.xacro",
                                            ]
                                        ),
                                        " ",
                                        "color:=",
                                        LaunchConfiguration("color"),
                                    ]
                                )
                            },
                            {"frame_prefix": [LaunchConfiguration("color"), "/"]},
                        ],
                    ),
                    Node(
                        package="joint_state_publisher",
                        executable="joint_state_publisher",
                        name="joint_state_publisher",
                        namespace=LaunchConfiguration("color"),
                        condition=IfCondition(
                            EqualsSubstitution(LaunchConfiguration("use_jsp"), "true")
                        ),
                    ),
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        namespace=LaunchConfiguration("color"),
                        name="rviz",
                        arguments=[
                            "-d",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("nuturtle_description"),
                                    LaunchConfiguration("rviz_config"),
                                ]
                            ),
                            " ",
                            "-f",
                            [LaunchConfiguration("color"), "/base_footprint"],
                        ],
                        condition=IfCondition(
                            EqualsSubstitution(LaunchConfiguration("use_rviz"), "true")
                        ),
                        on_exit=Shutdown(),
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        namespace=LaunchConfiguration("color"),
                        arguments=[
                            "--x",
                            LaunchConfiguration("x"),
                            "--y",
                            LaunchConfiguration("y"),
                            "--z",
                            "0",
                            "--yaw",
                            "0",
                            "--pitch",
                            "0",
                            "--roll",
                            "0",
                            "--frame-id",
                            "nusim/world",
                            "--child-frame-id",
                            [LaunchConfiguration("color"), "/base_footprint"],
                        ],
                        condition=IfCondition(
                            PythonExpression(
                                [
                                    "('",
                                    LaunchConfiguration("color"),
                                    "'== 'purple') or ('",
                                    LaunchConfiguration("color"),
                                    "' == 'green')",
                                ]
                            )
                        ),
                    ),
                ]
            ),
        ]
    )
