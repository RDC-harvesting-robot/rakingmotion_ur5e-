from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "direction",
            default_value="R",
            description="Set movement direction: L or R"
        ),
        Node(
            package="rakingmotion_ur5e",
            executable="rakingmotion",
            name="rakingmotion",
            output="screen",
            parameters=[{"direction": LaunchConfiguration("direction")}]
        )#,
        # Node(
        #     package="leptrino_force_sensor",
        #     executable="leptrino_force_sensor_node",
        #     name="leptrino_force_sensor",
        #     output="screen"
        # ),
        # Node(
        #     package="leptrino_force_sensor",
        #     executable='force_calibrator_node',
        #     name='force_calibrator'
        #     output="screen"
        # )
    ])
