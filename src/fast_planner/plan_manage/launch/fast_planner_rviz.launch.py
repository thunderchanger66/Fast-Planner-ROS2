import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("plan_manage"), "rviz2_fastplanner.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=default_config),
        DeclareLaunchArgument("planning_frame", default_value="lidar"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="fast_planner_rviz",
            output="screen",
            arguments=[
                "-d", LaunchConfiguration("rviz_config"),
                "-f", LaunchConfiguration("planning_frame"),
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
