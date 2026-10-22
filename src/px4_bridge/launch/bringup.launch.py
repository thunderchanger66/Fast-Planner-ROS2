import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory("px4_bridge")
    planner_share = get_package_share_directory("plan_manage")

    bridge_params = LaunchConfiguration("bridge_params")
    rviz_config = LaunchConfiguration("rviz_config")
    start_agent = LaunchConfiguration("start_agent")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_params",
            default_value=os.path.join(bridge_share, "config", "sitl_control.yaml"),
            description="Bridge parameter file; defaults to the checked-in SITL control profile"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("plan_manage"), "rviz2_fastplanner.rviz"),
            description="RViz configuration for Fast-Planner trajectory and map displays"),
        DeclareLaunchArgument(
            "start_agent",
            default_value="true",
            description=(
                "Start Micro XRCE-DDS Agent on UDP 8888. Set false when an agent is "
                "already running for PX4")),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            name="micro_xrce_agent",
            output="screen",
            condition=IfCondition(start_agent),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_share, "launch", "fast_planner_lidar.launch.py")),
        ),
        Node(
            package="px4_bridge",
            executable="px4_bridge_node",
            name="px4_bridge",
            output="screen",
            parameters=[bridge_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="fast_planner_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
