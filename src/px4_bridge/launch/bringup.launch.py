import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory("px4_bridge")
    planner_share = get_package_share_directory("plan_manage")

    bridge_params = LaunchConfiguration("bridge_params")
    rviz_config = LaunchConfiguration("rviz_config")
    start_agent = LaunchConfiguration("start_agent")
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    output_topic = LaunchConfiguration("output_topic")
    planning_frame = LaunchConfiguration("planning_frame")

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_share, "launch", "fast_planner_lidar.launch.py")),
        launch_arguments={
            "odom_topic": odom_topic,
            "cloud_topic": cloud_topic,
            "goal_topic": goal_topic,
            "output_topic": output_topic,
            "planning_frame": planning_frame,
            "use_sim_time": use_sim_time,
        }.items(),
    )

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
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/fastlio2/lio_odom"),
        DeclareLaunchArgument("cloud_topic", default_value="/fastlio2/world_cloud"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        DeclareLaunchArgument(
            "output_topic", default_value="/drone_0_planning/pos_cmd",
            description="Planner command topic consumed by px4_bridge"),
        DeclareLaunchArgument("planning_frame", default_value="lidar"),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            name="micro_xrce_agent",
            output="screen",
            condition=IfCondition(start_agent),
            respawn=True,
            respawn_delay=2.0,
        ),
        planner,
        Node(
            package="px4_bridge",
            executable="px4_bridge_node",
            name="px4_bridge",
            output="screen",
            parameters=[bridge_params, {
                "use_sim_time": use_sim_time,
                "planner_command_topic": output_topic,
                "planner_frame": planning_frame,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="fast_planner_rviz",
            output="screen",
            arguments=["-d", rviz_config, "-f", planning_frame],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
