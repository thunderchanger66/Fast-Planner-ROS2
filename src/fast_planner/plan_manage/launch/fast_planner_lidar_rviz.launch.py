import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("plan_manage")
    planner_launch = os.path.join(package_share, "launch", "fast_planner_lidar.launch.py")
    default_rviz = os.path.join(package_share, "rviz2_fastplanner.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    planning_frame = LaunchConfiguration("planning_frame")

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch),
        launch_arguments={
            "odom_topic": LaunchConfiguration("odom_topic"),
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "goal_topic": LaunchConfiguration("goal_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "planning_frame": planning_frame,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="fast_planner_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config"), "-f", planning_frame],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/fastlio2/lio_odom"),
        DeclareLaunchArgument("cloud_topic", default_value="/fastlio2/world_cloud"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        DeclareLaunchArgument("output_topic", default_value="/drone_0_planning/pos_cmd"),
        DeclareLaunchArgument("planning_frame", default_value="lidar"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        planner,
        rviz,
    ])
