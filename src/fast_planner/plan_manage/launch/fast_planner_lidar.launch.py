import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("plan_manage")
    planner_config = os.path.join(
        package_share, "config", "fast_planner_lidar.yaml")

    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    output_topic = LaunchConfiguration("output_topic")
    planning_frame = LaunchConfiguration("planning_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")

    planner = Node(
        package="plan_manage",
        executable="fast_planner_node",
        name="fast_planner_node",
        output="screen",
        parameters=[planner_config, {
            "use_sim_time": use_sim_time,
            "goal_topic": goal_topic,
            "sdf_map/frame_id": planning_frame,
        }],
        remappings=[
            ("/odom_world", odom_topic),
            ("/sdf_map/odom", odom_topic),
            ("/sdf_map/cloud", cloud_topic),
        ],
    )

    trajectory_server = Node(
        package="plan_manage",
        executable="traj_server",
        name="fast_planner_traj_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "planning_frame": planning_frame,
        }],
        remappings=[
            ("/odom_world", odom_topic),
            ("/position_cmd", output_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_topic", default_value="/fastlio2/lio_odom",
            description="FAST-LIO nav_msgs/Odometry topic"),
        DeclareLaunchArgument(
            "cloud_topic", default_value="/fastlio2/world_cloud",
            description="FAST-LIO PointCloud2 topic"),
        DeclareLaunchArgument(
            "goal_topic", default_value="/goal_pose",
            description="PoseStamped goal in the planner frame"),
        DeclareLaunchArgument(
            "output_topic", default_value="/drone_0_planning/pos_cmd",
            description="quadrotor_msgs/PositionCommand topic for px4_bridge"),
        DeclareLaunchArgument(
            "planning_frame", default_value="lidar",
            description="Must match FAST-LIO odometry and point cloud frames"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use Gazebo simulation clock"),
        planner,
        trajectory_server,
    ])
