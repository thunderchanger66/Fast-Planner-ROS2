#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, String


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a planner-frame goal position and optionally hold a manual yaw.")
    parser.add_argument("x", type=float, help="goal x in the planner frame, metres")
    parser.add_argument("y", type=float, help="goal y in the planner frame, metres")
    parser.add_argument("z", type=float, help="goal z in the planner frame, metres")
    parser.add_argument(
        "yaw_deg", nargs="?", type=float,
        help="manual heading in degrees in the planner frame; positive is counter-clockwise")
    parser.add_argument(
        "--auto-yaw", action="store_true",
        help="clear manual yaw and let EGO choose heading from the path")
    parser.add_argument("--frame", default="lidar", help="goal frame (default: lidar)")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--manual-yaw-topic", default="/px4_bridge/manual_yaw")
    parser.add_argument(
        "--clear-manual-yaw-topic", default="/px4_bridge/clear_manual_yaw")
    parser.add_argument("--status-topic", default="/px4_bridge/status")
    parser.add_argument(
        "--wait-timeout", type=float, default=60.0,
        help="seconds to wait for subscribers, yaw acknowledgement, and startup takeoff")
    args = parser.parse_args()
    if args.auto_yaw and args.yaw_deg is not None:
        parser.error("do not provide yaw_deg together with --auto-yaw")
    if not args.auto_yaw and args.yaw_deg is None:
        parser.error("yaw_deg is required unless --auto-yaw is used")
    values = [args.x, args.y, args.z]
    if args.yaw_deg is not None:
        values.append(args.yaw_deg)
    if not all(math.isfinite(value) for value in values):
        parser.error("coordinates and yaw must be finite numbers")
    if args.wait_timeout <= 0.0:
        parser.error("--wait-timeout must be positive")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = Node("manual_goal_sender")
    goal_pub = node.create_publisher(PoseStamped, args.goal_topic, 10)
    yaw_pub = node.create_publisher(Float64, args.manual_yaw_topic, 10)
    clear_yaw_pub = node.create_publisher(Empty, args.clear_manual_yaw_topic, 10)
    bridge_status = []
    status_sub = node.create_subscription(
        String, args.status_topic, lambda msg: bridge_status.append(msg.data), 10)

    required_publishers = [goal_pub, clear_yaw_pub if args.auto_yaw else yaw_pub]
    deadline = time.monotonic() + args.wait_timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if (all(pub.get_subscription_count() > 0 for pub in required_publishers) and
                    status_sub.get_publisher_count() > 0):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            missing = [
                pub.topic_name for pub in required_publishers
                if pub.get_subscription_count() == 0
            ]
            if status_sub.get_publisher_count() == 0:
                missing.append(args.status_topic)
            raise RuntimeError("no subscribers discovered for: " + ", ".join(missing))

        if args.auto_yaw:
            clear_yaw_pub.publish(Empty())
            heading_text = "EGO path heading"
            expected_status = "manual_yaw_cleared_using_planner_yaw"
        else:
            yaw = Float64()
            yaw.data = math.radians(args.yaw_deg)
            yaw_pub.publish(yaw)
            heading_text = f"{args.yaw_deg:g} deg"
            expected_status = "manual_yaw_enabled:"

        deadline = time.monotonic() + args.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if any(status.startswith(expected_status) for status in bridge_status):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise RuntimeError("bridge did not acknowledge the requested yaw mode")

        # Wait for the bridge's controlled startup takeoff. A one-shot goal
        # published earlier can be lost while the bridge intentionally rejects
        # planner trajectories during that profile.
        ready_deadline = time.monotonic() + args.wait_timeout
        while rclpy.ok() and time.monotonic() < ready_deadline:
            if any(status.startswith(
                    "startup_takeoff_reached_target_waiting_for_planner_goal")
                    for status in bridge_status):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            recent = "; ".join(dict.fromkeys(bridge_status[-20:]))
            raise RuntimeError(
                "bridge did not report a completed startup takeoff; do not send a goal yet. "
                f"Recent bridge states: {recent or 'none received'}")

        goal = PoseStamped()
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.header.frame_id = args.frame
        goal.pose.position.x = args.x
        goal.pose.position.y = args.y
        goal.pose.position.z = args.z
        goal_yaw = 0.0 if args.auto_yaw else math.radians(args.yaw_deg)
        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)
        goal_pub.publish(goal)
        rclpy.spin_once(node, timeout_sec=0.2)
        print(
            f"Sent goal ({args.x:g}, {args.y:g}, {args.z:g}) m in {args.frame}; "
            f"heading: {heading_text}.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, KeyboardInterrupt) as exc:
        if str(exc):
            print(f"manual_goal: {exc}", file=sys.stderr)
        sys.exit(1)
