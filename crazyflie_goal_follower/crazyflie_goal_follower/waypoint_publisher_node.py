#!/usr/bin/env python3
import math
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from builtin_interfaces.msg import Duration as MsgDuration
from crazyflie_interfaces.srv import Land

Waypoint = Tuple[float, float, float, Optional[float]]

class WaypointPublisherNode(Node):
    def __init__(self):
        """!
        @brief Initialize node parameters, publisher, and timers.
        """
        super().__init__("waypoint_publisher_node")
        self.robot_prefix = str(self.declare_and_get("robot_prefix", "crazyflie")).strip("/")
        self.goal_topic = str(self.declare_and_get("goal_topic", "/goal_pose"))
        self.waypoints_file = str(self.declare_and_get("waypoints_txt", ""))
        self.frame_id = str(self.declare_and_get("frame_id", "map"))
        self.default_z = float(self.declare_and_get("default_z", 0.5))
        self.publish_period_sec = float(self.declare_and_get("publish_period_sec", 1.05))
        self.start_delay_sec = float(self.declare_and_get("start_delay_sec", 5.0))
        self.land_after_last_waypoint = bool(self.declare_and_get("land_after_last_waypoint", True))
        self.land_height = float(self.declare_and_get("land_height", 0.0))
        self.land_duration_sec = float(self.declare_and_get("land_duration_sec", 3.0))


        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.land_cli = self.create_client(Land, f"/{self.robot_prefix}/land")
        self.waypoints = self.load_waypoints(self.waypoints_file)
        self.next_waypoint_idx = 0
        self.publish_timer = None
        self.start_timer = None

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from: {self.waypoints_file}"
        )
        self.get_logger().info(
            f"Publishing PoseStamped goals on {self.goal_topic} "
            f"(period={self.publish_period_sec:.2f}s)"
        )

        self.start_timer = self.create_timer(self.start_delay_sec, self.start_publishing)
        self.get_logger().info(f"Waiting {self.start_delay_sec:.2f}s before publishing first waypoint")

    def declare_and_get(self, name: str, default):
        """! @brief Declare a parameter and return its value."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def start_publishing(self):
        """! @brief Start periodic waypoint publishing."""
        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None

        self.publish_waypoint()
        if self.next_waypoint_idx < len(self.waypoints):
            self.publish_timer = self.create_timer(self.publish_period_sec, self.publish_waypoint)

    def send_land(self):
        """! @brief Send a land request to the Crazyflie"""
        if not self.land_cli.service_is_ready():
            self.get_logger().warn(f"/{self.robot_prefix}/land not ready; skipping land")
            return
        req = Land.Request()
        req.group_mask = 0
        req.height = self.land_height
        req.duration = MsgDuration()
        req.duration.sec = int(self.land_duration_sec)
        req.duration.nanosec = int((self.land_duration_sec - int(self.land_duration_sec)) * 1e9)
        self.land_cli.call_async(req)
        self.get_logger().info("Sent land request after last waypoint")

    def publish_waypoint(self):
        """! @brief Publish the next waypoint as a PoseStamped goal."""
        if self.next_waypoint_idx >= len(self.waypoints):
            self.publish_timer.cancel()
            self.publish_timer = None
            if self.land_after_last_waypoint:
                self.send_land()
            return

        x, y, z, yaw_rad = self.waypoints[self.next_waypoint_idx]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.5
        if yaw_rad is not None:
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_rad)
            msg.pose.orientation.x = float(qx)
            msg.pose.orientation.y = float(qy)
            msg.pose.orientation.z = float(qz)
            msg.pose.orientation.w = float(qw)
        else:
            # Keep a valid identity quaternion when heading is omitted.
            msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        if yaw_rad is not None:
            self.get_logger().info(
                f"Published waypoint {self.next_waypoint_idx + 1}/{len(self.waypoints)}: "
                f"x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw_deg={math.degrees(yaw_rad):.1f}"
            )
        else:
            self.get_logger().info(
                f"Published waypoint {self.next_waypoint_idx + 1}/{len(self.waypoints)}: "
                f"x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw=omitted"
            )
        self.next_waypoint_idx += 1

        if self.next_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("Finished publishing all waypoints")
            self.publish_timer.cancel()
            self.publish_timer = None
            if self.land_after_last_waypoint:
                self.send_land()

    def load_waypoints(self, waypoints_file: str) -> List[Waypoint]:
        """!
        @brief Load waypoint rows from a text file.
        @details Each non-empty line should look like:
                 x, y or x, y, z or x, y, z, yaw_deg
        """
        path = Path(waypoints_file).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"Waypoint file not found: {path}")

        waypoints: List[Waypoint] = []
        with path.open('r') as waypoint_file_obj:
            for line in waypoint_file_obj:
                coordinates = [coord.strip() for coord in line.split(',')]
                if not line.strip():
                    continue  # Skip empty lines
                elif coordinates[0].startswith('#'):
                    self.get_logger().info("Waypoint goal comment: " + line.strip())
                    continue  # Skip comment lines
                x = float(coordinates[0])
                y = float(coordinates[1])
                z = float(coordinates[2]) if len(coordinates) > 2 else self.default_z
                yaw_rad = math.radians(float(coordinates[3])) if len(coordinates) > 3 else None
                waypoints.append((x, y, z, yaw_rad))
        if not waypoints:
            raise RuntimeError(f"No valid waypoints found in file: {path}")
        return waypoints


def main():
    rclpy.init()
    node = WaypointPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
