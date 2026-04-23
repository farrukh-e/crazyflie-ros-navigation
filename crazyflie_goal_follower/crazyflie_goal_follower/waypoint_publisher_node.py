#!/usr/bin/env python3
from pathlib import Path
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

Waypoint = Tuple[float, float, float]

class WaypointPublisherNode(Node):
    def __init__(self):
        """!
        @brief Initialize node parameters, publisher, and timers.
        """
        super().__init__("waypoint_publisher_node")

        self.goal_topic = str(self.declare_and_get("goal_topic", "/goal_pose"))
        self.waypoints_file = str(self.declare_and_get("waypoints_txt", ""))
        self.frame_id = str(self.declare_and_get("frame_id", "map"))
        self.default_z = float(self.declare_and_get("default_z", 0.5))
        self.publish_period_sec = float(self.declare_and_get("publish_period_sec", 1.05))
        self.start_delay_sec = float(self.declare_and_get("start_delay_sec", 5.0))

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
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

    def publish_waypoint(self):
        """! @brief Publish the next waypoint as a PoseStamped goal."""
        if self.next_waypoint_idx >= len(self.waypoints):
            self.publish_timer.cancel()
            self.publish_timer = None
            return

        x, y, z = self.waypoints[self.next_waypoint_idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"Published waypoint {self.next_waypoint_idx + 1}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, z={z:.2f}"
        )
        self.next_waypoint_idx += 1

        if self.next_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("Finished publishing all waypoints")
            self.publish_timer.cancel()
            self.publish_timer = None

    def load_waypoints(self, waypoints_file: str) -> List[Waypoint]:
        """!
        @brief Load waypoint rows from a text file.
        @details Each non-empty line should look like:
                 x, y or x, y, z
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
                waypoints.append((x, y, z))
        if not waypoints:
            raise RuntimeError(f"No valid waypoints found in file: {path}")
        return waypoints


def main():
    rclpy.init()
    node = WaypointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Launch may have already shut down this context.
            pass

if __name__ == "__main__":
    main()
