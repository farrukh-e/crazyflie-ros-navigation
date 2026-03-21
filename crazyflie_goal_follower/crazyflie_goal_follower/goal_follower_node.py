#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration as MsgDuration
from nav_msgs.msg import Odometry

from crazyflie_interfaces.srv import Takeoff, GoTo


def to_msg_duration(seconds: float) -> MsgDuration:
    """!
    @brief Convert floating-point seconds into a ROS Duration message.
    @param seconds Duration in seconds.
    @return Converted duration message.
    """
    d = MsgDuration()
    d.sec = int(seconds)
    d.nanosec = int((seconds - d.sec) * 1e9)
    return d


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    """!
    @brief Convert quaternion orientation to yaw in degrees.
    @param x Quaternion x component.
    @param y Quaternion y component.
    @param z Quaternion z component.
    @param w Quaternion w component.
    @return Yaw angle in degrees.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


class GoalFollowerNode(Node):

    def __init__(self):
        """!
        @brief Initialize node parameters, service clients, and subscriptions.
        """
        super().__init__("goal_follower_node")

        self.declare_parameter("robot_prefix", "crazyflie")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("use_takeoff", True)
        self.declare_parameter("takeoff_height", 0.5)
        self.declare_parameter("takeoff_duration_sec", 2.0)
        self.declare_parameter("go_to_duration_sec", 3.0)
        self.declare_parameter("goal_follow_height", 0.5)
        self.declare_parameter("wait_for_services_timeout_sec", 10.0)

        self.prefix = self.get_prefix()
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.use_takeoff = bool(self.get_parameter("use_takeoff").value)
        self.takeoff_height = float(self.get_parameter("takeoff_height").value)
        self.takeoff_duration_sec = float(self.get_parameter("takeoff_duration_sec").value)
        self.go_to_duration_sec = float(self.get_parameter("go_to_duration_sec").value)
        self.goal_follow_height = float(self.get_parameter("goal_follow_height").value)
        self.wait_timeout = float(self.get_parameter("wait_for_services_timeout_sec").value)
        self.has_odometry = False
        self.has_taken_off = False
        self.takeoff_in_progress = False

        self.get_logger().info(f"Using robot prefix: /{self.prefix}")

        # Clients
        self.takeoff_cli = self.create_client(Takeoff, f"/{self.prefix}/takeoff")
        self.goto_cli = self.create_client(GoTo, f"/{self.prefix}/go_to")

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, f"/{self.prefix}/odom", self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)

        self.wait_for_services()
        self.get_logger().info(f"Goal follower ready. Listening on {self.goal_topic}")

    def get_prefix(self) -> str:
        """!
        @brief Read and sanitize the robot namespace prefix parameter.
        @return Robot prefix without leading/trailing slashes.
        """
        prefix = str(self.get_parameter("robot_prefix").value).strip("/")
        return prefix or "crazyflie"

    def wait_for_services(self):
        """!
        @brief Wait for required services to become available.
        @throws RuntimeError If any required service is unavailable within timeout.
        """
        if not self.goto_cli.wait_for_service(timeout_sec=self.wait_timeout):
            raise RuntimeError(f"Service not available within {self.wait_timeout}s: /{self.prefix}/go_to")
        if self.use_takeoff and not self.takeoff_cli.wait_for_service(timeout_sec=self.wait_timeout):
            raise RuntimeError(f"Service not available within {self.wait_timeout}s: /{self.prefix}/takeoff")

    def odom_cb(self, _msg: Odometry):
        """!
        @brief Odometry callback used to mark odometry availability.
        @param _msg Incoming odometry message.
        """
        is_first_odom = not self.has_odometry
        self.has_odometry = True

        if is_first_odom:
            self.get_logger().info("Odometry available")
            if self.use_takeoff:
                self.get_logger().info("Initiating takeoff after odometry became available")
                self.maybe_takeoff()

    def call(self, client, request, label: str, on_success=None, on_error=None):
        """!
        @brief Call a ROS service asynchronously and return immediately.
        @param client Service client instance.
        @param request Service request object.
        @param label Label used in error messages.
        @param on_success Optional callback called with service response on success.
        @param on_error Optional callback called with exception on failure.
        @return Service future.
        """
        future = client.call_async(request)

        def done_cb(fut):
            try:
                result = fut.result()
                if result is None:
                    exc = fut.exception()
                    raise RuntimeError(exc if exc is not None else "Unknown service failure")
                if on_success is not None:
                    on_success(result)
            except Exception as exc:
                self.get_logger().error(f"{label} failed: {exc}")
                if on_error is not None:
                    on_error(exc)

        future.add_done_callback(done_cb)
        return future

    def maybe_takeoff(self) -> bool:
        """!
        @brief Send one-time automatic takeoff.
        @return True if goal processing can continue, False otherwise.
        """
        if not self.use_takeoff or self.has_taken_off:
            return True
        if self.takeoff_in_progress:
            return False

        req = Takeoff.Request(group_mask=0, height=self.takeoff_height,
                              duration=to_msg_duration(self.takeoff_duration_sec))
        self.takeoff_in_progress = True
        self.call(
            self.takeoff_cli,
            req,
            "takeoff",
            on_success=self.takeoff_done_cb,
            on_error=self.takeoff_error_cb,
        )
        self.get_logger().info("Takeoff sent before first goal")
        return False

    def takeoff_done_cb(self, _response):
        """!
        @brief Handle successful takeoff service completion.
        @param _response Service response object.
        """
        self.takeoff_in_progress = False
        self.has_taken_off = True
        self.get_logger().info("Takeoff service completed")

    def takeoff_error_cb(self, _exception: Exception):
        """!
        @brief Handle takeoff service failure.
        @param _exception Exception raised by service call.
        """
        self.takeoff_in_progress = False
        self.has_taken_off = False

    def goal_cb(self, msg: PoseStamped):
        """!
        @brief Handle incoming goal poses and send go_to requests.
        @param msg Target pose message.
        """
        if not self.has_odometry:
            self.get_logger().warn("Ignoring goal: no odometry yet")
            return

        try:
            if not self.maybe_takeoff():
                return

            yaw_deg = quat_to_yaw_deg(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )

            req = GoTo.Request()
            req.group_mask = 0
            req.relative = False
            req.goal.x = float(msg.pose.position.x)
            req.goal.y = float(msg.pose.position.y)
            req.goal.z = float(self.goal_follow_height)
            req.yaw = float(yaw_deg)
            req.duration = to_msg_duration(self.go_to_duration_sec)

            self.call(self.goto_cli, req, "go_to")
            self.get_logger().info(
                f"Sent go_to: x={req.goal.x:.2f} y={req.goal.y:.2f} "
                f"z={req.goal.z:.2f} yaw={req.yaw:.1f}deg"
            )
        except Exception as exc:
            self.get_logger().error(f"Goal handling failed: {exc}")


def main():
    rclpy.init()
    node = GoalFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
