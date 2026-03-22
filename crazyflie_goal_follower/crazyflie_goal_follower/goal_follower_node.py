#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
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


class GoalFollowerNode(Node):

    def __init__(self):
        """!
        @brief Initialize node parameters, service clients, and subscriptions.
        """
        super().__init__("goal_follower_node")

        self.prefix = str(self.declare_and_get("robot_prefix", "/crazyflie"))
        self.goal_topic = self.declare_and_get("goal_topic", "/goal_pose")
        self.use_takeoff = self.declare_and_get("use_takeoff", True)
        self.takeoff_height = self.declare_and_get("takeoff_height", 0.5)
        self.takeoff_duration_sec = self.declare_and_get("takeoff_duration_sec", 2.0)
        self.go_to_duration_sec = self.declare_and_get("go_to_duration_sec", 5.0)
        self.goal_follow_height = self.declare_and_get("goal_follow_height", 0.5)
        self.wait_timeout = self.declare_and_get("wait_for_services_timeout_sec", 10.0)

        self.ready = False
        self.has_taken_off = False
        self.takeoff_in_progress = False
        self.initial_position = None
        self.current_position = None

        self.get_logger().info(f"Using robot prefix: /{self.prefix}")

        # Clients
        self.takeoff_cli = self.create_client(Takeoff, f"/{self.prefix}/takeoff")
        self.goto_cli = self.create_client(GoTo, f"/{self.prefix}/go_to")

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, f"/{self.prefix}/odom", self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.odom_log_timer = self.create_timer(1.0, self.log_current_odom)

        self.wait_for_services()
        self.get_logger().info(f"Goal follower ready. Listening on {self.goal_topic}")

    def declare_and_get(self, name: str, default):
        """Declare a parameter and return its value."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def wait_for_services(self):
        """!
        @brief Wait for required services to become available.
        @throws RuntimeError If any required service is unavailable within timeout.
        """
        required = [(self.goto_cli, "go_to")]
        if self.use_takeoff:
            required.append((self.takeoff_cli, "takeoff"))

        for client, name in required:
            if not client.wait_for_service(timeout_sec=self.wait_timeout):
                raise RuntimeError(
                    f"Service not available within {self.wait_timeout}s: /{self.prefix}/{name}"
                )

    def odom_cb(self, msg: Odometry):
        """!
        @brief Odometry callback used to mark odometry availability and capture initial pose.
        @param msg Incoming odometry message.
        """
        p = msg.pose.pose.position
        self.current_position = (float(p.x), float(p.y), float(p.z))

        if not self.ready:
            self.initial_position = (float(p.x), float(p.y), float(p.z))
            self.get_logger().info(
                f"Initial position captured: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}"
            )
            self.ready = True
            if self.use_takeoff:
                self.get_logger().info("Initiating automatic takeoff after odometry became available")
                self.ensure_takeoff_started()

    def log_current_odom(self):
        """! @brief Print the latest odometry position."""
        if self.current_position is None:
            return

        x, y, z = self.current_position
        self.get_logger().info(f"Current odom position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def call_service(self, client, request, label: str, on_success=None, on_error=None):
        """!
        @brief Call a ROS service asynchronously and return immediately.
        @param client Service client instance.
        @param request Service request object.
        @param label Label used in error messages.
        @param on_success Optional callback invoked with service response.
        @param on_error Optional callback invoked with exception.
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

    def ensure_takeoff_started(self) -> bool:
        """! @brief Ensure that the takeoff process has been initiated if required."""
        if not self.use_takeoff or self.has_taken_off:
            return True
        if self.takeoff_in_progress:
            return False

        req = Takeoff.Request(
            group_mask=0,
            height=float(self.takeoff_height),
            duration=to_msg_duration(float(self.takeoff_duration_sec)),
        )
        self.takeoff_in_progress = True
        self.call_service(
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

    def send_goto_for_goal(self, msg: PoseStamped):
        """Send goal as absolute go_to target from PoseStamped."""
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = Point(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(self.goal_follow_height),
        )
        req.yaw = 0.0
        req.duration = to_msg_duration(float(self.go_to_duration_sec))

        self.call_service(self.goto_cli, req, "go_to")
        self.get_logger().info(
            f"Sent go_to: x={req.goal.x:.2f} y={req.goal.y:.2f} "
            f"z={req.goal.z:.2f} yaw={req.yaw:.1f}deg"
        )

    def goal_cb(self, msg: PoseStamped):
        """!
        @brief Handle incoming goal poses and send go_to requests.
        @param msg Target pose message.
        """
        if not self.ready:
            self.get_logger().warn("Ignoring goal: no odometry yet")
            return

        if self.use_takeoff and not self.has_taken_off:
            if self.takeoff_in_progress:
                self.get_logger().warn("Ignoring goal: takeoff is still in progress")
            else:
                self.get_logger().warn("Ignoring goal: takeoff has not completed yet")
                self.ensure_takeoff_started()
            return

        try:
            self.send_goto_for_goal(msg)

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
