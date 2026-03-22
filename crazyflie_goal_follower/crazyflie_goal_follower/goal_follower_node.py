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

        self.declare_parameter("robot_prefix", "crazyflie")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("use_takeoff", True)
        self.declare_parameter("takeoff_height", 0.5)
        self.declare_parameter("takeoff_duration_sec", 2.0)
        self.declare_parameter("go_to_duration_sec", 5.0)
        self.declare_parameter("goal_follow_height", 0.5)
        self.declare_parameter("wait_for_services_timeout_sec", 10.0)

        self.prefix = self.get_prefix()
        self.goal_topic = self.get_parameter("goal_topic").value
        self.use_takeoff = self.get_parameter("use_takeoff").value
        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.takeoff_duration_sec = self.get_parameter("takeoff_duration_sec").value
        self.go_to_duration_sec = self.get_parameter("go_to_duration_sec").value
        self.goal_follow_height = self.get_parameter("goal_follow_height").value
        self.wait_timeout = self.get_parameter("wait_for_services_timeout_sec").value

        self.ready = False
        self.has_taken_off = False
        self.takeoff_in_progress = False
        self.initial_position = None

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
        if not self.ready:
            p = msg.pose.pose.position
            self.initial_position = (float(p.x), float(p.y), float(p.z))
            self.get_logger().info(
                f"Initial position captured: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}"
            )
            self.ready = True
            if self.use_takeoff:
                self.get_logger().info("Initiating automatic takeoff after odometry became available")
                self.try_takeoff()

    def call(self, client, request, label: str, on_success=None, on_error=None):
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

    def send_takeoff_request(self):
        """!
        @brief Build and send the takeoff service request.
        """
        req = Takeoff.Request(
            group_mask=0,
            height=float(self.takeoff_height),
            duration=to_msg_duration(float(self.takeoff_duration_sec)),
        )
        self.takeoff_in_progress = True
        self.call(
            self.takeoff_cli,
            req,
            "takeoff",
            on_success=self.takeoff_done_cb,
            on_error=self.takeoff_error_cb,
        )
        self.get_logger().info("Takeoff sent before first goal")

    def try_takeoff(self) -> bool:
        """!
        @brief Send one-time automatic takeoff.
        @return True if goal processing can continue immediately, False otherwise.
        """
        if not self.use_takeoff or self.has_taken_off:
            return True
        if self.takeoff_in_progress:
            return False

        self.send_takeoff_request()
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
        """!
        @brief Send goal as absolute go_to target from PoseStamped.
        @param msg Target pose message.
        """
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

        self.call(self.goto_cli, req, "go_to")
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
                self.try_takeoff()
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
