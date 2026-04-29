#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Point, PoseStamped
from builtin_interfaces.msg import Duration as MsgDuration
from nav_msgs.msg import Odometry, OccupancyGrid
from tf_transformations import euler_from_quaternion

from crazyflie_interfaces.srv import Takeoff, GoTo, Land


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

        self.prefix = str(self.declare_and_get("robot_prefix", "crazyflie")).strip("/")
        self.goal_topic = self.declare_and_get("goal_topic", "/goal_pose")
        self.use_takeoff = self.declare_and_get("use_takeoff", False)
        self.takeoff_height = self.declare_and_get("takeoff_height", 0.5)
        self.takeoff_duration_sec = self.declare_and_get("takeoff_duration_sec", 2.0)
        self.go_to_duration_sec = self.declare_and_get("go_to_duration_sec", 1.25)
        self.land_on_shutdown = self.declare_and_get("land_on_shutdown", True)
        self.land_height = self.declare_and_get("land_height", 0.05) #Safety margin
        self.land_duration_sec = self.declare_and_get("land_duration_sec", 2.0)
        self.goal_follow_height = self.declare_and_get("goal_follow_height", 0.5)
        self.wait_timeout = self.declare_and_get("wait_for_services_timeout_sec", 10.0)
        self.use_map_line_check = self.declare_and_get("use_map_line_check", True)
        self.map_topic = self.declare_and_get("map_topic", "/map")
        self.odom_topic = self.declare_and_get("odom_topic", f"/{self.prefix}/odom")
        self.occupancy_block_threshold = self.declare_and_get("occupancy_block_threshold", 65)
        self.treat_unknown_as_obstacle = self.declare_and_get("treat_unknown_as_obstacle", True)

        self.has_taken_off = False
        self.takeoff_in_progress = False
        self.current_position = None
        self.map_msg = None

        self.get_logger().info(f"Using robot prefix: /{self.prefix}")

        # Clients
        self.takeoff_cli = self.create_client(Takeoff, f"/{self.prefix}/takeoff")
        self.goto_cli = self.create_client(GoTo, f"/{self.prefix}/go_to")
        self.land_cli = self.create_client(Land, f"/{self.prefix}/land")

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        if self.use_map_line_check:
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)
        self.odom_log_timer = self.create_timer(1.0, self.log_current_odom)

        self.wait_for_services()
        self.get_logger().info(
            f"Goal follower ready. Listening on {self.goal_topic}"
            + (f", map checks on {self.map_topic}" if self.use_map_line_check else "")
        )
        self.get_logger().info(
            f"Odometry topic configured: {self.odom_topic} (used for map line-check when available)"
        )

    def declare_and_get(self, name: str, default):
        """! @brief Declare a parameter and return its value."""
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
        if self.land_on_shutdown:
            required.append((self.land_cli, "land"))

        for client, name in required:
            if not client.wait_for_service(timeout_sec=self.wait_timeout):
                raise RuntimeError(
                    f"Service not available within {self.wait_timeout}s: /{self.prefix}/{name}"
                )

    def odom_cb(self, msg: Odometry):
        """!
        @brief Odometry callback used to track current position.
        @param msg Incoming odometry message.
        """
        p = msg.pose.pose.position
        self.update_current_position(float(p.x), float(p.y), float(p.z))

    def update_current_position(self, x: float, y: float, z: float):
        """! @brief Store current position and trigger readiness-dependent startup once."""
        was_none = self.current_position is None
        self.current_position = (x, y, z)

        if was_none:
            self.get_logger().info(
                f"Initial odometry position captured: x={x:.2f}, y={y:.2f}, z={z:.2f}"
            )

    def map_cb(self, msg: OccupancyGrid):
        """! @brief Cache the latest static occupancy map used for line-of-sight checks."""
        self.map_msg = msg

    def log_current_odom(self):
        """! @brief Print the latest odometry position."""
        if self.current_position is None:
            return
        x, y, z = self.current_position
        self.get_logger().info(
            f"Current odometry position: x={x:.2f}, y={y:.2f}, z={z:.2f}"
        )

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

    def ensure_land_started(self):
        """! @brief Ensure that the shutdown landing process has been initiated."""
        if not self.land_cli.service_is_ready():
            self.get_logger().warn(f"Skipping shutdown landing: /{self.prefix}/land not ready")
            return

        req = Land.Request(
            group_mask=0,
            height=float(self.land_height),
            duration=to_msg_duration(float(self.land_duration_sec)),
        )
        self.land_cli.call_async(req)
        self.get_logger().info(
            f"Land sent during shutdown (height={self.land_height:.2f}, "
            f"duration={self.land_duration_sec:.2f}s)"
        )

    def send_goto_for_goal(self, msg: PoseStamped):
        """! @brief Send goal as absolute go_to target from PoseStamped."""
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = Point(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(self.goal_follow_height),
        )
        _, _, yaw = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])
        req.yaw = float(yaw)
        req.duration = to_msg_duration(float(self.go_to_duration_sec))

        self.call_service(self.goto_cli, req, "go_to")
        self.get_logger().info(
            f"Sent go_to: x={req.goal.x:.2f} y={req.goal.y:.2f} "
            f"z={req.goal.z:.2f} yaw={req.yaw:.1f}deg"
        )

    def land_before_shutdown(self):
        """! @brief Best-effort landing call used when Ctrl+C stops the node."""
        if not self.land_on_shutdown:
            self.get_logger().info("Shutdown landing is disabled (land_on_shutdown=false)")
            return

        self.get_logger().info("Ctrl+C received: initiating landing")
        self.ensure_land_started()

    @staticmethod
    def bresenham_line(x0: int, y0: int, x1: int, y1: int):
        """! @brief Returns a list of (x, y) coordinates from (x0, y0) to (x1, y1)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def world_to_map_cell(self, x: float, y: float):
        """! @brief Convert world coordinates into map cell indices."""
        if self.map_msg is None:
            self.get_logger().warn("Ignoring goal: no map received yet on map topic")
            return None
        mx = math.floor((x - float(self.map_msg.info.origin.position.x)) / float(self.map_msg.info.resolution))
        my = math.floor((y - float(self.map_msg.info.origin.position.y)) / float(self.map_msg.info.resolution))
        if mx < 0 or my < 0 or mx >= int(self.map_msg.info.width) or my >= int(self.map_msg.info.height):
            return None
        return mx, my

    def cell_is_blocked(self, occupancy_value: int) -> bool:
        """! @brief Check if a map cell is blocked."""
        if occupancy_value < 0:
            return bool(self.treat_unknown_as_obstacle)
        return occupancy_value >= int(self.occupancy_block_threshold)

    def path_is_clear(self, goal_x: float, goal_y: float) -> bool:
        if self.map_msg is None:
            self.get_logger().warn("No map yet; skipping map line-check for this goal")
            return True
        if self.current_position is None:
            self.get_logger().warn("No odometry yet; skipping map line-check for this goal")
            return True

        start = self.world_to_map_cell(self.current_position[0], self.current_position[1])
        goal = self.world_to_map_cell(goal_x, goal_y)
        if start is None or goal is None:
            self.get_logger().warn("Ignoring goal: start or goal is outside static map bounds")
            return False

        width = int(self.map_msg.info.width)
        for mx, my in self.bresenham_line(start[0], start[1], goal[0], goal[1]):
            occ = self.map_msg.data[my * width + mx]
            if self.cell_is_blocked(occ):
                self.get_logger().warn(
                    f"Ignoring goal: blocked map cell ({mx}, {my}) occupancy={occ}"
                )
                return False
        return True

    def goal_cb(self, msg: PoseStamped):
        """!
        @brief Handle incoming goal poses and send go_to requests.
        @param msg Target pose message.
        """
        if self.use_takeoff and not self.has_taken_off:
            if self.takeoff_in_progress:
                self.get_logger().warn("Ignoring goal: takeoff is still in progress")
            else:
                self.get_logger().warn("Ignoring goal: takeoff has not completed yet")
                self.ensure_takeoff_started()
            return

        try:
            if self.use_map_line_check:
                goal_x = float(msg.pose.position.x)
                goal_y = float(msg.pose.position.y)
                if not self.path_is_clear(goal_x, goal_y):
                    return
            self.send_goto_for_goal(msg)

        except Exception as exc:
            self.get_logger().error(f"Goal handling failed: {exc}")


def main():
    # Keep ROS context alive on Ctrl+C long enough to send the shutdown land request.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GoalFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.land_before_shutdown()
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
