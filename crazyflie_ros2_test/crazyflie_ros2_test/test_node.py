#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RosDuration

from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration as MsgDuration
from nav_msgs.msg import Odometry

from crazyflie_interfaces.srv import Takeoff, Land, GoTo


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


class UnicastGoTo(Node):

    TAKEOFF_Z = 0.5
    GOTO_TIME = 3.0
    GOTO_WAIT = 3.5

    def __init__(self):
        """!
        @brief Initialize node parameters, service clients, and odometry subscription.
        """
        super().__init__('unicas_goto')

        self.prefix = self.get_prefix()
        self.get_logger().info(f"Using robot prefix: /{self.prefix}")

        # Clients
        self.takeoff_cli = self.create_client(Takeoff, f"/{self.prefix}/takeoff")
        self.land_cli = self.create_client(Land, f"/{self.prefix}/land")
        self.goto_cli = self.create_client(GoTo, f"/{self.prefix}/go_to")

        # Odom
        self.initial_position = None
        self.odom_sub = self.create_subscription(Odometry, f"/{self.prefix}/odom", self.odom_cb, 10)

        self.wait_for_services()

    def get_prefix(self) -> str:
        """!
        @brief Read and sanitize the robot namespace prefix parameter.
        @return Robot prefix without leading/trailing slashes.
        """
        self.declare_parameter("robot_prefix", "crazyflie")
        prefix = str(self.get_parameter("robot_prefix").value).strip("/")
        return prefix or "crazyflie"

    def wait_for_services(self, timeout_sec: float = 10.0):
        """!
        @brief Wait for takeoff, land, and go_to services to become available.
        @param timeout_sec Timeout per service in seconds.
        @throws RuntimeError If any service is unavailable within timeout.
        """
        for client, name in [
            (self.takeoff_cli, "takeoff"),
            (self.land_cli, "land"),
            (self.goto_cli, "go_to"),
        ]:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"Service not available within {timeout_sec}s: /{self.prefix}/{name}")

    def odom_cb(self, msg: Odometry):
        """!
        @brief Odometry callback that captures the first pose as initial position.
        @param msg Incoming odometry message.
        """
        if self.initial_position is None:
            p = msg.pose.pose.position
            self.initial_position = (float(p.x), float(p.y), float(p.z))
            self.get_logger().info(
                f"Initial position captured: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )

    def wait_for_initial_position(self, timeout_sec: float = 10.0):
        """!
        @brief Block until initial position is available from odometry.
        @param timeout_sec Maximum wait time in seconds.
        @throws RuntimeError If no odometry is received before timeout and lands the drone.
        """
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.initial_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds / 1e9 >= deadline:
                break

        if self.initial_position is None:
            req = Land.Request(group_mask=0, height=0.00, duration=to_msg_duration(1.0))
            self.call(self.land_cli, req, "land")
            raise RuntimeError(f"No odometry received on /{self.prefix}/odom within {timeout_sec}s")

    def call(self, client, request, label: str):
        """!
        @brief Call a ROS service asynchronously and wait for completion.
        @param client Service client instance.
        @param request Service request object.
        @param label Label used in error messages.
        @return Service response object.
        @throws RuntimeError If the service call fails.
        """
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f"{label} failed: {future.exception()}")
        return future.result()

    def goto_req(self, x: float, y: float, z: float, yaw: float = 0.0, duration: float = GOTO_TIME):
        """!
        @brief Send an absolute go_to request and wait for completion window.
        @param x Goal x coordinate in meters.
        @param y Goal y coordinate in meters.
        @param z Goal z coordinate in meters.
        @param yaw Goal yaw in degrees.
        @param duration Requested trajectory duration in seconds.
        """
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = Point(x=float(x), y=float(y), z=float(z))
        req.yaw = float(yaw)
        req.duration = to_msg_duration(duration)
        self.call(self.goto_cli, req, "go_to")
        self.get_clock().sleep_for(RosDuration(seconds=duration))

    def run(self):
        """!
        @brief Execute takeoff, square trajectory, and landing sequence.
        @details Square waypoints are computed relative to initial odometry pose.
        """
        z = self.TAKEOFF_Z
        square_rel = [
            (0.0, 0.0, z),
            (0.5, 0.0, z),
            (0.5, 0.5, z),
            (0.0, 0.5, z),
            (0.0, 0.0, z),
        ]

        self.wait_for_initial_position()
        init_x, init_y, init_z = self.initial_position

        req = Takeoff.Request(group_mask=0, height=float(z), duration=to_msg_duration(1.0))
        self.call(self.takeoff_cli, req, "takeoff")
        self.get_clock().sleep_for(RosDuration(seconds=1.5))

        # TODO: Add odometry feedback check to confirm goal is reached within a tolerance
        for rx, ry, rz in square_rel:
            self.goto_req(init_x + rx, init_y + ry, init_z + rz)

        req = Land.Request(group_mask=0, height=0.00, duration=to_msg_duration(1.0))
        self.call(self.land_cli, req, "land")
        self.get_clock().sleep_for(RosDuration(seconds=1.5))

def main():
    rclpy.init()
    node = UnicastGoTo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
