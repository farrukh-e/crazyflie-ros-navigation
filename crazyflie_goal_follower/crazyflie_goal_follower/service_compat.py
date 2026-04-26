#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from crazyflie_interfaces.srv import GoTo, Land, Takeoff


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class CrazyflieServiceCompat(Node):
    """Expose Crazyswarm-like services by commanding /<prefix>/cmd_vel."""

    def __init__(self):
        super().__init__("crazyflie_service_compat")

        self.declare_parameter("robot_prefix", "crazyflie")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("max_xy_speed", 0.6)
        self.declare_parameter("max_z_speed", 0.5)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("kp_xy", 1.2)
        self.declare_parameter("kp_z", 1.5)
        self.declare_parameter("kp_yaw", 1.2)
        self.declare_parameter("position_tolerance_xy", 0.05)
        self.declare_parameter("position_tolerance_z", 0.05)
        self.declare_parameter("yaw_tolerance_deg", 5.0)
        self.declare_parameter("default_action_duration_sec", 2.0)

        self.robot_prefix = str(self.get_parameter("robot_prefix").value).strip("/")
        self.max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.pos_tol_xy = float(self.get_parameter("position_tolerance_xy").value)
        self.pos_tol_z = float(self.get_parameter("position_tolerance_z").value)
        self.yaw_tol_deg = float(self.get_parameter("yaw_tolerance_deg").value)
        self.default_action_duration_sec = float(
            self.get_parameter("default_action_duration_sec").value
        )

        cmd_vel_topic = self._topic("cmd_vel")
        odom_topic = self._topic("odom")

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self.takeoff_srv = self.create_service(
            Takeoff, self._topic("takeoff"), self._takeoff_cb
        )
        self.land_srv = self.create_service(Land, self._topic("land"), self._land_cb)
        self.go_to_srv = self.create_service(GoTo, self._topic("go_to"), self._go_to_cb)
        # Alias for clients that use '/goto' instead of '/go_to'.
        self.goto_alias_srv = self.create_service(GoTo, self._topic("goto"), self._go_to_cb)

        rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.current_yaw_rad: Optional[float] = None
        self.mode = "IDLE"
        self.goal_xyz = (0.0, 0.0, 0.0)
        self.goal_yaw_deg: Optional[float] = None
        self.action_deadline = 0.0

        self.get_logger().info(
            f"Service compatibility active on /{self.robot_prefix}/takeoff|land|go_to"
        )

    def _topic(self, suffix: str) -> str:
        return f"/{self.robot_prefix}/{suffix}"

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _duration_or_default(self, sec: int, nanosec: int) -> float:
        req_duration = float(sec) + float(nanosec) * 1e-9
        if req_duration <= 0.0:
            return self.default_action_duration_sec
        return req_duration

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_pose = (float(p.x), float(p.y), float(p.z))
        self.current_yaw_rad = yaw

    def _takeoff_cb(self, req: Takeoff.Request, res: Takeoff.Response):
        if self.current_pose is None:
            self.get_logger().warn("No odometry yet; delaying takeoff until odom is available")
        x = self.current_pose[0] if self.current_pose else 0.0
        y = self.current_pose[1] if self.current_pose else 0.0
        self.goal_xyz = (x, y, float(req.height))
        self.goal_yaw_deg = None
        self.mode = "TAKEOFF"
        duration = self._duration_or_default(req.duration.sec, req.duration.nanosec)
        self.action_deadline = self._now() + duration
        return res

    def _land_cb(self, req: Land.Request, res: Land.Response):
        if self.current_pose is None:
            self.get_logger().warn("No odometry yet; delaying land until odom is available")
        x = self.current_pose[0] if self.current_pose else 0.0
        y = self.current_pose[1] if self.current_pose else 0.0
        self.goal_xyz = (x, y, float(req.height))
        self.goal_yaw_deg = None
        self.mode = "LAND"
        duration = self._duration_or_default(req.duration.sec, req.duration.nanosec)
        self.action_deadline = self._now() + duration
        return res

    def _go_to_cb(self, req: GoTo.Request, res: GoTo.Response):
        if self.current_pose is None:
            self.get_logger().warn("Ignoring go_to: no odometry available yet")
            return res

        cx, cy, cz = self.current_pose
        if req.relative:
            gx = cx + float(req.goal.x)
            gy = cy + float(req.goal.y)
            gz = cz + float(req.goal.z)
        else:
            gx = float(req.goal.x)
            gy = float(req.goal.y)
            gz = float(req.goal.z)

        self.goal_xyz = (gx, gy, gz)
        self.goal_yaw_deg = float(req.yaw)
        self.mode = "GOTO"
        duration = self._duration_or_default(req.duration.sec, req.duration.nanosec)
        self.action_deadline = self._now() + duration
        return res

    def _publish_zero(self):
        self.cmd_vel_pub.publish(Twist())

    def _target_reached(self, px: float, py: float, pz: float) -> bool:
        gx, gy, gz = self.goal_xyz
        return (
            abs(gx - px) <= self.pos_tol_xy
            and abs(gy - py) <= self.pos_tol_xy
            and abs(gz - pz) <= self.pos_tol_z
        )

    def _yaw_error_rad(self) -> float:
        if self.goal_yaw_deg is None or self.current_yaw_rad is None:
            return 0.0
        target = math.radians(self.goal_yaw_deg)
        err = target - self.current_yaw_rad
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi
        return err

    def _timer_cb(self):
        if self.mode == "IDLE":
            self._publish_zero()
            return

        if self.current_pose is None:
            self._publish_zero()
            return

        px, py, pz = self.current_pose
        gx, gy, gz = self.goal_xyz
        ex = gx - px
        ey = gy - py
        ez = gz - pz

        yaw_err = self._yaw_error_rad()
        yaw_done = abs(math.degrees(yaw_err)) <= self.yaw_tol_deg

        cmd = Twist()
        cmd.linear.x = _clamp(self.kp_xy * ex, self.max_xy_speed)
        cmd.linear.y = _clamp(self.kp_xy * ey, self.max_xy_speed)
        cmd.linear.z = _clamp(self.kp_z * ez, self.max_z_speed)
        cmd.angular.z = _clamp(self.kp_yaw * yaw_err, self.max_yaw_rate)

        now = self._now()
        pos_done = self._target_reached(px, py, pz)
        timed_out = now >= self.action_deadline
        if (
            (self.mode in ("TAKEOFF", "LAND") and pos_done)
            or (pos_done and yaw_done)
            or timed_out
        ):
            self.mode = "IDLE"
            self._publish_zero()
            return

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieServiceCompat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
