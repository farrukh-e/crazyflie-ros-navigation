#!/usr/bin/env python3

import math
import os
import sys
from importlib import import_module
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from crazyflie_interfaces.srv import GoTo, Land, Takeoff


def _load_cffirmware(explicit_path: str = ""):
    try:
        return import_module("cffirmware")
    except ModuleNotFoundError:
        pass

    candidates = []
    if explicit_path:
        candidates.append(Path(explicit_path).expanduser())
    env_path = os.environ.get("CFFIRMWARE_PYTHON_PATH")
    if env_path:
        candidates.append(Path(env_path).expanduser())

    roots = [Path.cwd(), *Path(__file__).resolve().parents]
    for root in roots:
        candidates.append(root / "crazyflie-firmware" / "build")
        candidates.append(root.parent / "crazyflie-firmware" / "build")

    seen = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        if (candidate / "cffirmware.py").exists():
            sys.path.insert(0, str(candidate))
            return import_module("cffirmware")

    raise ModuleNotFoundError(
        "Could not import cffirmware. Build it with `make bindings_python` in "
        "crazyflie-firmware, or set CFFIRMWARE_PYTHON_PATH to its build folder."
    )


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _limit_xy(vx: float, vy: float, limit: float) -> Tuple[float, float]:
    speed = math.hypot(vx, vy)
    if speed <= limit or speed <= 0.0:
        return vx, vy
    scale = limit / speed
    return vx * scale, vy * scale


def _yaw_error(goal: float, current: float) -> float:
    err = goal - current
    while err > math.pi:
        err -= 2.0 * math.pi
    while err < -math.pi:
        err += 2.0 * math.pi
    return err


class CrazyflieServiceCompat(Node):
    """Expose Crazyswarm-like services by tracking firmware-planned trajectories."""

    def __init__(self):
        super().__init__("crazyflie_service_compat")

        self.declare_parameter("robot_prefix", "crazyflie")
        self.declare_parameter("cffirmware_path", "")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("max_xy_speed", 0.6)
        self.declare_parameter("max_z_speed", 0.5)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("kp_xy", 1.2)
        self.declare_parameter("kp_z", 1.5)
        self.declare_parameter("kp_yaw", 1.2)
        self.declare_parameter("cmd_vel_frame", "body")
        self.declare_parameter("position_tolerance_xy", 0.05)
        self.declare_parameter("position_tolerance_z", 0.05)
        self.declare_parameter("yaw_tolerance_deg", 5.0)
        self.declare_parameter("default_action_duration_sec", 2.0)
        self.declare_parameter("action_timeout_margin_sec", 5.0)

        self.robot_prefix = str(self.get_parameter("robot_prefix").value).strip("/")
        self.firm = _load_cffirmware(str(self.get_parameter("cffirmware_path").value))
        self.planner = self.firm.planner()
        self.firm.plan_init(self.planner)
        self.max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.cmd_vel_frame = str(self.get_parameter("cmd_vel_frame").value).lower()
        if self.cmd_vel_frame not in ("body", "world"):
            raise ValueError("cmd_vel_frame must be 'body' or 'world'")
        self.pos_tol_xy = float(self.get_parameter("position_tolerance_xy").value)
        self.pos_tol_z = float(self.get_parameter("position_tolerance_z").value)
        self.yaw_tol_deg = float(self.get_parameter("yaw_tolerance_deg").value)
        self.default_action_duration_sec = float(
            self.get_parameter("default_action_duration_sec").value
        )
        self.action_timeout_margin_sec = float(
            self.get_parameter("action_timeout_margin_sec").value
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
        self.goal_yaw_rad: Optional[float] = None
        self.action_deadline = 0.0
        self.hard_deadline = 0.0

        self.get_logger().info(
            f"Firmware-planner service compatibility active on "
            f"/{self.robot_prefix}/takeoff|land|go_to"
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

    def _set_deadlines(self, duration: float):
        now = self._now()
        self.action_deadline = now + duration
        self.hard_deadline = self.action_deadline + max(0.0, self.action_timeout_margin_sec)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_pose = (float(p.x), float(p.y), float(p.z))
        self.current_yaw_rad = yaw

    def _eval_from_current_state(self):
        ev = self.firm.traj_eval_zero()
        px, py, pz = self.current_pose if self.current_pose else (0.0, 0.0, 0.0)
        ev.pos = self.firm.mkvec(px, py, pz)
        ev.vel = self.firm.vzero()
        ev.acc = self.firm.vzero()
        ev.omega = self.firm.vzero()
        ev.yaw = self.current_yaw_rad if self.current_yaw_rad is not None else 0.0
        return ev

    def _current_planner_eval(self):
        if self.mode != "IDLE":
            ev = self.firm.plan_current_goal(self.planner, self._now())
            if self.firm.is_traj_eval_valid(ev):
                return ev
        return self._eval_from_current_state()

    def _plan_takeoff_or_land(self, height: float, duration: float, landing: bool) -> bool:
        if self.current_pose is None:
            return False

        now = self._now()
        curr = self._current_planner_eval()
        target_yaw = curr.yaw
        if landing:
            result = self.firm.plan_land(
                self.planner,
                curr.pos,
                curr.yaw,
                height,
                target_yaw,
                duration,
                now,
            )
        else:
            result = self.firm.plan_takeoff(
                self.planner,
                curr.pos,
                curr.yaw,
                height,
                target_yaw,
                duration,
                now,
            )
        return result == 0

    def _takeoff_cb(self, req: Takeoff.Request, res: Takeoff.Response):
        if self.current_pose is None:
            self.get_logger().warn("Ignoring takeoff: no odometry available yet")
            return res

        self.goal_yaw_rad = None
        duration = self._duration_or_default(req.duration.sec, req.duration.nanosec)
        if self._plan_takeoff_or_land(float(req.height), duration, landing=False):
            curr = self._current_planner_eval()
            self.goal_xyz = (curr.pos.x, curr.pos.y, float(req.height))
            self.mode = "TAKEOFF"
            self._set_deadlines(duration)
        else:
            self.get_logger().warn("Firmware planner rejected takeoff")
        return res

    def _land_cb(self, req: Land.Request, res: Land.Response):
        if self.current_pose is None:
            self.get_logger().warn("Ignoring land: no odometry available yet")
            return res

        self.goal_yaw_rad = None
        duration = self._duration_or_default(req.duration.sec, req.duration.nanosec)
        if self._plan_takeoff_or_land(float(req.height), duration, landing=True):
            curr = self._current_planner_eval()
            self.goal_xyz = (curr.pos.x, curr.pos.y, float(req.height))
            self.mode = "LAND"
            self._set_deadlines(duration)
        else:
            self.get_logger().warn("Firmware planner rejected land")
        return res

    def _go_to_cb(self, req: GoTo.Request, res: GoTo.Response):
        if self.current_pose is None:
            self.get_logger().warn("Ignoring go_to: no odometry available yet")
            return res

        now = self._now()
        curr = self._current_planner_eval()
        goal = self.firm.mkvec(float(req.goal.x), float(req.goal.y), float(req.goal.z))
        duration = self._duration_or_default(req.duration.sec, req.duration.nanosec)
        result = self.firm.plan_go_to_from(
            self.planner,
            curr,
            bool(req.relative),
            False,
            goal,
            float(req.yaw),
            duration,
            now,
        )
        if result != 0:
            self.get_logger().warn("Firmware planner rejected go_to")
            return res

        if req.relative:
            gx = curr.pos.x + float(req.goal.x)
            gy = curr.pos.y + float(req.goal.y)
            gz = curr.pos.z + float(req.goal.z)
            gyaw = curr.yaw + float(req.yaw)
        else:
            gx = float(req.goal.x)
            gy = float(req.goal.y)
            gz = float(req.goal.z)
            gyaw = float(req.yaw)

        self.goal_xyz = (gx, gy, gz)
        self.goal_yaw_rad = gyaw
        self.mode = "GOTO"
        self._set_deadlines(duration)
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
        if self.goal_yaw_rad is None or self.current_yaw_rad is None:
            return 0.0
        return _yaw_error(self.goal_yaw_rad, self.current_yaw_rad)

    def _timer_cb(self):
        if self.mode == "IDLE":
            self._publish_zero()
            return

        if self.current_pose is None:
            self._publish_zero()
            return

        px, py, pz = self.current_pose
        ev = self.firm.plan_current_goal(self.planner, self._now())
        if not self.firm.is_traj_eval_valid(ev):
            self.firm.plan_stop(self.planner)
            self.mode = "IDLE"
            self._publish_zero()
            return

        ex = ev.pos.x - px
        ey = ev.pos.y - py
        ez = ev.pos.z - pz

        if self.current_yaw_rad is None:
            yaw_err = 0.0
        else:
            yaw_err = _yaw_error(ev.yaw, self.current_yaw_rad)

        vx = ev.vel.x + self.kp_xy * ex
        vy = ev.vel.y + self.kp_xy * ey
        vx, vy = _limit_xy(vx, vy, self.max_xy_speed)

        if self.cmd_vel_frame == "body" and self.current_yaw_rad is not None:
            cos_yaw = math.cos(self.current_yaw_rad)
            sin_yaw = math.sin(self.current_yaw_rad)
            vx, vy = (
                cos_yaw * vx + sin_yaw * vy,
                -sin_yaw * vx + cos_yaw * vy,
            )

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = _clamp(ev.vel.z + self.kp_z * ez, self.max_z_speed)
        cmd.angular.z = _clamp(ev.omega.z + self.kp_yaw * yaw_err, self.max_yaw_rate)

        now = self._now()
        pos_done = self._target_reached(px, py, pz)
        yaw_done = abs(math.degrees(self._yaw_error_rad())) <= self.yaw_tol_deg
        finished = now >= self.action_deadline and (
            (self.mode in ("TAKEOFF", "LAND") and pos_done)
            or (pos_done and yaw_done)
        )
        timed_out = now >= self.hard_deadline
        if (
            finished
            or timed_out
        ):
            self.firm.plan_stop(self.planner)
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
