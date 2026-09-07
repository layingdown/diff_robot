#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mpc_controller.py
==================
ROS2 节点封装：把 control_models.LinearMPCController 接到真实话题上。
话题接口、参数风格、节点生命周期都和 pure_pursuit_controller.py 完全对称
（同样订阅 /plan + /odom，发布 /cmd_vel），这样两个节点可以在同一套
launch/bringup 里二选一启动，做 Pure Pursuit vs MPC 的 A/B 对比（需求3）。

MPC 是 LTV（时变线性化）+ 批量凝聚 QP，每个控制周期都要重新构造/求解，
求解耗时会打到 ~diff_robot_navigation/mpc_controller/solve_time_ms 话题，
方便和 scripts/tune_and_compare.py 里离线测的耗时做交叉验证
（要求"求解 < 15ms"，见需求描述里的 solve <15ms 指标）。
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from diff_robot_navigation.control_models import LinearMPCController, MPCParams
from diff_robot_navigation.ros_utils import yaw_from_quaternion


class MPCControllerNode(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        self.declare_parameter('horizon', 12)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('cruise_linear_vel', 0.5)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.8)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('decel_radius', 0.6)
        self.declare_parameter('q_xy', 8.0)
        self.declare_parameter('q_theta', 2.0)
        self.declare_parameter('r_v', 0.8)
        self.declare_parameter('r_omega', 0.6)
        self.declare_parameter('r_dv', 1.2)
        self.declare_parameter('r_domega', 1.4)
        self.declare_parameter('max_iters', 60)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('odom_topic', '/odom')

        self._controller: LinearMPCController | None = None
        self._odom_state = None
        self._active = False

        self.create_subscription(Path, '/plan', self._on_path, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                  self._on_odom, 20)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._reached_pub = self.create_publisher(Bool, '/mpc_controller/goal_reached', 1)
        self._solve_time_pub = self.create_publisher(Float64, '/mpc_controller/solve_time_ms', 1)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            'MPC controller 节点已启动（LTV-MPC，批量 QP + L-BFGS-B），等待 /plan 与 /odom ...')

    def _params_from_ros(self) -> MPCParams:
        g = self.get_parameter
        return MPCParams(
            horizon=int(g('horizon').value),
            dt=float(g('dt').value),
            cruise_linear_vel=float(g('cruise_linear_vel').value),
            max_linear_vel=float(g('max_linear_vel').value),
            max_angular_vel=float(g('max_angular_vel').value),
            goal_tolerance=float(g('goal_tolerance').value),
            decel_radius=float(g('decel_radius').value),
            q_xy=float(g('q_xy').value),
            q_theta=float(g('q_theta').value),
            r_v=float(g('r_v').value),
            r_omega=float(g('r_omega').value),
            r_dv=float(g('r_dv').value),
            r_domega=float(g('r_domega').value),
            max_iters=int(g('max_iters').value),
        )

    def _on_path(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn('收到的 /plan 少于 2 个点，忽略')
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._controller = LinearMPCController(pts, self._params_from_ros())
        self._active = True
        self.get_logger().info(f'收到新路径，{len(pts)} 个点，开始 MPC 跟踪')

    def _on_odom(self, msg: Odometry):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_state = np.array([
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def _on_timer(self):
        if not self._active or self._controller is None or self._odom_state is None:
            return

        if self._controller.is_goal_reached(self._odom_state):
            self._publish_cmd(0.0, 0.0)
            self._active = False
            self._reached_pub.publish(Bool(data=True))
            self.get_logger().info('到达终点，停止')
            return

        t0 = time.perf_counter()
        v, omega = self._controller.compute_control(self._odom_state)
        solve_ms = (time.perf_counter() - t0) * 1000.0
        self._solve_time_pub.publish(Float64(data=solve_ms))
        if solve_ms > 15.0:
            self.get_logger().warn(
                f'MPC 求解耗时 {solve_ms:.2f} ms 超过 15ms 预算，考虑减小 horizon', throttle_duration_sec=2.0)

        self._publish_cmd(v, omega)

    def _publish_cmd(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
