#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pure_pursuit_controller.py
============================
ROS2 节点封装：把 control_models.PurePursuitController 接到真实话题上。

设计成一个独立的"轨迹跟踪"节点，而不是塞进 Nav2 controller_server 的
FollowPath 插件接口，原因：
  - 需求3明确要求"实现 Pure Pursuit 轨迹跟踪控制器，并对比 MPC 方案"，
    这是一组可以独立对比、独立调参的实验对象，不依赖 Nav2 controller_server
    的 costmap/恢复行为树等重型基础设施，跑起来更直接，也方便和
    scripts/tune_and_compare.py 用同一套 control_models.py 代码对表。
  - 日常"跟着 A* 全局路径 + Nav2 DWB 局部避障"走完整闭环时，用的是
    controller_server 里的 DWB（对应需求2）；本节点是"研究/对比模式"：
    直接订阅 /plan，不经过 Nav2 的 FollowPath action，用于单独测量
    Pure Pursuit 的跟踪表现，和 mpc_controller.py 二选一启动做 A/B 对比。

话题：
    订阅 /plan   (nav_msgs/Path)      —— 来自 global_planner_node 或 RViz 保存的路径
    订阅 /odom   (nav_msgs/Odometry)  —— 里程计（EKF 融合后的 /odometry/filtered 也可以）
    发布 /cmd_vel (geometry_msgs/Twist)
参数见 config/pure_pursuit_params.yaml，逐个对应 control_models.PurePursuitParams。
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool

from diff_robot_navigation.control_models import PurePursuitController, PurePursuitParams
from diff_robot_navigation.ros_utils import yaw_from_quaternion


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('min_lookahead', 0.3)
        self.declare_parameter('max_lookahead', 1.5)
        self.declare_parameter('cruise_linear_vel', 0.5)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.8)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('curvature_slowdown_gain', 1.4)
        self.declare_parameter('decel_radius', 0.6)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_topic', '/odom')

        self._controller: PurePursuitController | None = None
        self._odom_state = None  # np.array([x,y,theta])
        self._active = False

        self.create_subscription(Path, '/plan', self._on_path, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                  self._on_odom, 20)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._reached_pub = self.create_publisher(Bool, '/pure_pursuit/goal_reached', 1)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info('Pure Pursuit controller 节点已启动，等待 /plan 与 /odom ...')

    def _params_from_ros(self) -> PurePursuitParams:
        g = self.get_parameter
        return PurePursuitParams(
            lookahead_distance=float(g('lookahead_distance').value),
            min_lookahead=float(g('min_lookahead').value),
            max_lookahead=float(g('max_lookahead').value),
            cruise_linear_vel=float(g('cruise_linear_vel').value),
            max_linear_vel=float(g('max_linear_vel').value),
            max_angular_vel=float(g('max_angular_vel').value),
            goal_tolerance=float(g('goal_tolerance').value),
            curvature_slowdown_gain=float(g('curvature_slowdown_gain').value),
            decel_radius=float(g('decel_radius').value),
        )

    def _on_path(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn('收到的 /plan 少于 2 个点，忽略')
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._controller = PurePursuitController(pts, self._params_from_ros())
        self._active = True
        self.get_logger().info(f'收到新路径，{len(pts)} 个点，开始跟踪')

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

        v, omega = self._controller.compute_control(self._odom_state)
        self._publish_cmd(v, omega)

    def _publish_cmd(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
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
