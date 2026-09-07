#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
global_planner_node.py
=======================
把 astar.py 的纯算法包装成一个 ROS2 action server，节点名/动作名与
Nav2 stock 的 planner_server 完全一致（node: planner_server，
action: compute_path_to_pose，类型 nav2_msgs/action/ComputePathToPose）。

这样做的目的：不需要用 C++ pluginlib 把 A* 塞进 nav2_core::GlobalPlanner
接口（Python 插件 nav2 官方不支持），而是让这个节点直接顶替 nav2_bringup
里的 planner_server —— bt_navigator 的行为树里 "ComputePathToPose" 节点
调用的就是这个同名同类型的 action，对 BT XML / bt_navigator 完全透明。
局部避障仍然用 Nav2 自带的 controller_server（DWB 插件），
不需要自己重新实现一遍 DWA（对应需求2 "结合 Nav2/DWA 局部规划器"）。

真正在 Nav2 生产环境里更规范的做法是把 astar_core 做成 C++ pluginlib 插件
（见 diff_robot_planner_cpp），这里的 Python 版本本身也是需求2明确要求的
"基于 Python 实现 A* 全局路径规划算法"，并作为 C++ 版本的正确性基准。
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from diff_robot_navigation.astar import GridMap, PlannerError, plan as astar_plan


class GlobalPlannerNode(Node):

    def __init__(self):
        super().__init__('planner_server')

        self.declare_parameter('planner_frame', 'map')
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('inflation_radius_m', 0.35)
        self.declare_parameter('inflation_weight', 6.0)
        # 注意：不要在这里手动 declare_parameter('use_sim_time', ...)。
        # rclpy.node.Node 的构造函数（上面的 super().__init__(...)）已经自动
        # 声明过 use_sim_time 这个标准参数了（每个节点都有，用来支持仿真时钟），
        # 这里如果重复 declare 会直接抛 ParameterAlreadyDeclaredException 把节点
        # 启动崩掉。launch 文件里传的 {'use_sim_time': True} 会通过 --params-file
        # 自动生效，不需要也不能再手动声明一次。

        self._frame = self.get_parameter('planner_frame').value
        self._allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self._inflation_radius = float(self.get_parameter('inflation_radius_m').value)
        self._inflation_weight = float(self.get_parameter('inflation_weight').value)

        self._map_msg: OccupancyGrid | None = None
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        self._path_pub = self.create_publisher(Path, 'plan', 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            'A* planner_server 就绪：监听 /map，提供 compute_path_to_pose action '
            '（可直接被 Nav2 bt_navigator 当作 planner_server 使用）')

    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid):
        self._map_msg = msg

    def _goal_cb(self, goal_request):
        if self._map_msg is None:
            self.get_logger().warn('尚未收到 /map，拒绝本次规划请求')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _grid_from_map_msg(self) -> GridMap:
        msg = self._map_msg
        data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        return GridMap(
            data=data,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            inflation_radius_m=self._inflation_radius,
            inflation_weight=self._inflation_weight,
        )

    def _lookup_current_pose(self) -> PoseStamped:
        try:
            t = self._tf_buffer.lookup_transform(
                self._frame, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            raise RuntimeError(f'无法获取 {self._frame}->base_link 的 TF: {e}')
        p = PoseStamped()
        p.header.frame_id = self._frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.orientation = t.transform.rotation
        return p

    # ------------------------------------------------------------------
    async def _execute(self, goal_handle):
        goal_msg = goal_handle.request
        result = ComputePathToPose.Result()

        use_start = bool(getattr(goal_msg, 'use_start', False))
        start_pose = goal_msg.start if use_start else None
        if start_pose is None:
            try:
                start_pose = self._lookup_current_pose()
            except RuntimeError as e:
                self.get_logger().error(str(e))
                goal_handle.abort()
                return result

        goal_pose = goal_msg.goal

        t0 = time.perf_counter()
        try:
            grid = self._grid_from_map_msg()
            start_xy = (start_pose.pose.position.x, start_pose.pose.position.y)
            goal_xy = (goal_pose.pose.position.x, goal_pose.pose.position.y)
            points = astar_plan(grid, start_xy, goal_xy, allow_unknown=self._allow_unknown)
        except PlannerError as e:
            self.get_logger().warn(f'A* 规划失败: {e}')
            goal_handle.abort()
            return result

        dt = time.perf_counter() - t0

        path = Path()
        path.header.frame_id = self._frame
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            if i + 1 < len(points):
                nx, ny = points[i + 1]
                yaw = np.arctan2(ny - y, nx - x)
            elif len(points) >= 2:
                yaw = np.arctan2(y - points[-2][1], x - points[-2][0])
            else:
                yaw = 0.0
            ps.pose.orientation.z = np.sin(yaw / 2.0)
            ps.pose.orientation.w = np.cos(yaw / 2.0)
            path.poses.append(ps)

        self._path_pub.publish(path)
        result.path = path
        result.planning_time = Duration(seconds=dt).to_msg()
        goal_handle.succeed()
        self.get_logger().info(
            f'A* 规划成功: {len(points)} 个路径点, 耗时 {dt * 1000:.2f} ms')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
