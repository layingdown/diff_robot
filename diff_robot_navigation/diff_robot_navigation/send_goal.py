#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_goal.py
============
命令行小工具：向 global_planner_node 提供的 compute_path_to_pose action 发一个目标，
拿到规划出来的路径后发布到 /plan（Pure Pursuit / MPC 控制器节点订阅的话题）。

用于 compare_controllers.launch.py 里"只做轨迹跟踪对比、不跑完整 BT 导航"的场景：
跳过 bt_navigator，直接调用规划 + 把结果喂给控制器节点，专注对比 Pure Pursuit 和 MPC
本身的跟踪表现（对应需求3），而不引入 Nav2 行为树/恢复行为的额外变量。

用法：
    ros2 run diff_robot_navigation send_goal --ros-args \
        -p goal_x:=3.0 -p goal_y:=2.5 -p use_start:=false
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class SendGoalNode(Node):

    def __init__(self):
        super().__init__('send_goal')
        self.declare_parameter('goal_x', 6.0)
        self.declare_parameter('goal_y', 6.0)
        self.declare_parameter('use_start', False)
        self.declare_parameter('start_x', 0.4)
        self.declare_parameter('start_y', 0.4)
        self.declare_parameter('frame', 'map')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._path_pub = self.create_publisher(Path, '/plan', qos)
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('compute_path_to_pose action server 未就绪，退出')
            return False

        goal = ComputePathToPose.Goal()
        frame = self.get_parameter('frame').value
        goal.goal.header.frame_id = frame
        goal.goal.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.goal.pose.position.y = float(self.get_parameter('goal_y').value)
        goal.goal.pose.orientation.w = 1.0

        use_start = bool(self.get_parameter('use_start').value)
        goal.use_start = use_start
        if use_start:
            start = PoseStamped()
            start.header.frame_id = frame
            start.pose.position.x = float(self.get_parameter('start_x').value)
            start.pose.position.y = float(self.get_parameter('start_y').value)
            start.pose.orientation.w = 1.0
            goal.start = start

        self.get_logger().info(
            f'发送规划请求: goal=({goal.goal.pose.position.x:.2f}, '
            f'{goal.goal.pose.position.y:.2f}) use_start={use_start}')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('规划请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if len(result.path.poses) == 0:
            self.get_logger().error('规划失败，返回空路径')
            return False

        self.get_logger().info(
            f'规划成功: {len(result.path.poses)} 个点，耗时 '
            f'{result.planning_time.sec + result.planning_time.nanosec / 1e9:.3f}s，'
            f'发布到 /plan')
        for _ in range(3):
            self._path_pub.publish(result.path)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SendGoalNode()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
