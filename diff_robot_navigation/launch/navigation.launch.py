#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
navigation.launch.py
=====================
完整自主导航闭环：Gazebo 仿真 + map_server(加载建图阶段保存的地图) + AMCL 定位 +
我们自己的 A* global_planner_node（顶替 planner_server）+ Nav2 controller_server(DWB
局部避障) + bt_navigator + lifecycle_manager + RViz(nav.rviz，带 Nav2 Goal 工具，
可以直接在 RViz 里点 "2D Nav Goal" 试動态起点/终点)。

对应需求1 "从地图构建、全局路径规划、局部避障到轨迹跟踪的完整自主导航闭环" 里
"全局路径规划 + 局部避障" 这一段；轨迹跟踪的独立对比实验见
compare_controllers.launch.py（需求3 Pure Pursuit vs MPC）。

用法：
    ros2 launch diff_robot_navigation navigation.launch.py map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('diff_robot_description')
    nav_share = get_package_share_directory('diff_robot_navigation')

    default_map = os.path.join(nav_share, 'maps', 'slam_test_world.yaml')
    map_arg = DeclareLaunchArgument('map', default_value=default_map,
                                     description='建图阶段用 map_saver_cli 保存的 .yaml 地图路径')
    params_arg = DeclareLaunchArgument(
        'nav2_params', default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'))

    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'rviz_config': os.path.join(desc_share, 'rviz', 'nav.rviz'),
        }.items(),
    )

    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[nav2_params, {'yaml_filename': map_yaml}],
    )
    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen', parameters=[nav2_params],
    )
    controller_server_node = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', parameters=[nav2_params],
    )
    bt_navigator_node = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[nav2_params],
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[nav2_params],
    )

    # 2026-09-07 修复：lifecycle_manager 的 autostart 会在"进程一起来"就立刻
    # 尝试 configure→activate 所有节点，但这时候 Gazebo 世界可能还没加载完、
    # 机器人实体还没 spawn 出来（gazebo.launch.py 里的 create 节点是异步的，
    # 实测要 8 秒左右才完成）、ekf_node 也还在等 /clock 第一帧，odom→base_link
    # 这条 TF 根本还不存在。controller_server 的 local_costmap 激活时会主动去
    # 等 base_link→odom 的变换，等不到就直接判定激活失败，lifecycle_manager
    # 会整体 abort bringup（不会自动重试，四个节点卡死在
    # inactive/unconfigured，需要重新整个 launch 或手动调用
    # /lifecycle_manager_navigation/manage_nodes 服务才能重新拉起）。
    # 用 TimerAction 把 lifecycle_manager 的启动延后，等 Gazebo/机器人生成/
    # EKF 都基本就绪之后再让它去做 configure/activate，是最简单可靠的修法
    # （比去改 costmap 的等待超时参数更稳妥，行为也更可预期）。
    delayed_lifecycle_manager = TimerAction(period=10.0, actions=[lifecycle_manager_node])

    # 我们自己的 A* 全局规划节点 —— 顶替 nav2_planner_server，
    # 是普通 rclpy 节点，不归 lifecycle_manager 管（详见 nav2_params.yaml 里的说明）。
    global_planner_node = Node(
        package='diff_robot_navigation', executable='global_planner_node',
        name='planner_server', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        gazebo_launch,
        map_server_node,
        amcl_node,
        controller_server_node,
        bt_navigator_node,
        delayed_lifecycle_manager,
        global_planner_node,
    ])
