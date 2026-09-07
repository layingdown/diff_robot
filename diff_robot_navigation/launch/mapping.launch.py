#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mapping.launch.py
==================
建图阶段：Gazebo 仿真 + slam_toolbox（在线异步建图模式）+ RViz。

用法：
    ros2 launch diff_robot_navigation mapping.launch.py
建完图之后另开一个终端保存地图：
    ros2 run nav2_map_server map_saver_cli -f ~/maps/diff_robot_map
（详见 docs/RUN_STEPS.md 第2步）

注意：这个 ROS2/slam_toolbox 版本里，async_slam_toolbox_node 是一个
lifecycle 节点，进程启动后默认停在 unconfigured 状态——不会自动订阅
/scan，也不会发布 /map，必须显式做 configure -> activate 两次状态
转换才会真正开始工作（表现为：节点活着、colcon/launch都不报错，但
/scan 一直没人订阅、/map 永远不出现、map_saver_cli 一直等不到消息
超时）。这里加了 nav2_lifecycle_manager 自动帮它做这两次转换
（autostart: true），不用每次都手动敲：
    ros2 lifecycle set /slam_toolbox configure
    ros2 lifecycle set /slam_toolbox activate
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('diff_robot_description')
    slam_params = os.path.join(desc_share, 'config', 'slam_toolbox_params.yaml')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py'))
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    # slam_toolbox 是 lifecycle 节点，光启动进程不会自动开始工作，
    # 需要有人帮它做 configure -> activate。用 nav2 自带的
    # lifecycle_manager 自动做这件事，autostart=true 表示启动后
    # 立刻自动激活。
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        slam_toolbox_node,
        lifecycle_manager_node,
    ])
