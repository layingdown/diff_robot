#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_controllers.launch.py
===============================
需求3的"实机对比"入口：Gazebo 仿真 + 全局规划节点 + 单独一个轨迹跟踪控制器
（Pure Pursuit 或 MPC，用 launch 参数 controller 二选一），跳过 bt_navigator/
DWB，专注对比两种跟踪算法本身的表现，和 scripts/tune_and_compare.py 的离线
结论相互印证。

用法：
    # 先跑一次 Pure Pursuit：
    ros2 launch diff_robot_navigation compare_controllers.launch.py controller:=pure_pursuit
    # 再跑一次 MPC，对比 rqt_plot /cmd_vel、/mpc_controller/solve_time_ms 等话题：
    ros2 launch diff_robot_navigation compare_controllers.launch.py controller:=mpc

    # 换一组起终点：
    ros2 launch diff_robot_navigation compare_controllers.launch.py \
        controller:=mpc goal_x:=5.0 goal_y:=3.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('diff_robot_description')
    nav_share = get_package_share_directory('diff_robot_navigation')

    controller_arg = DeclareLaunchArgument(
        'controller', default_value='pure_pursuit',
        description="'pure_pursuit' 或 'mpc'")
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='6.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='6.0')
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(
            get_package_share_directory('diff_robot_navigation'),
            'maps', 'slam_test_world.yaml'),
        description='A* 全局规划需要一张静态地图，建图阶段(mapping.launch.py)先保存好')

    controller = LaunchConfiguration('controller')
    map_yaml = LaunchConfiguration('map')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'rviz_config': os.path.join(desc_share, 'rviz', 'nav.rviz'),
        }.items(),
    )

    # A* 需要一张静态地图（占用栅格），这里只起 map_server 本身 +
    # 一个只管它一个节点的迷你 lifecycle_manager（不需要 AMCL/controller_server/
    # bt_navigator 这一整套，本 launch 的目的是单独对比跟踪控制器，不是完整导航闭环）。
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
    )
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}],
    )

    global_planner_node = Node(
        package='diff_robot_navigation', executable='global_planner_node',
        name='planner_server', output='screen',
        parameters=[{'use_sim_time': True, 'planner_frame': 'map'}],
    )

    # 这个 launch 不跑 AMCL（没有做全局重定位，只是单独对比跟踪控制器），
    # 所以手动发一个 map->odom 的静态恒等变换，把 "map" 系和机器人出生点所在的
    # "odom" 系重合起来，保证 TF 树完整、A* 和控制器用的坐标系一致。
    # 真正做闭环导航定位时用 navigation.launch.py（里面有 AMCL 动态估计这个变换）。
    static_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    pure_pursuit_node = Node(
        package='diff_robot_navigation', executable='pure_pursuit_controller',
        output='screen',
        parameters=[os.path.join(nav_share, 'config', 'pure_pursuit_params.yaml')],
        condition=IfCondition(PythonExpression(["'", controller, "' == 'pure_pursuit'"])),
    )

    mpc_node = Node(
        package='diff_robot_navigation', executable='mpc_controller',
        output='screen',
        parameters=[os.path.join(nav_share, 'config', 'mpc_params.yaml')],
        condition=IfCondition(PythonExpression(["'", controller, "' == 'mpc'"])),
    )

    # 等 Gazebo + 规划节点都起来之后再发目标，5s 是留给 gz_server/spawn 的经验值，
    # 如果机器比较慢需要相应调大（也可以手动在另一个终端里再跑一次 send_goal）。
    send_goal_action = TimerAction(
        period=8.0,
        actions=[Node(
            package='diff_robot_navigation', executable='send_goal',
            output='screen',
            parameters=[{
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'use_start': True,
                'start_x': 0.0, 'start_y': 0.0,  # 机器人出生点，见 gazebo.launch.py 里的 spawn 坐标
                'frame': 'map',
            }],
        )],
    )

    return LaunchDescription([
        controller_arg,
        goal_x_arg,
        goal_y_arg,
        map_arg,
        gazebo_launch,
        map_server_node,
        map_lifecycle_manager,
        static_map_to_odom,
        global_planner_node,
        pure_pursuit_node,
        mpc_node,
        send_goal_action,
    ])
