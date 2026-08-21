#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('diff_robot_description').find('diff_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'diff_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # 1. 启动 Gazebo Server（无 GUI）
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-v', '4', '-r', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),
        
        # 2. 延迟 3 秒后启动 GUI 客户端
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['gz', 'sim', '-g', '--render-engine', 'ogre'],
                    output='screen'
                )
            ]
        ),
        
        # 3. robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content},
                {'use_sim_time': True}
            ],
            output='screen'
        ),
        
        # 4. joint_state_publisher（调试用）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # 5. 延迟 5 秒后生成机器人，抬高 0.1m
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-name', 'diff_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
                    output='screen'
                )
            ]
        ),
        
        # 6. ros_gz_bridge 桥接
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'
            ],
            output='screen'
        ),
    ])
