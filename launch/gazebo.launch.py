#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('diff_robot_description').find('diff_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'diff_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # 1. 启动 Gazebo Server（无 GUI）
    # 修正：去掉了 libgazebo_ros_factory.so —— 这是 Gazebo Classic 的插件参数，
    # 在新版 gz sim (Harmonic) 里不认识这个参数，是导致偶发加载失败的可疑原因之一。
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-v', '4', '-r', '--headless-rendering', world_file],
        output='screen'
)

    # 2. Gazebo GUI 客户端
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '--render-engine', 'ogre2'],
        output='screen'
    )

    # 3. robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # 4. joint_state_publisher（调试用）


    # 5. 生成机器人实体
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diff_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # 6. ros_gz_bridge 桥接
    bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/world/default/model/diff_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    ],
    remappings=[
        ('/world/default/model/diff_robot/joint_state', '/joint_states'),
    ],
    output='screen'
)

    # 7.开rviz
    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', os.path.join(pkg_share, 'rviz', 'diff_robot.rviz')],
    output='screen'
)

    # ---- 事件驱动的启动顺序 ----
    # gz_server 进程真正启动后，再依次拉起 GUI / robot_state_publisher /
    odom_to_tf_node = Node(
        package='diff_robot_description',
        executable='odom_to_tf',
        output='screen'
)

# 新增：odom协方差注入节点
    injector_node = Node(
        package='diff_robot_description',
        executable='injector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 新增：EKF融合节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': True}
        ]
    )
    start_after_server = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_server,
            on_start=[
                gz_gui,
                robot_state_publisher_node,
                bridge_node,
                injector_node,
                ekf_node,
                rviz_node,
            ]
        )
    )

    # robot_state_publisher 启动后，再等 3 秒（给 Gazebo world 内部完全加载留余量），
    # 然后再 spawn 机器人。这里保留一点延时是因为"进程启动"不等于"gz sim 内部场景加载完毕"，
    # 用延时兜底比单纯监听进程启动更保险。
    spawn_after_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[spawn_entity_node]
                )
            ]
        )
    )

    return LaunchDescription([
        gz_server,
        start_after_server,
        spawn_after_rsp,
    ])
