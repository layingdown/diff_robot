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
    # 改用 SLAM 测试地图（原 empty.world 已替换）
    world_file = os.path.join(pkg_share, 'worlds', 'slam_test_world.world')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # 1. 启动 Gazebo Server（无 GUI）
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

    # 5. 生成机器人实体（保持原点出生，孤岛已在地图里挪开，不再冲突）
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diff_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # 6. ros_gz_bridge 桥接
    # 注意：world 名字改成了 slam_test_world，joint_state 话题路径必须跟着改，
    # 否则 gz 端根本没有 /world/default/... 这个话题，桥接会静默失效。
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/slam_test_world/model/diff_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/slam_test_world/model/diff_robot/joint_state', '/joint_states'),
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

    odom_to_tf_node = Node(
        package='diff_robot_description',
        executable='odom_to_tf',
        output='screen'
    )

    # odom协方差注入节点
    injector_node = Node(
        package='diff_robot_description',
        executable='injector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # EKF融合节点
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
                # gz_gui 默认不开：这是独立的 3D 渲染窗口（ogre2），和 headless
                # server 的传感器渲染分开吃资源，配置低的机器建图/跑导航时容易卡。
                # 建图/调试不需要看 Gazebo 里的 3D 画面，RViz 已经够用；
                # 真要用 Gazebo GUI（比如手动拖障碍物）就把下面这行取消注释。
                # gz_gui,
                robot_state_publisher_node,
                bridge_node,
                injector_node,
                ekf_node,
                rviz_node,
            ]
        )
    )

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
