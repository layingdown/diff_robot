import os
from glob import glob

from setuptools import setup, find_packages

package_name = 'diff_robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # 2026-09-07 新增：自定义的最小化行为树 XML（不依赖 global_costmap 清空
        # service，见 navigate_to_pose_minimal.xml 文件头注释），navigation.launch.py
        # 里通过 bt_navigator 的 default_nav_to_pose_bt_xml 参数指向这里安装后的路径。
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wei',
    maintainer_email='wei@todo.todo',
    description='A* global planner + Nav2/DWB local avoidance + Pure Pursuit/MPC tracking',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'global_planner_node = diff_robot_navigation.global_planner_node:main',
            'pure_pursuit_controller = diff_robot_navigation.pure_pursuit_controller:main',
            'mpc_controller = diff_robot_navigation.mpc_controller:main',
            'send_goal = diff_robot_navigation.send_goal:main',
        ],
    },
)
