from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'diff_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装所有 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装所有 urdf 文件（.urdf 和 .urdf.xacro）
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),
        # 安装 worlds 文件
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # 安装 config 文件（如果有 .yaml 或 .yaml）
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wei',
    maintainer_email='wei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
