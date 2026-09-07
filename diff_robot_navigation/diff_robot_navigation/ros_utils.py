#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_utils.py
============
一些两个控制器节点都要用到的小工具，单独拆出来避免重复代码。
故意不依赖 tf_transformations（该包在 ROS2 桌面完整安装里不是默认自带的，
额外要 `apt install ros-humble-tf-transformations` + `pip install transforms3d`），
自己手写一个 yaw-only 的四元数转换，减少这个项目的外部依赖面。
"""
import math


def yaw_from_quaternion(q) -> float:
    """只关心绕 Z 轴的偏航角（差速驱动机器人本来就是 2D 平面运动，roll/pitch 恒为 0）。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
