#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_maps.py
============
构造与 diff_robot_description/worlds/slam_test_world.world 风格类似的合成栅格地图
（多房间 + 走廊 + 孤立障碍块），供离线可视化 (scripts/visualize_astar_demo.py) 和
离线调参 (scripts/tune_and_compare.py) 复用，两边保证用的是同一张地图，结果可比。

真正联调时，这张合成地图会被 slam_toolbox 建出来的真实 map.yaml/map.pgm 替换，
用法完全一样（GridMap 只关心 occupancy 数组 + 分辨率 + 原点，不关心数据来源）。
"""
from __future__ import annotations

import numpy as np

from diff_robot_navigation.astar import GridMap


def build_demo_map(h=140, w=140, res=0.05,
                    inflation_radius_m=0.25, inflation_weight=6.0) -> GridMap:
    data = np.zeros((h, w), dtype=np.int16)
    data[0, :] = 100
    data[-1, :] = 100
    data[:, 0] = 100
    data[:, -1] = 100

    data[45:50, 0:140] = 100
    data[45:50, 55:65] = 0     # 门1

    data[90:95, 0:140] = 100
    data[90:95, 20:30] = 0     # 门2
    data[90:95, 100:112] = 0   # 门3

    data[15:25, 60:75] = 100
    data[60:75, 15:25] = 100
    data[60:70, 90:110] = 100
    data[105:120, 55:70] = 100

    return GridMap(data=data, resolution=res, origin_x=0.0, origin_y=0.0,
                    inflation_radius_m=inflation_radius_m, inflation_weight=inflation_weight)
