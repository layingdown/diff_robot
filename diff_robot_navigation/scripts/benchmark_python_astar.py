#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
benchmark_python_astar.py
===========================
和 diff_robot_planner_cpp/benchmark/benchmark_main.cpp 对称的 Python 版求解耗时基准，
用完全相同的地图生成逻辑（房间/走廊/障碍布局按网格尺寸等比缩放）、完全相同的
起终点选取方式、完全相同的重复次数（30 次取均值/最大值），
产出一张可以直接和 C++ 结果并排放进 docs/tuning_log.md 的表格
（需求4："C++ 重构 A* 核心模块...验证多语言开发能力"，这里给出实测依据，
而不是只写"C++ 更快"这种没有数字支撑的结论）。

用法：
    python3 scripts/benchmark_python_astar.py
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from diff_robot_navigation.astar import GridMap, plan, PlannerError  # noqa: E402


def build_scaled_demo_map(size, res):
    """和 diff_robot_planner_cpp/benchmark/benchmark_main.cpp 里的 build_demo_map
    使用同一套 140x140 基准布局按比例缩放，保证两边测的是"同一张逻辑地图"。"""
    h = w = size
    data = np.zeros((h, w), dtype=np.int16)
    data[0, :] = 100
    data[-1, :] = 100
    data[:, 0] = 100
    data[:, -1] = 100

    s = size / 140.0

    def blk(r0, r1, c0, c1, v):
        data[int(r0 * s):int(r1 * s), int(c0 * s):int(c1 * s)] = v

    blk(45, 50, 0, 140, 100)
    blk(45, 50, 55, 65, 0)
    blk(90, 95, 0, 140, 100)
    blk(90, 95, 20, 30, 0)
    blk(90, 95, 100, 112, 0)
    blk(15, 25, 60, 75, 100)
    blk(60, 75, 15, 25, 100)
    blk(60, 70, 90, 110, 100)
    blk(105, 120, 55, 70, 100)

    return GridMap(data=data, resolution=res, origin_x=0.0, origin_y=0.0,
                    inflation_radius_m=0.25, inflation_weight=6.0)


def main():
    print(f"{'grid':<10}{'resolution':<12}{'mean_ms':<14}{'max_ms':<14}")
    for size, res in [(140, 0.05), (280, 0.025), (560, 0.0125)]:
        grid = build_scaled_demo_map(size, res)
        start = (0.4, 0.4)
        goal = (size * res - 0.7, size * res - 0.7)
        times = []
        for _ in range(30):
            t0 = time.perf_counter()
            try:
                plan(grid, start, goal)
                times.append((time.perf_counter() - t0) * 1000.0)
            except PlannerError as e:
                print(f"  (skip trial: {e})")
        if times:
            print(f"{f'{size}x{size}':<10}{res:<12.4f}{np.mean(times):<14.4f}{np.max(times):<14.4f}")


if __name__ == '__main__':
    main()
