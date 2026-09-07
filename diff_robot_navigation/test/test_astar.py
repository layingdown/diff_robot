#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_astar.py
=============
纯离线单元测试（不需要 ROS2/colcon，直接 `pytest test/test_astar.py` 或
`python3 -m pytest test/` 即可跑），覆盖：
  1. 空地图上应规划出一条接近直线的最短路径；
  2. 复杂障碍物地图上路径必须完全无碰撞（逐点、逐样条采样点检查）；
  3. 起点/终点在障碍物里应抛出 PlannerError 而不是静默返回错误结果；
  4. 平滑后的路径长度不应比裁剪前的折线长太多（平滑是“走近道”不是“绕远路”）。
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from diff_robot_navigation.astar import (  # noqa: E402
    GridMap, PlannerError, a_star_search, plan, path_length, _has_line_of_sight,
)


def make_empty_grid(h=60, w=60, res=0.05):
    data = np.zeros((h, w), dtype=np.int16)
    return GridMap(data=data, resolution=res, origin_x=0.0, origin_y=0.0)


def make_maze_grid(h=80, w=80, res=0.05):
    """构造一个带走廊/凸起障碍的迷宫地图，逼真度接近 slam_test_world 的复杂度。

    布局（栅格行列）：起点房间(row<20) --横墙(row 20:25, 门在col 40:46)-->
    中间房间(20<row<50) --竖墙(col 30:34, 门在row 55:62)--> 终点房间。
    起点 (0.5,0.5)->cell(10,10)、终点 (3.5,3.6)->cell(72,70) 都留在空房间正中，
    不会被任何墙体覆盖。
    """
    data = np.zeros((h, w), dtype=np.int16)
    data[20:25, 0:80] = 100           # 第一堵横墙
    data[20:25, 40:46] = 0            # 留门
    data[30:62, 30:34] = 100          # 竖直隔墙
    data[55:62, 30:34] = 0            # 留门
    data[45:50, 0:80] = 100           # 第二堵横墙（终点房间和中间房间之间）
    data[45:50, 60:68] = 0            # 留门
    return GridMap(data=data, resolution=res, origin_x=0.0, origin_y=0.0,
                    inflation_radius_m=0.2)


def assert_path_collision_free(grid: GridMap, points):
    for x, y in points:
        cell = grid.world_to_grid(x, y)
        assert grid.is_free(cell), f"路径点 {(x, y)} -> 栅格 {cell} 落在障碍物上！"
    for i in range(len(points) - 1):
        a = grid.world_to_grid(*points[i])
        b = grid.world_to_grid(*points[i + 1])
        assert _has_line_of_sight(grid, a, b), (
            f"路径段 {points[i]} -> {points[i+1]} 穿过了障碍物！")


def test_empty_grid_near_straight_line():
    grid = make_empty_grid()
    start = (0.2, 0.2)
    goal = (2.5, 2.3)
    pts = plan(grid, start, goal)
    assert_path_collision_free(grid, pts)
    straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
    length = path_length(pts)
    assert length < straight * 1.05, f"空地图上路径应接近直线，实际长度 {length:.3f} vs 直线 {straight:.3f}"


def test_maze_grid_collision_free():
    grid = make_maze_grid()
    start = (0.5, 0.5)
    goal = (3.5, 3.6)
    pts = plan(grid, start, goal)
    assert len(pts) > 2
    assert_path_collision_free(grid, pts)


def test_start_or_goal_in_obstacle_raises():
    grid = make_maze_grid()
    start_in_wall = grid.grid_to_world(22, 10)  # 横墙内部（不在门缺口范围）
    goal = (3.5, 3.6)
    try:
        plan(grid, start_in_wall, goal)
        assert False, "起点在障碍物内应该抛出 PlannerError"
    except PlannerError:
        pass


def test_smoothing_does_not_lengthen_much():
    grid = make_maze_grid()
    start_cell = grid.world_to_grid(0.5, 0.5)
    goal_cell = grid.world_to_grid(3.5, 3.6)
    raw = a_star_search(grid, start_cell, goal_cell)
    raw_world = [grid.grid_to_world(r, c) for r, c in raw]
    raw_len = path_length(raw_world)

    smoothed = plan(grid, (0.5, 0.5), (3.5, 3.6))
    smoothed_len = path_length(smoothed)
    assert smoothed_len <= raw_len * 1.02, (
        f"平滑后路径不应显著变长: raw={raw_len:.3f} smoothed={smoothed_len:.3f}")


if __name__ == '__main__':
    test_empty_grid_near_straight_line()
    test_maze_grid_collision_free()
    test_start_or_goal_in_obstacle_raises()
    test_smoothing_does_not_lengthen_much()
    print("All astar tests passed.")
