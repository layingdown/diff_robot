#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visualize_astar_demo.py
========================
离线可视化脚本，不需要 ROS2 / Gazebo，用来在提交前肉眼+数值双重确认 A* 算法：
  - 构造一张带走廊/多个房间的合成栅格地图（也可以传 --map 加载真实的 map.pgm+yaml）；
  - 对比"原始 A* 折线路径" vs "视线裁剪+样条平滑后的路径"；
  - 支持 --dynamic-goals 随机撒点跑多组起终点，验证动态起点/终点场景下算法的鲁棒性；
  - 输出一张 PNG（起终点、障碍物、两种路径叠加），以及每组用例的路径长度、
    最近障碍物距离（安全裕度）、求解耗时，打印成表格，可以直接抄进调参记录。

用法：
    python3 scripts/visualize_astar_demo.py --out /tmp/astar_demo.png --dynamic-goals 5
"""
import argparse
import os
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from diff_robot_navigation.astar import (  # noqa: E402
    GridMap, a_star_search, prune_path, smooth_path_world, path_length, PlannerError,
)
from diff_robot_navigation.demo_maps import build_demo_map  # noqa: E402,F401


def min_clearance_m(grid: GridMap, points):
    field = grid.inflation_cost_field()
    # 用膨胀权重反推最近障碍物距离下界（仅用于报告，不影响规划）
    clearances = []
    for x, y in points:
        r, c = grid.world_to_grid(x, y)
        if 0 <= r < grid.height and 0 <= c < grid.width:
            cost = field[r, c]
            frac = 1.0 - cost / grid.inflation_weight if grid.inflation_weight else 1.0
            clearances.append(max(frac, 0.0) * grid.inflation_radius_m)
    return min(clearances) if clearances else float('nan')


def run_one(grid, start, goal):
    t0 = time.perf_counter()
    start_cell = grid.world_to_grid(*start)
    goal_cell = grid.world_to_grid(*goal)
    raw_cells = a_star_search(grid, start_cell, goal_cell)
    pruned_cells = prune_path(grid, raw_cells)
    smoothed = smooth_path_world(grid, pruned_cells)
    dt_ms = (time.perf_counter() - t0) * 1000.0

    raw_world = [grid.grid_to_world(r, c) for r, c in raw_cells]
    return {
        'start': start, 'goal': goal,
        'raw_world': raw_world, 'smoothed': smoothed,
        'raw_len': path_length(raw_world), 'smoothed_len': path_length(smoothed),
        'solve_ms': dt_ms,
        'clearance_m': min_clearance_m(grid, smoothed),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/astar_demo.png')
    ap.add_argument('--dynamic-goals', type=int, default=4,
                     help='随机附加多少组起终点用来验证动态起点/终点鲁棒性')
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()

    grid = build_demo_map()
    rng = np.random.default_rng(args.seed)

    cases = [((0.4, 0.4), (6.3, 6.3))]  # 主用例：贯穿三个房间的长距离路径
    tries = 0
    while len(cases) < 1 + args.dynamic_goals and tries < 200:
        tries += 1
        sx, sy = rng.uniform(0.3, 6.6, size=2)
        gx, gy = rng.uniform(0.3, 6.6, size=2)
        s_cell = grid.world_to_grid(sx, sy)
        g_cell = grid.world_to_grid(gx, gy)
        if grid.is_free(s_cell) and grid.is_free(g_cell):
            if np.hypot(gx - sx, gy - sy) > 2.0:
                cases.append(((sx, sy), (gx, gy)))

    results = []
    for start, goal in cases:
        try:
            results.append(run_one(grid, start, goal))
        except PlannerError as e:
            print(f"[SKIP] {start} -> {goal}: {e}")

    fig, ax = plt.subplots(figsize=(7, 7))
    occ = grid.data > 50
    ax.imshow(occ, cmap='Greys', origin='lower',
              extent=[grid.origin_x, grid.origin_x + grid.width * grid.resolution,
                      grid.origin_y, grid.origin_y + grid.height * grid.resolution])

    colors = plt.cm.tab10(np.linspace(0, 1, len(results)))
    print(f"{'case':<6}{'raw_len(m)':<12}{'smooth_len(m)':<15}{'solve_ms':<10}{'clearance(m)':<14}")
    for i, r in enumerate(results):
        raw_x = [p[0] for p in r['raw_world']]
        raw_y = [p[1] for p in r['raw_world']]
        sm_x = [p[0] for p in r['smoothed']]
        sm_y = [p[1] for p in r['smoothed']]
        ax.plot(raw_x, raw_y, '--', color=colors[i], alpha=0.5, linewidth=1.2,
                label=f'case{i} raw A*' if i == 0 else None)
        ax.plot(sm_x, sm_y, '-', color=colors[i], linewidth=2.2,
                label=f'case{i} smoothed' if i == 0 else None)
        ax.plot(*r['start'], marker='o', color=colors[i], markersize=8)
        ax.plot(*r['goal'], marker='*', color=colors[i], markersize=14)
        print(f"{i:<6}{r['raw_len']:<12.3f}{r['smoothed_len']:<15.3f}"
              f"{r['solve_ms']:<10.3f}{r['clearance_m']:<14.3f}")

    ax.set_title('A* global planner: raw path (dashed) vs smoothed path (solid)\n'
                  f'{len(results)} start/goal cases, dots=start stars=goal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"saved figure to {args.out}")


if __name__ == '__main__':
    main()
