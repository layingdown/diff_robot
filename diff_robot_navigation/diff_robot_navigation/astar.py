#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
astar.py
========
纯 Python / numpy 实现的栅格地图 A* 全局路径规划核心算法。

设计目标（对应需求 2）：
  - 输入一张占据栅格地图（0=空闲, 100=占据, -1=未知，遵循 nav_msgs/OccupancyGrid 惯例，
    也兼容 0/1 二值栅格），以及起点/终点（可以是任意像素坐标，运行时动态给定）。
  - 8 邻域搜索，允许沿对角线移动，代价用欧氏距离而不是曼哈顿距离，
    保证在无障碍区域规划出的路径就是直线，不会有锯齿。
  - 用一张"距离障碍物的膨胀代价场"叠加到搜索代价里，让路径天然倾向于走地图中间，
    不贴墙走（等价于 Nav2 costmap 的 inflation layer 思路，但是自己实现的）。
  - 规划完成后做两级平滑：
      1) 视线法（line-of-sight）裁剪共线/可直连的中间点，去掉锯齿状转折；
      2) 三次样条 / Catmull-Rom 平滑，并逐段做碰撞检测，若平滑后穿墙则回退到裁剪结果，
         保证"路径平滑且无碰撞"这个硬约束不被样条破坏。

本模块不依赖 rclpy / ROS2，可以独立 `python3 -c "import astar"` 单元测试，
也被 diff_robot_navigation/global_planner_node.py（ROS2 封装）和
scripts/tune_and_compare.py（离线整定脚本）复用，避免核心算法在两个地方各写一份。

C++ 版本见 diff_robot_planner_cpp/src/astar_core.cpp，是本文件算法的逐行对照移植，
用于验证多语言开发能力并做求解耗时对比（需求 4）。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

Cell = Tuple[int, int]  # (row, col)  即 (y, x) 栅格索引
Point = Tuple[float, float]  # (x, y) 连续坐标（米）

OCC_UNKNOWN = -1
OCC_FREE_MAX = 50  # occupancy <= 这个值视为可通行（nav_msgs/OccupancyGrid: 0~100）

# 8 邻域及其欧氏距离代价
_NEIGHBORS_8: Tuple[Tuple[int, int, float], ...] = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
)


@dataclass
class GridMap:
    """占据栅格地图的最小封装：数据 + 分辨率 + 原点，负责像素<->世界坐标转换。"""

    data: np.ndarray          # shape (height, width)，int8/int16，occupancy 值
    resolution: float         # 米/像素
    origin_x: float           # 栅格 (0,0) 对应的世界坐标 x（左下角）
    origin_y: float
    inflation_radius_m: float = 0.35   # 障碍物膨胀半径（米），影响“不贴墙走”的程度
    inflation_weight: float = 6.0      # 膨胀代价场在 A* 代价里的权重

    def __post_init__(self) -> None:
        self.height, self.width = self.data.shape
        self._cost_field: Optional[np.ndarray] = None

    # ---------- 坐标转换 ----------
    def world_to_grid(self, x: float, y: float) -> Cell:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def grid_to_world(self, row: int, col: int) -> Point:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    # ---------- 基础查询 ----------
    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, cell: Cell, allow_unknown: bool = False) -> bool:
        if not self.in_bounds(cell):
            return False
        v = self.data[cell[0], cell[1]]
        if v == OCC_UNKNOWN:
            return allow_unknown
        return v <= OCC_FREE_MAX

    # ---------- 膨胀代价场（等价于简化版 costmap inflation layer）----------
    def inflation_cost_field(self) -> np.ndarray:
        """返回与 data 同形状的浮点代价场：离障碍物越近代价越高，用于让 A* 路径居中。

        用 scipy 的欧氏距离变换（若不可用则退化为布尔膨胀 + 反比近似），
        对性能和依赖都比较友好，纯 numpy 也能算。
        """
        if self._cost_field is not None:
            return self._cost_field

        occupied = (self.data > OCC_FREE_MAX)
        if not occupied.any():
            self._cost_field = np.zeros_like(self.data, dtype=np.float64)
            return self._cost_field

        try:
            from scipy.ndimage import distance_transform_edt
            dist_px = distance_transform_edt(~occupied)
        except Exception:
            # 没有 scipy 时的退化实现：暴力布尔膨胀近似（略慢，但结果可用）
            dist_px = _bruteforce_distance_transform(occupied)

        dist_m = dist_px * self.resolution
        radius = max(self.inflation_radius_m, 1e-6)
        # 在膨胀半径以内，代价随距离线性衰减到 0；半径以外代价为 0（不影响自由空间选路）
        cost = np.clip(1.0 - dist_m / radius, 0.0, 1.0)
        self._cost_field = cost * self.inflation_weight
        return self._cost_field


def _bruteforce_distance_transform(occupied: np.ndarray) -> np.ndarray:
    """scipy 不可用时的退化实现：多源 BFS 计算到最近障碍物的像素距离（曼哈顿近似欧氏）。"""
    h, w = occupied.shape
    dist = np.full((h, w), np.inf)
    from collections import deque
    q = deque()
    ys, xs = np.nonzero(occupied)
    for y, x in zip(ys, xs):
        dist[y, x] = 0.0
        q.append((y, x))
    while q:
        y, x = q.popleft()
        d0 = dist[y, x]
        for dy, dx, dc in _NEIGHBORS_8:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w:
                nd = d0 + dc
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    q.append((ny, nx))
    return dist


class PlannerError(RuntimeError):
    """A* 规划失败（起点/终点非法、或搜索空间内不存在可行路径）。"""


def _nearest_free_cell(
    grid: "GridMap", cell: Cell, max_radius: int, allow_unknown: bool = True
) -> Optional[Cell]:
    """以 cell 为中心，一圈一圈往外找最近的一个"可通行"格子，找不到返回 None。

    用于起点恰好落在孤立噪声像素/极窄障碍物上时的兜底（见⑯的说明）：只按
    切比雪夫距离一圈一圈扩大搜索范围（对栅格来说足够均匀，没必要用更贵的
    欧氏距离扩圈算法），同一圈内再按真实欧氏距离挑最近的一个，尽量让"贴回
    最近的自由格子"这个动作本身产生的偏移最小。
    """
    if grid.is_free(cell, allow_unknown):
        return cell
    r0, c0 = cell
    for radius in range(1, max_radius + 1):
        candidates = []
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if max(abs(dr), abs(dc)) != radius:
                    continue  # 只看这一圈的外壳，内圈上一轮已经查过了
                nc = (r0 + dr, c0 + dc)
                if grid.in_bounds(nc) and grid.is_free(nc, allow_unknown):
                    candidates.append(nc)
        if candidates:
            candidates.sort(key=lambda c: (c[0] - r0) ** 2 + (c[1] - c0) ** 2)
            return candidates[0]
    return None


def a_star_search(
    grid: GridMap,
    start: Cell,
    goal: Cell,
    allow_unknown: bool = False,
    max_expansions: int = 400_000,
    start_snap_radius_cells: int = 6,
) -> List[Cell]:
    """标准 8 邻域 A*，欧氏距离启发式（可采纳、一致），返回栅格坐标路径（含起终点）。"""
    if not grid.in_bounds(start):
        raise PlannerError(f"起点越界: {start}")
    if not grid.in_bounds(goal):
        raise PlannerError(f"终点越界: {goal}")
    # 2026-09-07 修复（会话更新⑮，⑯里发现诊断有误并修正）：起点永远来自机器人
    # 当下的真实 TF 位置——它"现在就站在这里"是既成事实。⑮版最初以为问题是
    # "SLAM 地图这一格没探索到"（occupancy 未知），后来实测比对地图像素发现
    # 判断错了：两次真实触发的失败格子其实都是 occupancy 明确 >50 的"障碍物"，
    # 而且都是嵌在大片自由空间中的孤立小块（3~6个格子的细线/斜线），跟本项目
    # 早就记录过的"SLAM扫描匹配噪声在地图上留下细斜线痕迹"（见会话更新⑥）
    # 完全对得上——这些不是真实世界里存在的墙，只是建图阶段的噪声，Gazebo
    # 里机器人从那正常穿过去毫无阻碍，但 A* 严格按地图判断会误以为撞墙。
    # 由于用户已经明确表示不想再花时间修SLAM建图质量本身，这里改成从算法
    # 侧兜底：起点这一格如果确实是"障碍物"，不要立刻整体报错，先在起点周围
    # 一小圈（start_snap_radius_cells，默认6格≈0.3米，小于机器人自身尺寸，
    # 不会把起点挪到明显不合理的位置）里找最近的自由格子，把起点贴过去继续
    # 规划；这一小圈里如果确实一个自由格子都找不到（说明起点周围是一片连续
    # 的真实障碍区，不是孤立噪声点），才真正报错——这种情况继续报错是对的、
    # 不应该被这个兜底掩盖掉。
    start_allow_unknown = True
    if not grid.is_free(start, allow_unknown=start_allow_unknown):
        snapped = _nearest_free_cell(
            grid, start, start_snap_radius_cells, allow_unknown=start_allow_unknown)
        if snapped is None:
            raise PlannerError(f"起点位于障碍物，且周围{start_snap_radius_cells}格内找不到自由格子: {start}")
        start = snapped
    if not grid.is_free(goal, allow_unknown):
        raise PlannerError(f"终点位于障碍物/未知区域: {goal}")

    cost_field = grid.inflation_cost_field()

    def heuristic(a: Cell, b: Cell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: List[Tuple[float, int, Cell]] = []
    counter = 0  # 打破堆里 f 值相同时的比较歧义，保证稳定排序
    heapq.heappush(open_heap, (0.0, counter, start))

    g_score = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    closed = set()

    expansions = 0
    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expansions += 1
        if expansions > max_expansions:
            raise PlannerError("超过最大搜索次数，可能地图过大或无可行路径")

        if current == goal:
            return _reconstruct_path(came_from, current)

        for dy, dx, step_cost in _NEIGHBORS_8:
            nxt = (current[0] + dy, current[1] + dx)
            if nxt in closed or not grid.is_free(nxt, allow_unknown):
                continue
            # 禁止穿对角线夹缝（两侧都是障碍物时不允许斜着穿过去）
            if dy != 0 and dx != 0:
                if not grid.is_free((current[0] + dy, current[1]), allow_unknown):
                    continue
                if not grid.is_free((current[0], current[1] + dx), allow_unknown):
                    continue

            extra = cost_field[nxt[0], nxt[1]]
            tentative_g = g_score[current] + step_cost + extra
            if tentative_g < g_score.get(nxt, math.inf):
                g_score[nxt] = tentative_g
                came_from[nxt] = current
                f = tentative_g + heuristic(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (f, counter, nxt))

    raise PlannerError("A* 未找到可行路径（起点终点不连通）")


def _reconstruct_path(came_from: dict, current: Cell) -> List[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# 路径平滑
# ---------------------------------------------------------------------------

def _has_line_of_sight(grid: GridMap, a: Cell, b: Cell) -> bool:
    """Bresenham 直线走栅格，逐格检查是否穿过障碍物，用于视线法裁剪和样条防穿墙校验。

    这里发现过一个真实 bug（C++ 移植版跑 gtest 时先暴露出来的）：Bresenham 在
    "对角步"（同一步里 r 和 c 都变化）时，只检查了目的地格子是否空闲，没有像
    a_star_search 的邻居扩展那样，同时检查对角线两侧的"墙角"格子——这会让视线法
    判定为"可通视"，实际上是贴着一个障碍物的墙角斜穿过去的，在门框这种一格宽的缺口
    附近，样条平滑得到的曲线可能因此贴到墙角外沿一点点，被判定成"无碰撞"但其实
    已经蹭到障碍物。修复方式和 a_star_search 里禁止斜穿墙缝完全一致：对角步时，
    额外要求两个相邻的"直边"格子也都空闲。
    """
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        if not grid.is_free((r, c)):
            return False
        if r == r1 and c == c1:
            return True
        e2 = 2 * err
        step_r = e2 > -dc
        step_c = e2 < dr
        if step_r and step_c:
            # 对角步：禁止贴着墙角斜穿（两侧格子必须都空闲，否则视为撞墙）
            if not grid.is_free((r + sr, c)) or not grid.is_free((r, c + sc)):
                return False
        if step_r:
            err -= dc
            r += sr
        if step_c:
            err += dr
            c += sc


def prune_path(grid: GridMap, path: Sequence[Cell]) -> List[Cell]:
    """视线法路径裁剪：从当前点尽量往后找最远的、有直线视线的点，删掉中间冗余点。"""
    if len(path) <= 2:
        return list(path)
    pruned = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        j = n - 1
        while j > i + 1 and not _has_line_of_sight(grid, path[i], path[j]):
            j -= 1
        pruned.append(path[j])
        i = j
    return pruned


def smooth_path_world(
    grid: GridMap,
    pruned_grid_path: Sequence[Cell],
    samples_per_segment: int = 12,
) -> List[Point]:
    """对裁剪后的关键点做 Catmull-Rom 样条平滑，输出世界坐标点序列。

    每段样条采样后都会逐段做碰撞检测（is_free），一旦发现该段穿墙，
    整条路径退化为对裁剪结果做线性插值（不平滑但保证安全），
    从而保证“路径平滑且无碰撞”这个约束优先级高于“好看”。
    """
    world_pts = [grid.grid_to_world(r, c) for r, c in pruned_grid_path]
    if len(world_pts) < 3:
        return world_pts

    pts = [world_pts[0]] + list(world_pts) + [world_pts[-1]]
    smoothed: List[Point] = [world_pts[0]]
    collision_free = True

    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
        for s in range(1, samples_per_segment + 1):
            t = s / samples_per_segment
            pt = _catmull_rom(p0, p1, p2, p3, t)
            smoothed.append(pt)

    # 逐段碰撞检测（用世界坐标转回栅格做视线检查）
    for k in range(len(smoothed) - 1):
        a = grid.world_to_grid(*smoothed[k])
        b = grid.world_to_grid(*smoothed[k + 1])
        if not _has_line_of_sight(grid, a, b):
            collision_free = False
            break

    if collision_free:
        return smoothed

    # 回退：对裁剪路径做等距线性插值（不平滑但保证与 A* 结果一样安全）
    fallback: List[Point] = []
    for i in range(len(world_pts) - 1):
        p0, p1 = world_pts[i], world_pts[i + 1]
        for s in range(samples_per_segment):
            t = s / samples_per_segment
            fallback.append((p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t))
    fallback.append(world_pts[-1])
    return fallback


def _catmull_rom(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        2 * p1[0] + (-p0[0] + p2[0]) * t
        + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        2 * p1[1] + (-p0[1] + p2[1]) * t
        + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
    )
    return x, y


def plan(
    grid: GridMap,
    start_world: Point,
    goal_world: Point,
    allow_unknown: bool = False,
) -> List[Point]:
    """一步到位的对外 API：世界坐标起终点 -> A* -> 裁剪 -> 样条平滑 -> 世界坐标路径。"""
    start_cell = grid.world_to_grid(*start_world)
    goal_cell = grid.world_to_grid(*goal_world)
    raw = a_star_search(grid, start_cell, goal_cell, allow_unknown=allow_unknown)
    pruned = prune_path(grid, raw)
    return smooth_path_world(grid, pruned)


def path_length(points: Sequence[Point]) -> float:
    return sum(
        math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        for i in range(len(points) - 1)
    )
