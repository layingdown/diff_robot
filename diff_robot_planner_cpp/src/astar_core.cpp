// Copyright 2026 diff_robot_planner_cpp contributors
#include "diff_robot_planner_cpp/astar_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace diff_robot_planner_cpp
{

namespace
{
// 8 邻域偏移 + 步进代价（欧氏距离：直走1.0，斜走sqrt(2)），
// 和 astar.py 里的 _NEIGHBORS_8 完全对应。
struct NeighborOffset
{
  int dr;
  int dc;
  double cost;
};

const std::vector<NeighborOffset> kNeighbors8 = {
  {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
  {-1, -1, std::sqrt(2.0)}, {-1, 1, std::sqrt(2.0)},
  {1, -1, std::sqrt(2.0)}, {1, 1, std::sqrt(2.0)},
};

double heuristic(const Cell & a, const Cell & b)
{
  double dr = a.row - b.row;
  double dc = a.col - b.col;
  return std::sqrt(dr * dr + dc * dc);
}

bool has_line_of_sight(const GridMap & grid, const Cell & a, const Cell & b)
{
  // Bresenham 直线走栅格，逐格检查是否穿过障碍物，和 astar.py 的
  // _has_line_of_sight 是同一套整数算术，方便交叉验证结果一致。
  //
  // 这里补了一个真实 bug 的修复：对角步（同一步 r、c 都变化）时，如果只检查
  // 目的地格子，会漏掉"贴着墙角斜穿"的情况——gtest 里 MazeGridCollisionFree
  // 一开始就是这么挂的（跑出来的路径贴到了门框墙角外沿一点点，has_line_of_sight
  // 却判定成通视）。和 a_star_search 里禁止斜穿墙缝的逻辑保持一致：对角步时
  // 额外检查两个相邻直边格子。
  int r0 = a.row, c0 = a.col, r1 = b.row, c1 = b.col;
  int dr = std::abs(r1 - r0), dc = std::abs(c1 - c0);
  int sr = (r1 > r0) ? 1 : -1;
  int sc = (c1 > c0) ? 1 : -1;
  int err = dr - dc;
  int r = r0, c = c0;
  while (true) {
    if (!grid.is_free({r, c})) {return false;}
    if (r == r1 && c == c1) {return true;}
    int e2 = 2 * err;
    bool step_r = e2 > -dc;
    bool step_c = e2 < dr;
    if (step_r && step_c) {
      if (!grid.is_free({r + sr, c}) || !grid.is_free({r, c + sc})) {return false;}
    }
    if (step_r) {
      err -= dc;
      r += sr;
    }
    if (step_c) {
      err += dr;
      c += sc;
    }
  }
}

Point catmull_rom(const Point & p0, const Point & p1, const Point & p2, const Point & p3, double t)
{
  double t2 = t * t;
  double t3 = t2 * t;
  double x = 0.5 * (
    2 * p1.first + (-p0.first + p2.first) * t +
    (2 * p0.first - 5 * p1.first + 4 * p2.first - p3.first) * t2 +
    (-p0.first + 3 * p1.first - 3 * p2.first + p3.first) * t3);
  double y = 0.5 * (
    2 * p1.second + (-p0.second + p2.second) * t +
    (2 * p0.second - 5 * p1.second + 4 * p2.second - p3.second) * t2 +
    (-p0.second + 3 * p1.second - 3 * p2.second + p3.second) * t3);
  return {x, y};
}

}  // namespace

GridMap::GridMap(
  std::vector<int16_t> data, int height, int width, double resolution,
  double origin_x, double origin_y, double inflation_radius_m, double inflation_weight)
: data_(std::move(data)), height_(height), width_(width), resolution_(resolution),
  origin_x_(origin_x), origin_y_(origin_y),
  inflation_radius_m_(inflation_radius_m), inflation_weight_(inflation_weight)
{
  if (static_cast<int>(data_.size()) != height_ * width_) {
    throw std::invalid_argument("GridMap: data size 与 height*width 不匹配");
  }
}

Cell GridMap::world_to_grid(double x, double y) const
{
  int col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  int row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return {row, col};
}

Point GridMap::grid_to_world(int row, int col) const
{
  double x = origin_x_ + (col + 0.5) * resolution_;
  double y = origin_y_ + (row + 0.5) * resolution_;
  return {x, y};
}

bool GridMap::in_bounds(const Cell & c) const
{
  return c.row >= 0 && c.row < height_ && c.col >= 0 && c.col < width_;
}

bool GridMap::is_free(const Cell & c, bool allow_unknown) const
{
  if (!in_bounds(c)) {return false;}
  int16_t v = data_[c.row * width_ + c.col];
  if (v == OCC_UNKNOWN) {return allow_unknown;}
  return v <= OCC_FREE_MAX;
}

const std::vector<double> & GridMap::inflation_cost_field() const
{
  if (cost_field_computed_) {return cost_field_;}
  cost_field_.assign(static_cast<size_t>(height_) * width_, 0.0);

  // 多源 BFS：从所有障碍物格子同时出发，求每个格子到最近障碍物的 8 邻域距离
  // （octile distance，近似欧氏距离），和 astar.py 里 scipy 缺失时的退化实现
  // 同一个思路，复杂度 O(H*W)。
  std::vector<double> dist(static_cast<size_t>(height_) * width_,
    std::numeric_limits<double>::infinity());
  std::deque<Cell> q;
  bool any_obstacle = false;
  for (int r = 0; r < height_; ++r) {
    for (int c = 0; c < width_; ++c) {
      if (data_[r * width_ + c] > OCC_FREE_MAX) {
        dist[r * width_ + c] = 0.0;
        q.push_back({r, c});
        any_obstacle = true;
      }
    }
  }
  if (!any_obstacle) {
    cost_field_computed_ = true;
    return cost_field_;
  }
  while (!q.empty()) {
    Cell cur = q.front();
    q.pop_front();
    double d0 = dist[cur.row * width_ + cur.col];
    for (const auto & n : kNeighbors8) {
      int nr = cur.row + n.dr, nc = cur.col + n.dc;
      if (nr < 0 || nr >= height_ || nc < 0 || nc >= width_) {continue;}
      double nd = d0 + n.cost;
      if (nd < dist[nr * width_ + nc]) {
        dist[nr * width_ + nc] = nd;
        q.push_back({nr, nc});
      }
    }
  }

  double radius = std::max(inflation_radius_m_, 1e-6);
  for (int r = 0; r < height_; ++r) {
    for (int c = 0; c < width_; ++c) {
      double dist_m = dist[r * width_ + c] * resolution_;
      double frac = std::clamp(1.0 - dist_m / radius, 0.0, 1.0);
      cost_field_[r * width_ + c] = frac * inflation_weight_;
    }
  }
  cost_field_computed_ = true;
  return cost_field_;
}

std::vector<Cell> a_star_search(
  const GridMap & grid, const Cell & start, const Cell & goal,
  bool allow_unknown, int max_expansions)
{
  if (!grid.in_bounds(start)) {throw PlannerError("起点越界");}
  if (!grid.in_bounds(goal)) {throw PlannerError("终点越界");}
  if (!grid.is_free(start, allow_unknown)) {throw PlannerError("起点位于障碍物/未知区域");}
  if (!grid.is_free(goal, allow_unknown)) {throw PlannerError("终点位于障碍物/未知区域");}

  const auto & cost_field = grid.inflation_cost_field();
  const int width = grid.width();

  using QueueItem = std::tuple<double, int64_t, Cell>;
  auto cmp = [](const QueueItem & a, const QueueItem & b) {
      if (std::get<0>(a) != std::get<0>(b)) {return std::get<0>(a) > std::get<0>(b);}
      return std::get<1>(a) > std::get<1>(b);
    };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(cmp)> open_heap(cmp);

  std::unordered_map<Cell, double, CellHash> g_score;
  std::unordered_map<Cell, Cell, CellHash> came_from;
  std::unordered_set<Cell, CellHash> closed;

  auto cell_eq = [](const Cell & a, const Cell & b) {return a.row == b.row && a.col == b.col;};
  (void)cell_eq;

  g_score[start] = 0.0;
  int64_t counter = 0;
  open_heap.push({0.0, counter, start});

  int expansions = 0;
  while (!open_heap.empty()) {
    auto [f, cnt, current] = open_heap.top();
    (void)f; (void)cnt;
    open_heap.pop();
    if (closed.count(current)) {continue;}
    closed.insert(current);
    ++expansions;
    if (expansions > max_expansions) {
      throw PlannerError("超过最大搜索次数，可能地图过大或无可行路径");
    }

    if (current.row == goal.row && current.col == goal.col) {
      std::vector<Cell> path;
      Cell c = current;
      path.push_back(c);
      while (came_from.count(c)) {
        c = came_from.at(c);
        path.push_back(c);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const auto & n : kNeighbors8) {
      Cell nxt{current.row + n.dr, current.col + n.dc};
      if (closed.count(nxt) || !grid.is_free(nxt, allow_unknown)) {continue;}
      if (n.dr != 0 && n.dc != 0) {
        if (!grid.is_free({current.row + n.dr, current.col}, allow_unknown)) {continue;}
        if (!grid.is_free({current.row, current.col + n.dc}, allow_unknown)) {continue;}
      }
      double extra = cost_field[nxt.row * width + nxt.col];
      double tentative_g = g_score[current] + n.cost + extra;
      auto it = g_score.find(nxt);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[nxt] = tentative_g;
        came_from[nxt] = current;
        double h = heuristic(nxt, goal);
        ++counter;
        open_heap.push({tentative_g + h, counter, nxt});
      }
    }
  }
  throw PlannerError("A* 未找到可行路径（起点终点不连通）");
}

std::vector<Cell> prune_path(const GridMap & grid, const std::vector<Cell> & path)
{
  if (path.size() <= 2) {return path;}
  std::vector<Cell> pruned;
  pruned.push_back(path.front());
  size_t i = 0;
  size_t n = path.size();
  while (i < n - 1) {
    size_t j = n - 1;
    while (j > i + 1 && !has_line_of_sight(grid, path[i], path[j])) {--j;}
    pruned.push_back(path[j]);
    i = j;
  }
  return pruned;
}

std::vector<Point> smooth_path_world(
  const GridMap & grid, const std::vector<Cell> & pruned_grid_path, int samples_per_segment)
{
  std::vector<Point> world_pts;
  world_pts.reserve(pruned_grid_path.size());
  for (const auto & c : pruned_grid_path) {
    world_pts.push_back(grid.grid_to_world(c.row, c.col));
  }
  if (world_pts.size() < 3) {return world_pts;}

  std::vector<Point> pts;
  pts.push_back(world_pts.front());
  for (const auto & p : world_pts) {
    pts.push_back(p);
  }
  pts.push_back(world_pts.back());

  std::vector<Point> smoothed;
  smoothed.push_back(world_pts.front());
  for (size_t i = 1; i + 2 < pts.size(); ++i) {
    const Point & p0 = pts[i - 1];
    const Point & p1 = pts[i];
    const Point & p2 = pts[i + 1];
    const Point & p3 = pts[i + 2];
    for (int s = 1; s <= samples_per_segment; ++s) {
      double t = static_cast<double>(s) / samples_per_segment;
      smoothed.push_back(catmull_rom(p0, p1, p2, p3, t));
    }
  }

  bool collision_free = true;
  for (size_t k = 0; k + 1 < smoothed.size(); ++k) {
    Cell a = grid.world_to_grid(smoothed[k].first, smoothed[k].second);
    Cell b = grid.world_to_grid(smoothed[k + 1].first, smoothed[k + 1].second);
    if (!has_line_of_sight(grid, a, b)) {
      collision_free = false;
      break;
    }
  }
  if (collision_free) {return smoothed;}

  std::vector<Point> fallback;
  for (size_t i = 0; i + 1 < world_pts.size(); ++i) {
    const Point & p0 = world_pts[i];
    const Point & p1 = world_pts[i + 1];
    for (int s = 0; s < samples_per_segment; ++s) {
      double t = static_cast<double>(s) / samples_per_segment;
      fallback.push_back({p0.first + (p1.first - p0.first) * t,
          p0.second + (p1.second - p0.second) * t});
    }
  }
  fallback.push_back(world_pts.back());
  return fallback;
}

PlanResult plan(
  const GridMap & grid, const Point & start_world, const Point & goal_world, bool allow_unknown)
{
  auto t0 = std::chrono::steady_clock::now();
  Cell start_cell = grid.world_to_grid(start_world.first, start_world.second);
  Cell goal_cell = grid.world_to_grid(goal_world.first, goal_world.second);
  std::vector<Cell> raw = a_star_search(grid, start_cell, goal_cell, allow_unknown);
  std::vector<Cell> pruned = prune_path(grid, raw);
  std::vector<Point> smoothed = smooth_path_world(grid, pruned);
  auto t1 = std::chrono::steady_clock::now();

  PlanResult result;
  result.points = std::move(smoothed);
  result.raw_cells = std::move(raw);
  result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

double path_length(const std::vector<Point> & points)
{
  double total = 0.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    double dx = points[i + 1].first - points[i].first;
    double dy = points[i + 1].second - points[i].second;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

}  // namespace diff_robot_planner_cpp
