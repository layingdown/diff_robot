// Copyright 2026 diff_robot_planner_cpp contributors
// astar_core.hpp
// ================
// diff_robot_navigation/diff_robot_navigation/astar.py 的 C++ 逐行对照移植
// （需求4："使用 C++ 重构 A* 核心模块，验证多语言开发能力"）。
//
// 不依赖任何 ROS2 / rclcpp 头文件，纯标准库 + Eigen 都不需要，方便：
//   1) 单独用 g++ 编译、跑 benchmark/benchmark_main.cpp 里的求解耗时对比；
//   2) 被 astar_planner.hpp/.cpp （nav2_core::GlobalPlanner 插件）复用，
//      插件那一层只负责跟 Nav2 costmap/plugin 接口打交道，转换成 GridMap
//      之后调用这里的纯算法函数——和 Python 那边 global_planner_node.py
//      调用 astar.py 的分层方式完全对称。
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace diff_robot_planner_cpp
{

using Point = std::pair<double, double>;  // (x, y) 世界坐标，单位米

struct Cell
{
  int row;
  int col;
  bool operator==(const Cell & o) const {return row == o.row && col == o.col;}
};

struct CellHash
{
  std::size_t operator()(const Cell & c) const noexcept
  {
    return (static_cast<std::size_t>(c.row) << 32) ^ static_cast<std::size_t>(c.col);
  }
};

constexpr int8_t OCC_UNKNOWN = -1;
constexpr int16_t OCC_FREE_MAX = 50;

// 规划失败异常：起点/终点非法、或搜索空间内不存在可行路径。
// 对应 Python 版本的 PlannerError。
class PlannerError : public std::runtime_error
{
public:
  explicit PlannerError(const std::string & msg)
  : std::runtime_error(msg) {}
};

class GridMap
{
public:
  GridMap(
    std::vector<int16_t> data, int height, int width, double resolution,
    double origin_x, double origin_y,
    double inflation_radius_m = 0.35, double inflation_weight = 6.0);

  Cell world_to_grid(double x, double y) const;
  Point grid_to_world(int row, int col) const;

  bool in_bounds(const Cell & c) const;
  bool is_free(const Cell & c, bool allow_unknown = false) const;

  // 障碍物膨胀代价场（多源 BFS 求到最近障碍物的 8 邻域距离，
  // 和 astar.py 里 scipy 不可用时的退化实现思路一致：不要求逐 bit 对齐
  // Python 版的 scipy EDT 结果，只要求"离墙越远代价越低"这个单调性质，
  // 两边独立实现、独立验证，这也是"用两种语言分别做一遍"的意义所在）。
  const std::vector<double> & inflation_cost_field() const;

  int height() const {return height_;}
  int width() const {return width_;}
  double resolution() const {return resolution_;}

  const std::vector<int16_t> & data() const {return data_;}

private:
  std::vector<int16_t> data_;
  int height_;
  int width_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  double inflation_radius_m_;
  double inflation_weight_;
  mutable std::vector<double> cost_field_;
  mutable bool cost_field_computed_ = false;
};

struct PlanResult
{
  std::vector<Point> points;      // 平滑后的世界坐标路径
  std::vector<Cell> raw_cells;    // 原始 A* 栅格路径（未平滑），用于调试/评测
  double solve_time_ms = 0.0;
};

// 标准 8 邻域 A*，欧氏距离启发式，返回栅格坐标路径（含起终点）。
std::vector<Cell> a_star_search(
  const GridMap & grid, const Cell & start, const Cell & goal,
  bool allow_unknown = false, int max_expansions = 400000);

// 视线法裁剪
std::vector<Cell> prune_path(const GridMap & grid, const std::vector<Cell> & path);

// Catmull-Rom 样条平滑 + 逐段碰撞检测，穿墙则回退到线性插值
std::vector<Point> smooth_path_world(
  const GridMap & grid, const std::vector<Cell> & pruned_path, int samples_per_segment = 12);

// 一步到位 API：世界坐标起终点 -> A* -> 裁剪 -> 平滑
PlanResult plan(
  const GridMap & grid, const Point & start_world, const Point & goal_world,
  bool allow_unknown = false);

double path_length(const std::vector<Point> & points);

}  // namespace diff_robot_planner_cpp
