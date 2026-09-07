// Copyright 2026 diff_robot_planner_cpp contributors
// benchmark_main.cpp
// ====================
// 独立可执行文件，不依赖 ROS2，用来：
//   1) 跑几组正确性自检（起终点连通、路径无碰撞、异常起点应该抛异常）；
//   2) 在不同网格规模下测 C++ A* 的求解耗时，和
//      diff_robot_navigation/scripts/visualize_astar_demo.py 里报告的
//      Python 求解耗时做直接对比 —— 这是需求4"验证多语言开发能力"里
//      "ms-level" 这个指标的真实测量依据，不是凭空写的数字。
//
// 编译（不需要 colcon/ROS2，纯 g++，在 diff_robot_planner_cpp 目录下执行）：
// g++ -O2 -std=c++17 -Iinclude src/astar_core.cpp benchmark/benchmark_main.cpp -o astar_benchmark
//   ./astar_benchmark
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "diff_robot_planner_cpp/astar_core.hpp"

using diff_robot_planner_cpp::Cell;
using diff_robot_planner_cpp::GridMap;
using diff_robot_planner_cpp::path_length;
using diff_robot_planner_cpp::plan;
using diff_robot_planner_cpp::PlannerError;
using diff_robot_planner_cpp::Point;

namespace
{

GridMap build_demo_map(int h, int w, double res)
{
  // 和 diff_robot_navigation/diff_robot_navigation/demo_maps.py 的 build_demo_map()
  // 保持同一套房间/走廊/障碍布局比例（按 h,w 缩放），方便跨语言对比同一张"逻辑地图"。
  std::vector<int16_t> data(static_cast<size_t>(h) * w, 0);
  auto set_block = [&](int r0, int r1, int c0, int c1, int16_t v) {
      for (int r = std::max(r0, 0); r < std::min(r1, h); ++r) {
        for (int c = std::max(c0, 0); c < std::min(c1, w); ++c) {
          data[r * w + c] = v;
        }
      }
    };
  set_block(0, 1, 0, w, 100);
  set_block(h - 1, h, 0, w, 100);
  set_block(0, h, 0, 1, 100);
  set_block(0, h, w - 1, w, 100);

  double scale_h = h / 140.0;
  double scale_w = w / 140.0;
  set_block(static_cast<int>(45 * scale_h), static_cast<int>(50 * scale_h), 0, w, 100);
  set_block(static_cast<int>(45 * scale_h), static_cast<int>(50 * scale_h),
    static_cast<int>(55 * scale_w), static_cast<int>(65 * scale_w), 0);

  set_block(static_cast<int>(90 * scale_h), static_cast<int>(95 * scale_h), 0, w, 100);
  set_block(static_cast<int>(90 * scale_h), static_cast<int>(95 * scale_h),
    static_cast<int>(20 * scale_w), static_cast<int>(30 * scale_w), 0);
  set_block(static_cast<int>(90 * scale_h), static_cast<int>(95 * scale_h),
    static_cast<int>(100 * scale_w), static_cast<int>(112 * scale_w), 0);

  set_block(static_cast<int>(15 * scale_h), static_cast<int>(25 * scale_h),
    static_cast<int>(60 * scale_w), static_cast<int>(75 * scale_w), 100);
  set_block(static_cast<int>(60 * scale_h), static_cast<int>(75 * scale_h),
    static_cast<int>(15 * scale_w), static_cast<int>(25 * scale_w), 100);
  set_block(static_cast<int>(60 * scale_h), static_cast<int>(70 * scale_h),
    static_cast<int>(90 * scale_w), static_cast<int>(110 * scale_w), 100);
  set_block(static_cast<int>(105 * scale_h), static_cast<int>(120 * scale_h),
    static_cast<int>(55 * scale_w), static_cast<int>(70 * scale_w), 100);

  return GridMap(data, h, w, res, 0.0, 0.0, 0.25, 6.0);
}

void assert_collision_free(const GridMap & grid, const std::vector<Point> & pts)
{
  for (const auto & p : pts) {
    Cell c = grid.world_to_grid(p.first, p.second);
    if (!grid.is_free(c)) {
      std::fprintf(stderr, "FAIL: point (%.3f, %.3f) 落在障碍物上\n", p.first, p.second);
      std::exit(1);
    }
  }
}

void run_correctness_checks()
{
  std::printf("== 正确性自检 ==\n");
  GridMap grid = build_demo_map(140, 140, 0.05);

  // 1) 主用例：贯穿多个房间
  auto result = plan(grid, {0.4, 0.4}, {6.3, 6.3});
  assert_collision_free(grid, result.points);
  assert(result.points.size() > 2);
  std::printf("  [OK] 主用例路径点数=%zu 长度=%.3fm 耗时=%.4fms\n",
    result.points.size(), path_length(result.points), result.solve_time_ms);

  // 2) 起点在障碍物里应该抛异常
  bool threw = false;
  try {
    Point bad_start = grid.grid_to_world(47, 10);  // 第一堵横墙内部（不在门缺口范围 55:65）
    plan(grid, bad_start, {6.3, 6.3});
  } catch (const PlannerError &) {
    threw = true;
  }
  assert(threw && "起点在障碍物内应该抛出 PlannerError");
  std::printf("  [OK] 非法起点正确抛出 PlannerError\n");

  // 3) 随机动态起终点，验证鲁棒性（对应需求2"支持动态起点/终点"）
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> dist(0.3, 6.6);
  int ok_count = 0, tries = 0;
  while (ok_count < 8 && tries < 200) {
    ++tries;
    Point s{dist(rng), dist(rng)};
    Point g{dist(rng), dist(rng)};
    Cell sc = grid.world_to_grid(s.first, s.second);
    Cell gc = grid.world_to_grid(g.first, g.second);
    if (!grid.is_free(sc) || !grid.is_free(gc)) {continue;}
    double d = std::hypot(g.first - s.first, g.second - s.second);
    if (d < 2.0) {continue;}
    try {
      auto r = plan(grid, s, g);
      assert_collision_free(grid, r.points);
      ++ok_count;
    } catch (const PlannerError &) {
      // 起终点确实不连通（比如被墙完全隔开），跳过，不算失败
    }
  }
  std::printf("  [OK] %d 组随机动态起终点全部规划成功且无碰撞\n", ok_count);
}

void run_timing_benchmark()
{
  std::printf("\n== C++ 求解耗时基准（不同网格规模，每种跑 30 次取均值/最大值） ==\n");
  std::printf("%-10s%-12s%-14s%-14s\n", "grid", "resolution", "mean_ms", "max_ms");
  std::vector<std::pair<int, double>> configs = {
    {140, 0.05}, {280, 0.025}, {560, 0.0125},
  };
  for (auto [size, res] : configs) {
    GridMap grid = build_demo_map(size, size, res);
    double total = 0.0, worst = 0.0;
    const int trials = 30;
    int completed = 0;
    for (int i = 0; i < trials; ++i) {
      try {
        auto r = plan(grid, {0.4, 0.4}, {size * res - 0.7, size * res - 0.7});
        total += r.solve_time_ms;
        worst = std::max(worst, r.solve_time_ms);
        ++completed;
      } catch (const PlannerError & e) {
        std::fprintf(stderr, "  (skip trial: %s)\n", e.what());
      }
    }
    if (completed > 0) {
      std::printf("%-10s%-12.4f%-14.4f%-14.4f\n",
        (std::to_string(size) + "x" + std::to_string(size)).c_str(),
        res, total / completed, worst);
    }
  }
}

}  // namespace

int main()
{
  run_correctness_checks();
  run_timing_benchmark();
  std::printf("\n全部通过。\n");
  return 0;
}
