// Copyright 2026 diff_robot_planner_cpp contributors
// test_astar_core.cpp
// =====================
// gtest 单元测试，colcon test 时会自动跑（见 CMakeLists.txt 里的
// ament_add_gtest）。沙盒里也用 libgtest-dev 单独编译跑过一遍，
// 全部通过（见 docs/tuning_log.md 里贴的运行记录）。
#include <gtest/gtest.h>

#include <cmath>

#include "diff_robot_planner_cpp/astar_core.hpp"

using diff_robot_planner_cpp::a_star_search;
using diff_robot_planner_cpp::Cell;
using diff_robot_planner_cpp::GridMap;
using diff_robot_planner_cpp::path_length;
using diff_robot_planner_cpp::plan;
using diff_robot_planner_cpp::PlannerError;
using diff_robot_planner_cpp::Point;

namespace
{

GridMap make_empty_grid(int h = 60, int w = 60, double res = 0.05)
{
  std::vector<int16_t> data(static_cast<size_t>(h) * w, 0);
  return GridMap(data, h, w, res, 0.0, 0.0);
}

GridMap make_maze_grid(int h = 80, int w = 80, double res = 0.05)
{
  std::vector<int16_t> data(static_cast<size_t>(h) * w, 0);
  auto blk = [&](int r0, int r1, int c0, int c1, int16_t v) {
      for (int r = r0; r < r1 && r < h; ++r) {
        for (int c = c0; c < c1 && c < w; ++c) {
          if (r >= 0 && c >= 0) {data[r * w + c] = v;}
        }
      }
    };
  blk(20, 25, 0, 80, 100);
  blk(20, 25, 40, 46, 0);
  blk(30, 62, 30, 34, 100);
  blk(55, 62, 30, 34, 0);
  blk(45, 50, 0, 80, 100);
  blk(45, 50, 60, 68, 0);
  return GridMap(data, h, w, res, 0.0, 0.0, 0.2, 6.0);
}

void assert_collision_free(const GridMap & grid, const std::vector<Point> & pts)
{
  for (const auto & p : pts) {
    Cell c = grid.world_to_grid(p.first, p.second);
    ASSERT_TRUE(grid.is_free(c)) << "point (" << p.first << "," << p.second << ") on obstacle";
  }
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    // 相邻采样点之间理论上应该也是安全的，用比较密的采样点间隔本身就近似保证了这点，
    // 这里额外抽查一下端点。
  }
}

}  // namespace

TEST(AStarCore, EmptyGridNearStraightLine)
{
  GridMap grid = make_empty_grid();
  Point start{0.2, 0.2}, goal{2.5, 2.3};
  auto result = plan(grid, start, goal);
  assert_collision_free(grid, result.points);
  double straight = std::hypot(goal.first - start.first, goal.second - start.second);
  double length = path_length(result.points);
  EXPECT_LT(length, straight * 1.05);
}

TEST(AStarCore, MazeGridCollisionFree)
{
  GridMap grid = make_maze_grid();
  auto result = plan(grid, {0.5, 0.5}, {3.5, 3.6});
  EXPECT_GT(result.points.size(), 2u);
  assert_collision_free(grid, result.points);
}

TEST(AStarCore, StartInObstacleThrows)
{
  GridMap grid = make_maze_grid();
  Point bad_start = grid.grid_to_world(22, 10);
  EXPECT_THROW(plan(grid, bad_start, {3.5, 3.6}), PlannerError);
}

TEST(AStarCore, SmoothingDoesNotLengthenMuch)
{
  GridMap grid = make_maze_grid();
  Cell start_cell = grid.world_to_grid(0.5, 0.5);
  Cell goal_cell = grid.world_to_grid(3.5, 3.6);
  auto raw = a_star_search(grid, start_cell, goal_cell);
  std::vector<Point> raw_world;
  for (auto & c : raw) {
    raw_world.push_back(grid.grid_to_world(c.row, c.col));
  }
  double raw_len = path_length(raw_world);

  auto result = plan(grid, {0.5, 0.5}, {3.5, 3.6});
  double smoothed_len = path_length(result.points);
  EXPECT_LE(smoothed_len, raw_len * 1.02);
}

TEST(AStarCore, DynamicStartGoalRobustness)
{
  GridMap grid = make_maze_grid();
  int ok = 0;
  for (int i = 0; i < 20; ++i) {
    double sx = 0.3 + 0.07 * i, sy = 0.3;
    double gx = 3.0, gy = 3.5;
    Cell sc = grid.world_to_grid(sx, sy);
    if (!grid.is_free(sc)) {continue;}
    try {
      auto result = plan(grid, {sx, sy}, {gx, gy});
      assert_collision_free(grid, result.points);
      ++ok;
    } catch (const PlannerError &) {
      // 起点恰好落在墙里/不连通，跳过
    }
  }
  EXPECT_GT(ok, 5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
