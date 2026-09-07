// Copyright 2026 diff_robot_planner_cpp contributors
#include "diff_robot_planner_cpp/astar_planner.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace diff_robot_planner_cpp
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".inflation_radius_m", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".inflation_weight", rclcpp::ParameterValue(6.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".inflation_radius_m", inflation_radius_m_);
  node->get_parameter(name_ + ".inflation_weight", inflation_weight_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  RCLCPP_INFO(
    node->get_logger(),
    "AStarPlanner[%s] configured: inflation_radius_m=%.3f inflation_weight=%.2f "
    "allow_unknown=%s (C++ astar_core，见 diff_robot_planner_cpp/benchmark 里的独立耗时基准)",
    name_.c_str(), inflation_radius_m_, inflation_weight_, allow_unknown_ ? "true" : "false");
}

void AStarPlanner::cleanup()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "AStarPlanner[%s] cleanup", name_.c_str());
  }
}

void AStarPlanner::activate()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "AStarPlanner[%s] activate", name_.c_str());
  }
}

void AStarPlanner::deactivate()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "AStarPlanner[%s] deactivate", name_.c_str());
  }
}

GridMap AStarPlanner::costmap_to_gridmap() const
{
  auto * costmap = costmap_ros_->getCostmap();
  int w = static_cast<int>(costmap->getSizeInCellsX());
  int h = static_cast<int>(costmap->getSizeInCellsY());

  // Costmap2D 的 cost 取值约定和 nav_msgs/OccupancyGrid 不一样
  // （FREE_SPACE=0, LETHAL_OBSTACLE=254, INSCRIBED_INFLATED_OBSTACLE=253,
  //  NO_INFORMATION=255），这里换算成 astar_core::GridMap 认的 0~100/-1 约定，
  // 和 Python 版 global_planner_node.py 里从 OccupancyGrid 消息构造 GridMap
  // 的处理方式对齐，两条代码路径最终喂给 A* 的语义是一致的。
  std::vector<int16_t> data(static_cast<size_t>(h) * w);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      unsigned char cost = costmap->getCost(x, y);
      int16_t v;
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        v = OCC_UNKNOWN;
      } else if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        v = 100;
      } else {
        v = 0;
      }
      data[static_cast<size_t>(y) * w + x] = v;
    }
  }

  return GridMap(
    data, h, w, costmap->getResolution(),
    costmap->getOriginX(), costmap->getOriginY(),
    inflation_radius_m_, inflation_weight_);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  auto node = node_.lock();
  path.header.frame_id = global_frame_;
  path.header.stamp = node ? node->now() : rclcpp::Clock().now();

  // A* 是单次阻塞求解，没有可以中途轮询取消的循环；这里只在开始前查一次，
  // 如果调用方（bt_navigator）在下发请求后立刻取消，就不做无意义的计算。
  if (cancel_checker && cancel_checker()) {
    if (node) {
      RCLCPP_INFO(node->get_logger(), "AStarPlanner[%s] 收到取消请求，跳过规划", name_.c_str());
    }
    return path;
  }

  GridMap grid = costmap_to_gridmap();
  Point start_xy{start.pose.position.x, start.pose.position.y};
  Point goal_xy{goal.pose.position.x, goal.pose.position.y};

  try {
    PlanResult result = plan(grid, start_xy, goal_xy, allow_unknown_);
    for (size_t i = 0; i < result.points.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = result.points[i].first;
      ps.pose.position.y = result.points[i].second;
      double yaw = 0.0;
      if (i + 1 < result.points.size()) {
        yaw = std::atan2(
          result.points[i + 1].second - result.points[i].second,
          result.points[i + 1].first - result.points[i].first);
      } else if (i > 0) {
        yaw = std::atan2(
          result.points[i].second - result.points[i - 1].second,
          result.points[i].first - result.points[i - 1].first);
      }
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(ps);
    }
    if (node) {
      RCLCPP_INFO(
        node->get_logger(), "AStarPlanner[%s] 规划成功: %zu 点, 耗时 %.3f ms",
        name_.c_str(), result.points.size(), result.solve_time_ms);
    }
  } catch (const PlannerError & e) {
    if (node) {
      RCLCPP_WARN(node->get_logger(), "AStarPlanner[%s] 规划失败: %s", name_.c_str(), e.what());
    }
    // 按 nav2_core 约定返回空路径，由上层（controller_server/bt_navigator 的
    // 恢复行为）处理规划失败的情况，这里不直接抛异常中断 BT 执行。
  }
  return path;
}

}  // namespace diff_robot_planner_cpp

PLUGINLIB_EXPORT_CLASS(diff_robot_planner_cpp::AStarPlanner, nav2_core::GlobalPlanner)
