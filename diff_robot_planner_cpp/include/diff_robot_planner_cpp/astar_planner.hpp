// Copyright 2026 diff_robot_planner_cpp contributors
// astar_planner.hpp
// ===================
// nav2_core::GlobalPlanner 插件：把 astar_core 包装成一个可以被 Nav2
// planner_server 通过 pluginlib 加载的"正式"全局规划器插件。
//
// 这是需求4"使用 C++ 重构 A* 核心模块"里更贴近生产环境的落地方式：
// diff_robot_navigation/global_planner_node.py 用一个普通 rclpy 节点顶替
// planner_server（因为 nav2_core 插件接口是 C++ pluginlib，Python 官方不支持），
// 而这里则是把同一套算法（C++ 版本）做成"正规军"插件，可以在 nav2_params.yaml 里
// 用标准方式配置：
//     planner_server:
//       ros__parameters:
//         planner_plugins: ["GridBased"]
//         GridBased:
//           plugin: "diff_robot_planner_cpp/AStarPlanner"
//           inflation_radius_m: 0.35
//           inflation_weight: 6.0
// 如果想用这个插件而不是 Python 版 global_planner_node.py，把
// navigation.launch.py 里的自定义 planner_server 节点换成标准的
// `nav2_planner::PlannerServer` + 上面这段参数即可，二者二选一，不需要同时跑。
//
// 注意：本文件依赖 nav2_core / nav2_costmap_2d / rclcpp_lifecycle / pluginlib
// 等 ROS2 Humble 头文件，只能在装好 ROS2 Humble + Nav2 的机器上用 colcon 编译，
// 这个云端沙盒里没有 ROS2，所以这部分代码**没有**在本地编译验证过；
// 真正做到"独立编译通过"验证的是 astar_core.cpp/.hpp（纯标准库，见
// benchmark/benchmark_main.cpp 的编译+运行记录，docs/tuning_log.md 里有实测耗时）。
// 落地时如果 API 细节因 Nav2 小版本差异对不上，对照
// `ros2 pkg prefix nav2_navfn_planner` 或 nav2_smac_planner 的源码修正一下签名即可，
// 核心算法逻辑（astar_core）不用动。
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "diff_robot_planner_cpp/astar_core.hpp"

namespace diff_robot_planner_cpp
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // 注意：这个三参数签名（多了 cancel_checker）是 Nav2 从 Iron/Jazzy 开始的
  // nav2_core::GlobalPlanner 接口；ROS2 Humble 上 nav2_core 的 createPlan
  // 只有 (start, goal) 两个参数。这台机器装的是 ROS2 Jazzy，所以按三参数写。
  // 如果换到 Humble 上编译，把这里和 .cpp 里的签名都去掉第三个参数即可，
  // 核心算法逻辑（astar_core）不用动。
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  GridMap costmap_to_gridmap() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  std::string global_frame_;

  double inflation_radius_m_ = 0.35;
  double inflation_weight_ = 6.0;
  bool allow_unknown_ = false;
};

}  // namespace diff_robot_planner_cpp
