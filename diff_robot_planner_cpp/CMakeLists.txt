cmake_minimum_required(VERSION 3.8)
project(diff_robot_planner_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

# astar_core 是静态库，但后面会被链接进 diff_robot_planner_cpp_plugin 这个
# SHARED 插件库里；静态库默认不带 -fPIC，链接进 .so 时会报
# "relocation ... can not be used when making a shared object; recompile
# with -fPIC"。开这个全局开关让所有目标（包括 astar_core 和纯 g++ 也能独立
# 编译的 astar_benchmark）都带 -fPIC，不影响可执行文件，最简单可靠。
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# astar_core: 纯标准库的核心算法，不依赖任何 ROS2 包。
# 独立于下面的 Nav2 插件目标存在，这样即使在没有装 Nav2 的机器上，
# 也能单独编译 astar_core + benchmark 跑求解耗时基准
# （沙盒里就是这么验证的：g++ -O2 -std=c++17 -I include src/astar_core.cpp
#   benchmark/benchmark_main.cpp -o astar_benchmark && ./astar_benchmark）。
# ---------------------------------------------------------------------------
add_library(astar_core src/astar_core.cpp)
target_include_directories(astar_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(astar_benchmark benchmark/benchmark_main.cpp)
target_link_libraries(astar_benchmark astar_core)

# ---------------------------------------------------------------------------
# nav2_core::GlobalPlanner 插件：需要 ROS2 Humble + Nav2 的头文件才能编译，
# 这部分在云端沙盒（没有 ROS2）里没有实际编译验证过，见 astar_planner.hpp
# 顶部注释。落地时如果这几个 find_package 失败，说明 Nav2 没装全，
# 先确认 `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`。
# ---------------------------------------------------------------------------
find_package(rclcpp QUIET)
find_package(rclcpp_lifecycle QUIET)
find_package(nav2_core QUIET)
find_package(nav2_costmap_2d QUIET)
find_package(nav2_util QUIET)
find_package(nav_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(tf2_ros QUIET)
find_package(pluginlib QUIET)

if(rclcpp_FOUND AND rclcpp_lifecycle_FOUND AND nav2_core_FOUND AND
  nav2_costmap_2d_FOUND AND nav2_util_FOUND AND nav_msgs_FOUND AND
  geometry_msgs_FOUND AND tf2_ros_FOUND AND pluginlib_FOUND)
  message(STATUS "diff_robot_planner_cpp: 检测到 Nav2 依赖，编译 AStarPlanner 插件")

  add_library(diff_robot_planner_cpp_plugin SHARED src/astar_planner.cpp)
  target_include_directories(diff_robot_planner_cpp_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(diff_robot_planner_cpp_plugin astar_core)
  ament_target_dependencies(diff_robot_planner_cpp_plugin
    rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util
    nav_msgs geometry_msgs tf2_ros pluginlib)
  pluginlib_export_plugin_description_file(nav2_core diff_robot_planner_cpp_plugin.xml)

  install(TARGETS diff_robot_planner_cpp_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  message(WARNING
    "diff_robot_planner_cpp: 未检测到完整的 Nav2 依赖，跳过 AStarPlanner 插件编译，"
    "只编译不依赖 ROS2 的 astar_core + astar_benchmark。装好 ROS2 Humble + Nav2 "
    "后重新 colcon build 即可编出插件。")
endif()

install(TARGETS astar_core astar_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_astar_core test/test_astar_core.cpp)
    target_link_libraries(test_astar_core astar_core)
  endif()
endif()

ament_package()
