# 运行步骤 (RUN_STEPS)

目标环境：**Ubuntu 24.04 + ROS2 Jazzy + Gazebo (gz sim / Harmonic，随 ROS2 Jazzy desktop-full 一起装)**。

> 这份步骤是在真实 Ubuntu24.04+Jazzy+Gazebo 机器上跑的操作说明。仓库里能脱离 ROS2 独立验证的部分（A* 算法本身、Pure Pursuit/MPC 离线调参、C++ 核心 benchmark）已经在开发过程中跑过、结果记在 `docs/tuning_log.md` 里；下面第0步是那部分的复现命令，第1步开始是需要真实 ROS2/Gazebo 环境的部分。

## 第0步：不需要 ROS2 就能跑的部分（建议先做，验证算法本身没问题）

```bash
cd diff_robot_ws/src/diff_robot_navigation

# 装依赖（只要 numpy/scipy/matplotlib，不需要 ROS2）
pip install numpy scipy matplotlib --break-system-packages

# A* 单元测试
python3 -m pytest test/ -v
# 或不装 pytest 直接跑：
python3 test/test_astar.py
python3 test/test_control_models.py

# A* 可视化 demo（多组动态起终点，出图到 docs/figures/astar_demo.png）
python3 scripts/visualize_astar_demo.py --out ../../docs/figures/astar_demo.png --dynamic-goals 5

# Pure Pursuit / MPC 离线调参 + 对比（几十秒，出 CSV+图到 docs/）
python3 scripts/tune_and_compare.py --out-dir ../../docs

# Python vs C++ A* 求解耗时对比（Python 这一侧）
python3 scripts/benchmark_python_astar.py
```

```bash
cd diff_robot_ws/src/diff_robot_planner_cpp
g++ -O2 -std=c++17 -I include src/astar_core.cpp benchmark/benchmark_main.cpp -o astar_benchmark
./astar_benchmark   # C++ 那一侧的求解耗时 + 正确性自检

# 如果装了 libgtest-dev，还可以跑 gtest 单元测试：
g++ -O2 -std=c++17 -I include src/astar_core.cpp test/test_astar_core.cpp \
    -lgtest -lgtest_main -lpthread -o test_astar_core
./test_astar_core
```

## 第1步：装 ROS2 Jazzy + Gazebo + Nav2 相关依赖

```bash
# 假设已经装好 ROS2 Jazzy desktop-full（自带 gz sim / ros_gz）
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  python3-colcon-common-extensions

pip install numpy scipy --break-system-packages   # controller 节点要用
```

## 第2步：建工作空间、拉代码、colcon build

```bash
mkdir -p ~/diff_robot_ws/src
cd ~/diff_robot_ws/src
git clone https://github.com/layingdown/diff_robot.git diff_robot_description
# diff_robot_navigation、diff_robot_planner_cpp 是本次新增的两个包，
# 如果和 diff_robot_description 在同一个仓库/同一次提交里，clone 一次就都有了；
# 如果是分开的 PR/分支，按实际情况把这两个目录也放进 src/ 下。

cd ~/diff_robot_ws
colcon build --symlink-install
source install/setup.bash
```

`diff_robot_planner_cpp` 在 `find_package` 检测不到完整 Nav2 依赖时会自动跳过插件编译、只编译不依赖 ROS2 的 `astar_core`+`astar_benchmark`（见该包 CMakeLists.txt 里的 QUIET 检测逻辑），所以就算 Nav2 没装全，`colcon build` 也不会因为这一个包直接失败。

## 第3步：建图 (SLAM)

```bash
ros2 launch diff_robot_navigation mapping.launch.py
```
Gazebo 起来后，用 RViz 里的工具或者键盘遥控节点（`ros2 run teleop_twist_keyboard teleop_twist_keyboard`，需要额外装）把机器人开一圈，把 `slam_test_world.world` 里的房间都探索到。确认 RViz 里 `/map` 话题的地图基本闭合、没有大片未知区域之后，保存地图：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/diff_robot_map
```

会生成 `~/maps/diff_robot_map.yaml` 和 `~/maps/diff_robot_map.pgm`。也可以直接用仓库里 `diff_robot_navigation/maps/` 下预留的地图（如果已经放了跑好的地图文件的话）。

## 第4步：完整自主导航闭环（全局规划 + Nav2/DWB 局部避障）

```bash
ros2 launch diff_robot_navigation navigation.launch.py map:=$HOME/maps/diff_robot_map.yaml
```

RViz 打开后：
1. 用工具栏 "2D Pose Estimate" 给 AMCL 一个初始位姿估计（如果机器人还在建图时的出生点，直接给 (0,0,0) 附近就行）。
2. 用 "2D Nav Goal" 点一个目标点——这会触发 bt_navigator 调用我们的 `global_planner_node`（A*）算全局路径，再用 Nav2 controller_server 的 DWB 局部规划器边走边避障（激光扫到的动态障碍物也能实时避开，不只是地图里的静态障碍物）。
3. 换几个不同的起终点（多点几次 "2D Nav Goal"）验证"动态起点/终点"这个需求点。
4. 在世界文件里加几个 Gazebo 里的临时障碍物（或者直接在 Gazebo GUI 里拖一个 box 模型进场景），验证 DWB 对未建过图的障碍物也能避开。

## 第5步：Pure Pursuit vs MPC 单独对比实验

```bash
# 先跑 Pure Pursuit
ros2 launch diff_robot_navigation compare_controllers.launch.py \
    controller:=pure_pursuit map:=$HOME/maps/diff_robot_map.yaml goal_x:=6.0 goal_y:=6.0

# 记录/画完图之后，Ctrl+C 关掉，换 MPC 再跑一次同样的起终点：
ros2 launch diff_robot_navigation compare_controllers.launch.py \
    controller:=mpc map:=$HOME/maps/diff_robot_map.yaml goal_x:=6.0 goal_y:=6.0
```

跑的时候建议开 `rqt_plot` 或者 `ros2 topic echo` 记录这些话题，方便和 `docs/tuning_log.md` 里的仿真数据对照：
```bash
ros2 topic echo /odometry/filtered --field pose.pose.position
ros2 topic echo /cmd_vel
ros2 topic echo /mpc_controller/solve_time_ms   # 只有 MPC 模式下有
```

如果想要更严谨的量化对比（不是靠肉眼看 RViz），可以用 `ros2 bag record /odometry/filtered /plan /cmd_vel` 把这两次跑的数据录下来，之后写个小脚本（复用 `control_models.py` 里的 `signed_lateral_error` 逻辑）离线算真机 RMSE，填进 `docs/tuning_log.md` 第6节的表格。

## 第6步：C++ 全局规划器插件（可选，替换 Python 版）

如果想用 `diff_robot_planner_cpp` 里的 `AStarPlanner` 插件替代 `global_planner_node.py`：

1. 确认 `colcon build` 已经成功编出了 `diff_robot_planner_cpp_plugin`（`ros2 pkg prefix diff_robot_planner_cpp` 下应该能看到 `lib/libdiff_robot_planner_cpp_plugin.so`）。
2. 在 `diff_robot_navigation/config/nav2_params.yaml` 里加一段标准的 `planner_server` 配置（这个仓库默认没写这段，因为默认用的是 Python 版 `global_planner_node.py`）：
   ```yaml
   planner_server:
     ros__parameters:
       planner_plugins: ["GridBased"]
       GridBased:
         plugin: "diff_robot_planner_cpp/AStarPlanner"
         inflation_radius_m: 0.35
         inflation_weight: 6.0
   ```
3. 把 `navigation.launch.py` 里手写的 `global_planner_node` 那个 `Node(...)` 换成标准的 `nav2_planner::PlannerServer`（`package='nav2_planner', executable='planner_server'`），并把它加进 `lifecycle_manager` 的 `node_names` 列表（这次是标准 Nav2 C++ 节点，支持 lifecycle，和 Python 版不一样）。
4. 重新 `colcon build` + `source install/setup.bash`，重复第4步。

两种方案（Python 节点 / C++ 插件）二选一即可，默认仓库配置用的是 Python 节点（集成更简单，不需要改 nav2_params.yaml），C++ 插件是"验证多语言开发能力 + 追求更低求解耗时"时的进阶选项。

## 第7步：录制演示视频

建议至少录这几段，剪成一个演示视频：
1. 建图过程（RViz 里地图逐渐显现）；
2. 完整导航闭环：给一个目标点，看着机器人规划路径、边走边避开激光扫到的临时障碍物、到达终点；
3. 换 2-3 组不同起终点，展示"动态起点/终点"；
4. Pure Pursuit 和 MPC 各跑一遍同样的路径，最好用分屏或者字幕标出两者的差异（参考 `docs/tuning_log.md` 第4节的量化结论）；
5.（可选）`astar_benchmark` / `test_astar_core` 在终端里跑一遍，展示 C++ 版本的正确性自检和求解耗时。

录屏工具随意（`gnome-screenshot`/`SimpleScreenRecorder`/`OBS Studio` 都可以），Gazebo/RViz 画面 + 一个显示当前话题数据的终端窗口（比如 `ros2 topic echo /cmd_vel`）放一起会比较有说服力。
