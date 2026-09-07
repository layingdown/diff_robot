#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_control_models.py
=======================
纯离线单元测试（`python3 -m pytest test/`），验证 Pure Pursuit 和 MPC 两个
控制器核心算法本身是正确、收敛、稳定的，独立于 ROS2 节点封装：
  1. 两个控制器在一条直线路径上都应该能在合理时间内到达终点；
  2. 到达终点后跟踪误差应该很小（不能"差不多就行"地停在半路）；
  3. 角速度指令不应超过参数里设的 max_angular_vel 上限；
  4. MPC 单步求解耗时应该在毫秒级（对应需求里 "solve < 15ms" 的约束）。
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from diff_robot_navigation.control_models import (  # noqa: E402
    PurePursuitController, PurePursuitParams,
    LinearMPCController, MPCParams,
    unicycle_step,
)

STRAIGHT_PATH = [(0.0, 0.0), (1.0, 0.2), (2.0, 0.5), (3.5, 1.2), (5.0, 1.5)]


def _run(controller, start_state, dt=0.1, max_time=40.0, max_omega=None):
    state = np.array(start_state, dtype=float)
    t = 0.0
    solve_times = []
    while t < max_time:
        import time
        t0 = time.perf_counter()
        v, omega = controller.compute_control(state)
        solve_times.append((time.perf_counter() - t0) * 1000.0)
        if max_omega is not None:
            assert abs(omega) <= max_omega + 1e-6, f"omega {omega} 超过上限 {max_omega}"
        state = unicycle_step(state, v, omega, dt)
        t += dt
        if controller.is_goal_reached(state):
            return state, t, solve_times
    return state, t, solve_times


def test_pure_pursuit_reaches_goal():
    params = PurePursuitParams(lookahead_distance=0.5, cruise_linear_vel=0.5,
                                max_angular_vel=1.5)
    ctrl = PurePursuitController(STRAIGHT_PATH, params)
    final_state, t, _ = _run(ctrl, (0.0, 0.0, 0.0), max_omega=1.5)
    goal = STRAIGHT_PATH[-1]
    err = math.hypot(final_state[0] - goal[0], final_state[1] - goal[1])
    assert err < params.goal_tolerance + 0.02, f"未收敛到终点, 剩余误差 {err:.3f}"
    assert t < 40.0, "Pure Pursuit 超时未到达终点"


def test_mpc_reaches_goal_and_is_fast_enough():
    params = MPCParams(horizon=10, dt=0.1, cruise_linear_vel=0.5, max_angular_vel=1.5)
    ctrl = LinearMPCController(STRAIGHT_PATH, params)
    final_state, t, solve_times = _run(ctrl, (0.0, 0.0, 0.0), max_omega=1.5)
    goal = STRAIGHT_PATH[-1]
    err = math.hypot(final_state[0] - goal[0], final_state[1] - goal[1])
    assert err < params.goal_tolerance + 0.02, f"未收敛到终点, 剩余误差 {err:.3f}"
    assert t < 40.0, "MPC 超时未到达终点"
    mean_solve = float(np.mean(solve_times))
    assert mean_solve < 15.0, f"MPC 平均求解耗时 {mean_solve:.2f}ms 超过 15ms 预算"


def test_mpc_and_pure_pursuit_agree_reasonably_on_easy_path():
    """两者在同一条简单路径上跑完，终点误差应该都很小且量级接近（互相印证没有算法错误）。"""
    pp = PurePursuitController(STRAIGHT_PATH, PurePursuitParams(cruise_linear_vel=0.4))
    mpc = LinearMPCController(STRAIGHT_PATH, MPCParams(cruise_linear_vel=0.4, horizon=10))
    s_pp, _, _ = _run(pp, (0.0, 0.0, 0.0))
    s_mpc, _, _ = _run(mpc, (0.0, 0.0, 0.0))
    goal = STRAIGHT_PATH[-1]
    err_pp = math.hypot(s_pp[0] - goal[0], s_pp[1] - goal[1])
    err_mpc = math.hypot(s_mpc[0] - goal[0], s_mpc[1] - goal[1])
    assert err_pp < 0.15 and err_mpc < 0.15


if __name__ == '__main__':
    test_pure_pursuit_reaches_goal()
    test_mpc_reaches_goal_and_is_fast_enough()
    test_mpc_and_pure_pursuit_agree_reasonably_on_easy_path()
    print("All control_models tests passed.")
