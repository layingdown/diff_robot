#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tune_and_compare.py
====================
不依赖 ROS2 / Gazebo 的离线整定脚本，用一个差速驱动运动学仿真器（unicycle_step）
把 Pure Pursuit 和 MPC 两套控制器接到同一条 A* 规划+平滑出来的参考路径上跑闭环，
扫描关键参数（前视距离 / MPC 预测步数、最大线速度），记录跟踪误差 RMSE、
最大误差、角速度"抖动"（用来衡量"平稳无振荡"）、单步求解耗时，
产出 CSV + 对比图，直接喂给 docs/tuning_log.md。

注意：这里的仿真器是理想 unicycle 运动学模型 + 高斯过程噪声，用来做算法层面的
参数整定和相对对比（Pure Pursuit vs MPC 谁更抖、谁误差更大），
不能替代 Gazebo 里真实的轮式动力学、电机响应延迟、传感器噪声。
真正的收敛调参仍然要在 Gazebo 里跑一遍确认，本脚本给出的是"起点参数"和
"两种算法的相对表现趋势"，可以显著减少在 Gazebo 里从头盲调的次数。

用法：
    python3 scripts/tune_and_compare.py --out-dir /path/to/docs
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import asdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from diff_robot_navigation.astar import plan as astar_plan  # noqa: E402
from diff_robot_navigation.demo_maps import build_demo_map  # noqa: E402
from diff_robot_navigation.control_models import (  # noqa: E402
    PurePursuitController, PurePursuitParams,
    LinearMPCController, MPCParams,
    unicycle_step, evaluate_tracking,
)


def signed_lateral_error(path_xy, headings, state):
    """找最近参考点，返回带符号的横向偏差（左正右负），比欧氏距离更能体现"贴线"程度。"""
    x, y = state[0], state[1]
    d2 = [(px - x) ** 2 + (py - y) ** 2 for px, py in path_xy]
    i = int(np.argmin(d2))
    th = headings[i]
    px, py = path_xy[i]
    dx, dy = x - px, y - py
    lateral = -math.sin(th) * dx + math.cos(th) * dy
    return lateral, math.sqrt(d2[i])


def simulate(controller, path_xy, headings, start_state, dt, max_time=60.0,
             obstacle_check_fn=None, process_noise_std=(0.0, 0.0)):
    state = np.array(start_state, dtype=float)
    t = 0.0
    lateral_errors, dist_errors, omegas, solve_ms = [], [], [], []
    collided = False
    steps = int(max_time / dt)
    for _ in range(steps):
        t0 = time.perf_counter()
        v, omega = controller.compute_control(state)
        solve_ms.append((time.perf_counter() - t0) * 1000.0)

        lateral, dist = signed_lateral_error(path_xy, headings, state)
        lateral_errors.append(lateral)
        dist_errors.append(dist)
        omegas.append(omega)

        v_noisy = v + np.random.normal(0, process_noise_std[0])
        omega_noisy = omega + np.random.normal(0, process_noise_std[1])
        state = unicycle_step(state, v_noisy, omega_noisy, dt)
        t += dt

        if obstacle_check_fn is not None and obstacle_check_fn(state):
            collided = True
            break
        if controller.is_goal_reached(state):
            break

    metrics = evaluate_tracking(dist_errors, omegas, dt, t, solve_ms, collided)
    return metrics, {
        'lateral_errors': lateral_errors, 'omegas': omegas, 'time_to_goal': t,
    }


def get_reference_path():
    grid = build_demo_map()
    start = (0.4, 0.4)
    goal = (6.3, 6.3)
    points = astar_plan(grid, start, goal)
    from diff_robot_navigation.control_models import resample_path, path_headings
    dense = resample_path(points, spacing=0.03)
    headings = path_headings(dense)
    return dense, headings


def sweep_pure_pursuit(path_xy, headings, out_dir):
    rows = []
    lookaheads = [0.25, 0.4, 0.6, 0.9, 1.2, 1.6]
    for la in lookaheads:
        params = PurePursuitParams(lookahead_distance=la, cruise_linear_vel=0.6,
                                    max_linear_vel=1.0, max_angular_vel=1.8)
        ctrl = PurePursuitController(path_xy, params)
        metrics, _ = simulate(ctrl, path_xy, headings, (path_xy[0][0], path_xy[0][1], headings[0]),
                               dt=0.1, process_noise_std=(0.01, 0.01))
        row = {'lookahead_distance': la, **asdict(metrics)}
        rows.append(row)
        print(f"[PurePursuit] L={la:.2f}  RMSE={metrics.rmse:.4f}  "
              f"max_err={metrics.max_error:.4f}  jerk={metrics.mean_abs_omega_jerk:.4f}  "
              f"t2g={metrics.time_to_goal:.2f}s  solve={metrics.solve_time_ms_mean:.4f}ms")

    _write_csv(os.path.join(out_dir, 'pure_pursuit_lookahead_sweep.csv'), rows)

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(lookaheads, [r['rmse'] for r in rows], 'o-')
    axes[0].set_xlabel('lookahead distance (m)')
    axes[0].set_ylabel('tracking RMSE (m)')
    axes[0].set_title('Pure Pursuit: lookahead vs RMSE')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(lookaheads, [r['mean_abs_omega_jerk'] for r in rows], 'o-', color='tab:orange')
    axes[1].set_xlabel('lookahead distance (m)')
    axes[1].set_ylabel('mean |Δomega|/dt (rad/s^2)')
    axes[1].set_title('Pure Pursuit: lookahead vs oscillation')
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'figures', 'pure_pursuit_lookahead_sweep.png'), dpi=150)
    plt.close(fig)
    return rows


def sweep_mpc(path_xy, headings, out_dir):
    rows = []
    horizons = [4, 6, 8, 10, 12, 16, 20]
    for N in horizons:
        params = MPCParams(horizon=N, dt=0.1, cruise_linear_vel=0.6,
                            max_linear_vel=1.0, max_angular_vel=1.8)
        ctrl = LinearMPCController(path_xy, params)
        metrics, _ = simulate(ctrl, path_xy, headings, (path_xy[0][0], path_xy[0][1], headings[0]),
                               dt=0.1, process_noise_std=(0.01, 0.01))
        row = {'horizon': N, **asdict(metrics)}
        rows.append(row)
        print(f"[MPC] N={N:<3d}  RMSE={metrics.rmse:.4f}  max_err={metrics.max_error:.4f}  "
              f"jerk={metrics.mean_abs_omega_jerk:.4f}  t2g={metrics.time_to_goal:.2f}s  "
              f"solve={metrics.solve_time_ms_mean:.4f}ms (max {metrics.solve_time_ms_max:.4f}ms)")

    _write_csv(os.path.join(out_dir, 'mpc_horizon_sweep.csv'), rows)

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(horizons, [r['rmse'] for r in rows], 'o-', color='tab:green')
    axes[0].set_xlabel('MPC horizon N')
    axes[0].set_ylabel('tracking RMSE (m)')
    axes[0].set_title('MPC: horizon vs RMSE')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(horizons, [r['solve_time_ms_mean'] for r in rows], 'o-', color='tab:red',
                 label='mean')
    axes[1].plot(horizons, [r['solve_time_ms_max'] for r in rows], 's--', color='tab:red',
                 alpha=0.5, label='max')
    axes[1].axhline(15.0, color='k', linestyle=':', linewidth=1, label='15 ms budget')
    axes[1].set_xlabel('MPC horizon N')
    axes[1].set_ylabel('solve time (ms)')
    axes[1].set_title('MPC: horizon vs solve time')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'figures', 'mpc_horizon_sweep.png'), dpi=150)
    plt.close(fig)
    return rows


def compare_at_speeds(path_xy, headings, out_dir, best_lookahead, best_horizon, n_seeds=6):
    """每个巡航速度跑 n_seeds 组不同过程噪声种子，取均值±标准差，
    避免单次仿真里的随机噪声让"谁更准"这个结论看起来像噪声而不是趋势。"""
    speeds = [0.3, 0.5, 0.7, 0.9, 1.1, 1.3]
    rows = []
    for v_cruise in speeds:
        pp_rmses, mpc_rmses, pp_jerks, mpc_jerks, mpc_solves = [], [], [], [], []
        for seed in range(n_seeds):
            np.random.seed(1000 * seed + int(v_cruise * 100))

            pp_params = PurePursuitParams(lookahead_distance=best_lookahead,
                                           cruise_linear_vel=v_cruise, max_linear_vel=1.4,
                                           max_angular_vel=2.2)
            pp = PurePursuitController(path_xy, pp_params)
            pp_metrics, _ = simulate(pp, path_xy, headings,
                                      (path_xy[0][0], path_xy[0][1], headings[0]),
                                      dt=0.1, process_noise_std=(0.01, 0.01))

            mpc_params = MPCParams(horizon=best_horizon, dt=0.1, cruise_linear_vel=v_cruise,
                                    max_linear_vel=1.4, max_angular_vel=2.2)
            mpc = LinearMPCController(path_xy, mpc_params)
            mpc_metrics, _ = simulate(mpc, path_xy, headings,
                                       (path_xy[0][0], path_xy[0][1], headings[0]),
                                       dt=0.1, process_noise_std=(0.01, 0.01))

            pp_rmses.append(pp_metrics.rmse)
            mpc_rmses.append(mpc_metrics.rmse)
            pp_jerks.append(pp_metrics.mean_abs_omega_jerk)
            mpc_jerks.append(mpc_metrics.mean_abs_omega_jerk)
            mpc_solves.append(mpc_metrics.solve_time_ms_mean)

        pp_rmse_mean, pp_rmse_std = float(np.mean(pp_rmses)), float(np.std(pp_rmses))
        mpc_rmse_mean, mpc_rmse_std = float(np.mean(mpc_rmses)), float(np.std(mpc_rmses))
        improve = (pp_rmse_mean - mpc_rmse_mean) / pp_rmse_mean * 100.0
        rows.append({
            'cruise_speed_mps': v_cruise, 'n_seeds': n_seeds,
            'pp_rmse_mean': pp_rmse_mean, 'pp_rmse_std': pp_rmse_std,
            'mpc_rmse_mean': mpc_rmse_mean, 'mpc_rmse_std': mpc_rmse_std,
            'rmse_improvement_pct': improve,
            'pp_jerk_mean': float(np.mean(pp_jerks)), 'mpc_jerk_mean': float(np.mean(mpc_jerks)),
            'mpc_solve_ms_mean': float(np.mean(mpc_solves)),
        })
        print(f"[Compare] v={v_cruise:.2f} m/s (n={n_seeds})  "
              f"PP_RMSE={pp_rmse_mean:.4f}±{pp_rmse_std:.4f}  "
              f"MPC_RMSE={mpc_rmse_mean:.4f}±{mpc_rmse_std:.4f}  improve={improve:+.1f}%  "
              f"MPC_solve={float(np.mean(mpc_solves)):.3f}ms")

    _write_csv(os.path.join(out_dir, 'pure_pursuit_vs_mpc_by_speed.csv'), rows)

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].errorbar(speeds, [r['pp_rmse_mean'] for r in rows],
                      yerr=[r['pp_rmse_std'] for r in rows], fmt='o-', label='Pure Pursuit',
                      capsize=3)
    axes[0].errorbar(speeds, [r['mpc_rmse_mean'] for r in rows],
                      yerr=[r['mpc_rmse_std'] for r in rows], fmt='s-', label='MPC', capsize=3)
    axes[0].set_xlabel('cruise speed (m/s)')
    axes[0].set_ylabel('tracking RMSE (m), mean ± std over %d seeds' % n_seeds)
    axes[0].set_title('Tracking RMSE vs speed')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(speeds, [r['rmse_improvement_pct'] for r in rows], 'o-', color='tab:purple')
    axes[1].axhline(0, color='k', linewidth=0.8)
    axes[1].set_xlabel('cruise speed (m/s)')
    axes[1].set_ylabel('MPC RMSE improvement over PP (%)')
    axes[1].set_title('MPC vs Pure Pursuit RMSE improvement by speed')
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'figures', 'pure_pursuit_vs_mpc_by_speed.png'), dpi=150)
    plt.close(fig)
    return rows


def plot_trajectories(path_xy, headings, out_dir, best_lookahead, best_horizon, v_cruise=0.9):
    pp_params = PurePursuitParams(lookahead_distance=best_lookahead, cruise_linear_vel=v_cruise,
                                   max_linear_vel=1.4, max_angular_vel=2.2)
    pp = PurePursuitController(path_xy, pp_params)
    _, pp_track = simulate(pp, path_xy, headings, (path_xy[0][0], path_xy[0][1], headings[0]),
                            dt=0.1, process_noise_std=(0.01, 0.01))

    mpc_params = MPCParams(horizon=best_horizon, dt=0.1, cruise_linear_vel=v_cruise,
                            max_linear_vel=1.4, max_angular_vel=2.2)
    mpc = LinearMPCController(path_xy, mpc_params)
    _, mpc_track = simulate(mpc, path_xy, headings, (path_xy[0][0], path_xy[0][1], headings[0]),
                             dt=0.1, process_noise_std=(0.01, 0.01))

    fig, ax = plt.subplots(figsize=(6, 4))
    t_pp = np.arange(len(pp_track['lateral_errors'])) * 0.1
    t_mpc = np.arange(len(mpc_track['lateral_errors'])) * 0.1
    ax.plot(t_pp, pp_track['lateral_errors'], label='Pure Pursuit')
    ax.plot(t_mpc, mpc_track['lateral_errors'], label='MPC')
    ax.axhline(0, color='k', linewidth=0.8)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('signed lateral error (m)')
    ax.set_title(f'Lateral tracking error over time @ {v_cruise} m/s cruise')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'figures', 'lateral_error_timeseries.png'), dpi=150)
    plt.close(fig)


def _write_csv(path, rows):
    if not rows:
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default=os.path.join(
        os.path.dirname(__file__), '..', '..', '..', 'docs'))
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()
    np.random.seed(args.seed)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(os.path.join(out_dir, 'figures'), exist_ok=True)

    path_xy, headings = get_reference_path()
    total_len = sum(
        math.hypot(path_xy[i + 1][0] - path_xy[i][0], path_xy[i + 1][1] - path_xy[i][1])
        for i in range(len(path_xy) - 1))
    print(f"reference path: {len(path_xy)} points, length={total_len:.2f} m")

    pp_rows = sweep_pure_pursuit(path_xy, headings, out_dir)
    mpc_rows = sweep_mpc(path_xy, headings, out_dir)

    best_pp = min(pp_rows, key=lambda r: r['rmse'])
    best_mpc = min(mpc_rows, key=lambda r: r['rmse'])
    print(f"\nbest lookahead by RMSE: {best_pp['lookahead_distance']} "
          f"(RMSE={best_pp['rmse']:.4f})")
    print(f"best horizon by RMSE: {best_mpc['horizon']} (RMSE={best_mpc['rmse']:.4f}, "
          f"solve_mean={best_mpc['solve_time_ms_mean']:.3f}ms)")

    compare_at_speeds(path_xy, headings, out_dir,
                       best_pp['lookahead_distance'], best_mpc['horizon'])
    plot_trajectories(path_xy, headings, out_dir,
                       best_pp['lookahead_distance'], best_mpc['horizon'])

    print("\nDone. CSVs + figures written under:", out_dir)


if __name__ == '__main__':
    main()
