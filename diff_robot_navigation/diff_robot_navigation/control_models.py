#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
control_models.py
==================
差速驱动机器人的运动学模型 + 两种轨迹跟踪控制器的核心算法实现，全部是纯
numpy/scipy 代码，不依赖 rclpy，可以被三个地方复用（同一份算法，三处调用，
避免"ROS 节点里一份逻辑、离线调参脚本里再抄一份"导致两边行为不一致）：
  1. diff_robot_navigation/pure_pursuit_controller.py  （ROS2 节点）
  2. diff_robot_navigation/mpc_controller.py            （ROS2 节点）
  3. scripts/tune_and_compare.py                        （离线整定 + 出图，需求3）

对应需求 3："实现 Pure Pursuit 轨迹跟踪控制器，并对比 MPC 方案，对前视距离、
最大线速度/角速度等关键参数进行调优，将跟踪误差控制在合理范围，行驶平稳无振荡"。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# 运动学"真实"模型：用于闭环仿真（既是离线调参脚本的仿真器，
# 也是两台控制器共用的"世界模型"假设：差速驱动机器人可以近似成 unicycle）
# ---------------------------------------------------------------------------

def unicycle_step(state: np.ndarray, v: float, omega: float, dt: float) -> np.ndarray:
    """state = [x, y, theta]，前向欧拉积分一步。"""
    x, y, theta = state
    return np.array([
        x + v * math.cos(theta) * dt,
        y + v * math.sin(theta) * dt,
        wrap_to_pi(theta + omega * dt),
    ])


def resample_path(path: Sequence[Point], spacing: float) -> List[Point]:
    """把折线/样条路径按固定弧长间隔重新采样，供 lookahead 搜索和 MPC 参考轨迹使用。"""
    if len(path) < 2:
        return list(path)
    out = [path[0]]
    acc = 0.0
    for i in range(len(path) - 1):
        p0 = np.array(path[i])
        p1 = np.array(path[i + 1])
        seg_len = float(np.linalg.norm(p1 - p0))
        if seg_len < 1e-9:
            continue
        d = acc
        while d + spacing <= seg_len:
            d += spacing
            t = d / seg_len
            out.append(tuple(p0 + t * (p1 - p0)))
        acc = d - seg_len
    if np.linalg.norm(np.array(out[-1]) - np.array(path[-1])) > 1e-6:
        out.append(path[-1])
    return out


def build_arclength_table(path: Sequence[Point]) -> np.ndarray:
    """累计弧长表，cum[0]=0，cum[i] = 从 path[0] 走到 path[i] 的折线长度。"""
    cum = np.zeros(len(path))
    for i in range(1, len(path)):
        cum[i] = cum[i - 1] + math.hypot(path[i][0] - path[i - 1][0],
                                          path[i][1] - path[i - 1][1])
    return cum


def point_at_arclength(path: Sequence[Point], cum: np.ndarray, s: float) -> Point:
    """按弧长 s 在折线上做线性插值取点，s 会被夹到 [0, cum[-1]] 范围内。"""
    s = float(np.clip(s, 0.0, cum[-1]))
    i = int(np.searchsorted(cum, s, side='right') - 1)
    i = min(max(i, 0), len(path) - 2) if len(path) > 1 else 0
    seg_len = cum[i + 1] - cum[i]
    t = 0.0 if seg_len < 1e-9 else (s - cum[i]) / seg_len
    x = path[i][0] + t * (path[i + 1][0] - path[i][0])
    y = path[i][1] + t * (path[i + 1][1] - path[i][1])
    return x, y


def heading_at_arclength(path: Sequence[Point], cum: np.ndarray, s: float,
                          window: float = 0.15) -> float:
    """取 s 附近 ±window/2 弧长窗口两端点连线方向作为该处切线方向，
    比相邻两个采样点直接求方向更抗局部抖动（对应"跟踪平稳不振荡"的诉求）。
    """
    half = window / 2.0
    p_back = point_at_arclength(path, cum, s - half)
    p_fwd = point_at_arclength(path, cum, s + half)
    if math.hypot(p_fwd[0] - p_back[0], p_fwd[1] - p_back[1]) < 1e-6:
        return 0.0
    return math.atan2(p_fwd[1] - p_back[1], p_fwd[0] - p_back[0])


def path_headings(path: Sequence[Point]) -> np.ndarray:
    n = len(path)
    headings = np.zeros(n)
    for i in range(n):
        if i < n - 1:
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
        else:
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
        headings[i] = math.atan2(dy, dx)
    return headings


# ---------------------------------------------------------------------------
# Pure Pursuit
# ---------------------------------------------------------------------------

@dataclass
class PurePursuitParams:
    lookahead_distance: float = 0.6      # 前视距离 (m) —— 核心调参对象
    min_lookahead: float = 0.3
    max_lookahead: float = 1.5
    cruise_linear_vel: float = 0.5       # 巡航线速度 (m/s)
    max_linear_vel: float = 1.0
    max_angular_vel: float = 1.8
    goal_tolerance: float = 0.10
    # 曲率越大越减速：curvature_slowdown_gain 越大，转弯降速越明显，越不容易转弯振荡
    curvature_slowdown_gain: float = 1.4
    decel_radius: float = 0.6            # 距终点这个半径内开始线性减速


class PurePursuitController:
    """标准几何 Pure Pursuit：在参考路径上搜索前视点，按圆弧曲率转成角速度指令。"""

    def __init__(self, path: Sequence[Point], params: PurePursuitParams | None = None):
        self.params = params or PurePursuitParams()
        self._path = resample_path(path, spacing=0.05)
        self._closest_idx = 0

    def reset(self):
        self._closest_idx = 0

    def is_goal_reached(self, state: np.ndarray) -> bool:
        gx, gy = self._path[-1]
        return math.hypot(state[0] - gx, state[1] - gy) < self.params.goal_tolerance

    def _find_lookahead_point(self, state: np.ndarray, lookahead: float) -> Point:
        x, y, _ = state
        n = len(self._path)
        # 先在局部窗口内找最近点，避免路径自交/回头时前视点跳变
        search_lo = max(0, self._closest_idx - 5)
        best_i, best_d = self._closest_idx, math.inf
        for i in range(search_lo, n):
            d = math.hypot(self._path[i][0] - x, self._path[i][1] - y)
            if d < best_d:
                best_d, best_i = d, i
        self._closest_idx = best_i

        for i in range(best_i, n):
            d = math.hypot(self._path[i][0] - x, self._path[i][1] - y)
            if d >= lookahead:
                return self._path[i]
        return self._path[-1]

    def compute_control(self, state: np.ndarray) -> Tuple[float, float]:
        p = self.params
        x, y, theta = state

        dist_to_goal = math.hypot(self._path[-1][0] - x, self._path[-1][1] - y)
        lookahead = float(np.clip(p.lookahead_distance, p.min_lookahead, p.max_lookahead))
        target = self._find_lookahead_point(state, lookahead)

        dx = target[0] - x
        dy = target[1] - y
        # 转到机器人坐标系
        local_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        local_y = math.sin(-theta) * dx + math.cos(-theta) * dy
        local_x = max(local_x, 1e-3)  # 防止目标点几乎在正后方时曲率发散

        curvature = 2.0 * local_y / (lookahead ** 2)

        v = p.cruise_linear_vel / (1.0 + p.curvature_slowdown_gain * abs(curvature))
        if dist_to_goal < p.decel_radius:
            v *= max(dist_to_goal / p.decel_radius, 0.15)
        v = float(np.clip(v, 0.0, p.max_linear_vel))

        omega = float(np.clip(v * curvature, -p.max_angular_vel, p.max_angular_vel))
        return v, omega


# ---------------------------------------------------------------------------
# Linear (LTV) MPC —— 批量凝聚 QP，用 scipy L-BFGS-B 求解（只有输入上下界约束）
# ---------------------------------------------------------------------------

@dataclass
class MPCParams:
    horizon: int = 12                    # 预测步数 N —— 核心调参对象
    dt: float = 0.1
    cruise_linear_vel: float = 0.5
    max_linear_vel: float = 1.0
    max_angular_vel: float = 1.8
    goal_tolerance: float = 0.10
    decel_radius: float = 0.6
    q_xy: float = 8.0                    # 位置误差权重
    q_theta: float = 2.0                 # 航向误差权重
    r_v: float = 0.8                     # 线速度控制量权重（越大越平滑但响应越慢）
    r_omega: float = 0.6                 # 角速度控制量权重
    r_dv: float = 1.2                    # 控制量变化率权重（抑制振荡的关键项）
    r_domega: float = 1.4
    max_iters: int = 60


class LinearMPCController:
    """基于运动学 unicycle 模型在参考轨迹附近线性化的时变线性 MPC (LTV-MPC)。

    误差状态 e = [x-x_r, y-y_r, theta-theta_r]，控制偏差 du = [v,omega] - [v_r,omega_r]。
    线性化：
        A_k = [[1, 0, -v_r*sin(theta_r)*dt],
               [0, 1,  v_r*cos(theta_r)*dt],
               [0, 0,  1]]
        B_k = [[cos(theta_r)*dt, 0],
               [sin(theta_r)*dt, 0],
               [0,               dt]]
    在 horizon 内把 e_1..e_N 表示成 e_0 和 du_0..du_{N-1} 的线性函数（"凝聚"/condensing），
    转成一个只有上下界约束的二次规划，用 L-BFGS-B + 解析梯度求解，
    不需要额外装 osqp/cvxpy 等重量级 QP 库，ROS2 桌面环境自带的 scipy 就够用。
    """

    def __init__(self, path: Sequence[Point], params: MPCParams | None = None):
        self.params = params or MPCParams()
        # 参考路径本身只需要比控制周期内的位移精细即可，不需要跟 v_ref 强绑定
        # spacing（这正是旧实现的 bug 根源：固定按巡航速度重采样后，减速段的
        # "第 k 个采样点"和"按 v_ref*dt 累积的第 k 步参考位置"会对不上，
        # 越靠近终点误差越大，导致 MPC 在减速段追着一个错位的参考点跑，
        # 表现为绕圈/角速度抖动、迟迟到不了终点）。这里改成弧长参数化 +
        # 按当前（含减速）速度剖面沿弧长现取参考点，彻底避免这个错位。
        self._path = resample_path(path, spacing=0.03)
        self._cum = build_arclength_table(self._path)
        self._closest_idx = 0
        self._prev_du = None  # 上一周期的解，用来热启动 + 计算 du 的变化率代价

    def reset(self):
        self._closest_idx = 0
        self._prev_du = None

    def is_goal_reached(self, state: np.ndarray) -> bool:
        gx, gy = self._path[-1]
        return math.hypot(state[0] - gx, state[1] - gy) < self.params.goal_tolerance

    def _v_ref_at_s(self, s: float) -> float:
        p = self.params
        dist_to_goal = self._cum[-1] - s
        v_ref = p.cruise_linear_vel
        if dist_to_goal < p.decel_radius:
            v_ref *= max(dist_to_goal / p.decel_radius, 0.15)
        return max(v_ref, 0.0)

    def _reference_trajectory(self, state: np.ndarray):
        p = self.params
        x, y, _ = state
        n = len(self._path)
        lo = max(0, self._closest_idx - 5)
        best_i, best_d = self._closest_idx, math.inf
        for i in range(lo, n):
            d = math.hypot(self._path[i][0] - x, self._path[i][1] - y)
            if d < best_d:
                best_d, best_i = d, i
        self._closest_idx = best_i
        s0 = self._cum[best_i]

        x_r = np.zeros(p.horizon + 1)
        y_r = np.zeros(p.horizon + 1)
        th_r = np.zeros(p.horizon + 1)
        v_r = np.zeros(p.horizon)

        s = s0
        px, py = point_at_arclength(self._path, self._cum, s)
        x_r[0], y_r[0] = px, py
        th_r[0] = heading_at_arclength(self._path, self._cum, s)
        for k in range(p.horizon):
            v_k = self._v_ref_at_s(s)
            v_r[k] = v_k
            s = min(s + v_k * p.dt, self._cum[-1])
            px, py = point_at_arclength(self._path, self._cum, s)
            x_r[k + 1], y_r[k + 1] = px, py
            th_r[k + 1] = heading_at_arclength(self._path, self._cum, s)

        omega_r = np.zeros(p.horizon)
        for k in range(p.horizon):
            omega_r[k] = wrap_to_pi(th_r[k + 1] - th_r[k]) / p.dt
        return x_r, y_r, th_r, v_r, omega_r

    def compute_control(self, state: np.ndarray) -> Tuple[float, float]:
        p = self.params
        N = p.horizon
        x_r, y_r, th_r, v_r, omega_r = self._reference_trajectory(state)

        e0 = np.array([
            state[0] - x_r[0],
            state[1] - y_r[0],
            wrap_to_pi(state[2] - th_r[0]),
        ])

        # ---- 构造时变 A_k,B_k 并做条件凝聚：E = Sx*e0 + Su*dU ----
        nx, nu = 3, 2
        A_list, B_list = [], []
        for k in range(N):
            th = th_r[k]
            v = v_r[k]
            A = np.array([
                [1.0, 0.0, -v * math.sin(th) * p.dt],
                [0.0, 1.0, v * math.cos(th) * p.dt],
                [0.0, 0.0, 1.0],
            ])
            B = np.array([
                [math.cos(th) * p.dt, 0.0],
                [math.sin(th) * p.dt, 0.0],
                [0.0, p.dt],
            ])
            A_list.append(A)
            B_list.append(B)

        Sx = np.zeros((N * nx, nx))
        Su = np.zeros((N * nx, N * nu))
        A_prod = np.eye(nx)
        for k in range(N):
            A_prod = A_list[k] @ A_prod
            Sx[k * nx:(k + 1) * nx, :] = A_prod
            for j in range(k + 1):
                # e_{k+1} 里 du_j 的系数 = A_k A_{k-1}...A_{j+1} * B_j
                M = B_list[j]
                for m in range(j + 1, k + 1):
                    M = A_list[m] @ M
                Su[k * nx:(k + 1) * nx, j * nu:(j + 1) * nu] = M

        Q = np.diag([p.q_xy, p.q_xy, p.q_theta])
        Qbar = np.kron(np.eye(N), Q)
        Rbar = np.kron(np.eye(N), np.diag([p.r_v, p.r_omega]))

        # du 变化率惩罚：D 是一阶差分矩阵， cost += dU^T D^T Rd D dU （Rd 常数对角）
        D = np.eye(N * nu) - np.eye(N * nu, k=-nu)
        Rd = np.kron(np.eye(N), np.diag([p.r_dv, p.r_domega]))

        H = 2.0 * (Su.T @ Qbar @ Su + Rbar + D.T @ Rd @ D)
        f = 2.0 * (Su.T @ Qbar @ (Sx @ e0))

        # du_0 相对"上一控制周期实际下发的 du"也要纳入变化率代价，
        # 否则相邻两次求解之间的指令仍可能跳变（这是抑制振荡的关键项之一）。
        prev0 = np.zeros(N * nu)
        if self._prev_du is not None:
            prev0[0:nu] = self._prev_du
        f += -2.0 * (D.T @ Rd @ D @ prev0)

        lb = np.zeros(N * nu)
        ub = np.zeros(N * nu)
        for k in range(N):
            lb[k * nu + 0] = 0.0 - v_r[k]
            ub[k * nu + 0] = p.max_linear_vel - v_r[k]
            lb[k * nu + 1] = -p.max_angular_vel - omega_r[k]
            ub[k * nu + 1] = p.max_angular_vel - omega_r[k]

        def cost_and_grad(dU):
            g = H @ dU + f
            c = 0.5 * dU @ H @ dU + f @ dU
            return c, g

        from scipy.optimize import minimize

        x0 = np.zeros(N * nu)
        if self._prev_du is not None:
            x0[0:nu] = self._prev_du

        res = minimize(
            cost_and_grad, x0, jac=True, method='L-BFGS-B',
            bounds=list(zip(lb, ub)),
            options={'maxiter': p.max_iters, 'ftol': 1e-10, 'gtol': 1e-8},
        )
        dU = res.x
        self._prev_du = dU[0:nu]

        v = float(np.clip(v_r[0] + dU[0], 0.0, p.max_linear_vel))
        omega = float(np.clip(omega_r[0] + dU[1], -p.max_angular_vel, p.max_angular_vel))
        return v, omega


# ---------------------------------------------------------------------------
# 通用评价指标：两种控制器共用同一套评分标准，保证对比公平
# ---------------------------------------------------------------------------

@dataclass
class TrackingMetrics:
    rmse: float
    max_error: float
    mean_abs_omega_jerk: float   # 角速度前后差分的均值绝对值，越小越"平稳无振荡"
    time_to_goal: float
    solve_time_ms_mean: float
    solve_time_ms_max: float
    collided: bool = False


def evaluate_tracking(errors: Sequence[float], omegas: Sequence[float],
                       dt: float, time_to_goal: float,
                       solve_times_ms: Sequence[float], collided: bool) -> TrackingMetrics:
    err = np.asarray(errors)
    om = np.asarray(omegas)
    jerk = np.abs(np.diff(om)) / dt if len(om) > 1 else np.array([0.0])
    st = np.asarray(solve_times_ms) if len(solve_times_ms) else np.array([0.0])
    return TrackingMetrics(
        rmse=float(np.sqrt(np.mean(err ** 2))) if len(err) else float('nan'),
        max_error=float(np.max(err)) if len(err) else float('nan'),
        mean_abs_omega_jerk=float(np.mean(jerk)),
        time_to_goal=time_to_goal,
        solve_time_ms_mean=float(np.mean(st)),
        solve_time_ms_max=float(np.max(st)),
        collided=collided,
    )
