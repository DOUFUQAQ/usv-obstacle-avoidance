#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
详细的DWA调试脚本 - 分析成本计算过程
"""

import numpy as np
from src.planning.local_planner import DWALocalPlanner, DynamicState
from src.utils.config import Config

def debug_dwa_cost_calculation():
    """详细调试DWA成本计算"""
    
    # 创建局部规划器
    local_planner = DWALocalPlanner(enable_vlm=False)
    
    # 设置简单的状态和目标
    state = DynamicState(5.0, 5.0, 0.0, 0.0, 0.0)
    goal = (13.5, 18.5)
    obstacles = []  # 先不考虑障碍物
    
    print(f"当前状态: x={state.x}, y={state.y}, yaw={state.yaw}, v={state.v}")
    print(f"目标位置: {goal}")
    print(f"DWA参数:")
    print(f"  max_speed: {local_planner.max_speed}")
    print(f"  velocity_resolution: {local_planner.velocity_resolution}")
    print(f"  angular_resolution: {local_planner.angular_resolution}")
    print(f"  to_goal_cost_gain: {local_planner.to_goal_cost_gain}")
    print(f"  speed_cost_gain: {local_planner.speed_cost_gain}")
    print(f"  obstacle_cost_gain: {local_planner.obstacle_cost_gain}")
    
    # 计算动态窗口
    dw = local_planner.calc_dynamic_window(state)
    print(f"\n动态窗口: v_min={dw[0]}, v_max={dw[1]}, omega_min={dw[2]}, omega_max={dw[3]}")
    
    # 测试在动态窗口内的速度组合
    test_velocities = []
    v = dw[0]
    while v <= dw[1]:
        omega = dw[2]
        while omega <= dw[3]:
            test_velocities.append((v, omega))
            omega += local_planner.angular_resolution
        v += local_planner.velocity_resolution
    
    print(f"\n详细成本分析:")
    print(f"{'v':>5} {'ω':>5} {'goal_cost':>10} {'speed_cost':>11} {'obs_cost':>9} {'total_cost':>11}")
    print("-" * 70)
    
    min_cost = float("inf")
    best_v = 0.0
    best_omega = 0.0
    
    for v, omega in test_velocities:
        # 检查是否在动态窗口内
        if not (dw[0] <= v <= dw[1] and dw[2] <= omega <= dw[3]):
            continue
            
        # 预测轨迹
        trajectory = local_planner.predict_trajectory(state, v, omega)
        
        # 计算各项成本
        goal_cost = local_planner.calc_to_goal_cost(trajectory, goal)
        speed_cost = local_planner.calc_speed_cost(trajectory)
        obs_cost = local_planner.calc_obstacle_cost(trajectory, obstacles)
        
        # 总成本
        total_cost = (local_planner.to_goal_cost_gain * goal_cost +
                     local_planner.speed_cost_gain * speed_cost +
                     local_planner.obstacle_cost_gain * obs_cost)
        
        print(f"{v:5.1f} {omega:5.1f} {goal_cost:10.3f} {speed_cost:11.3f} {obs_cost:9.3f} {total_cost:11.3f}")
        
        if total_cost < min_cost:
            min_cost = total_cost
            best_v = v
            best_omega = omega
    
    print("-" * 70)
    print(f"最优选择: v={best_v:.3f}, ω={best_omega:.3f}, 成本={min_cost:.3f}")
    
    # 详细分析最优解
    if min_cost < float("inf"):
        print(f"\n最优解轨迹分析:")
        best_trajectory = local_planner.predict_trajectory(state, best_v, best_omega)
        if best_trajectory:
            end_state = best_trajectory[-1]
            print(f"  轨迹终点: x={end_state.x:.2f}, y={end_state.y:.2f}, yaw={end_state.yaw:.2f}")
            
            dx = goal[0] - end_state.x
            dy = goal[1] - end_state.y
            distance_to_goal = np.sqrt(dx**2 + dy**2)
            print(f"  到目标距离: {distance_to_goal:.2f}m")

if __name__ == "__main__":
    debug_dwa_cost_calculation() 