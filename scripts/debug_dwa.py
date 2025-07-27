#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DWA调试脚本 - 分析路径跟踪问题
"""

import numpy as np
import matplotlib.pyplot as plt
from src.planning.global_planner import AStarPlanner
from src.planning.local_planner import DWALocalPlanner, DynamicState
from src.simulation.vrx_simulator import Environment
from src.utils.config import Config

def debug_dwa_behavior():
    """调试DWA行为"""
    
    # 创建环境和规划器
    environment = Environment(50, 50)
    global_planner = AStarPlanner()
    local_planner = DWALocalPlanner(enable_vlm=False)
    
    # 添加障碍物
    environment.add_static_obstacle(15, 15, 2)
    environment.add_static_obstacle(25, 30, 1.5)
    environment.add_static_obstacle(35, 20, 2.5)
    
    # 规划全局路径
    start = (5.0, 5.0)
    goal = (45.0, 45.0)
    global_path = global_planner.plan(start, goal, environment.obstacle_map)
    
    if global_path:
        global_path = global_planner.smooth_path(global_path, environment.obstacle_map)
        print(f"全局路径点数: {len(global_path)}")
        print(f"前5个路径点: {global_path[:5]}")
    else:
        print("全局路径规划失败")
        return
    
    # 初始状态
    current_state = DynamicState(5.0, 5.0, 0.0, 0.0, 0.0)
    path_index = 0
    
    # 仿真步骤
    plt.figure(figsize=(12, 10))
    
    for step in range(10):  # 仅仿真10步进行调试
        print(f"\n=== 步骤 {step} ===")
        print(f"当前位置: ({current_state.x:.2f}, {current_state.y:.2f})")
        print(f"当前路径索引: {path_index}")
        
        # 获取障碍物
        obstacles = environment.get_all_obstacles()
        
        # 局部规划
        v, omega, new_path_index = local_planner.plan_with_global_path(
            current_state, global_path, path_index, obstacles)
        
        print(f"规划结果: v={v:.3f}, omega={omega:.3f}")
        print(f"新路径索引: {new_path_index}")
        
        # 找到当前目标点
        if new_path_index < len(global_path):
            target_point = local_planner._find_lookahead_point(
                current_state, global_path, new_path_index, 5.0)
            print(f"前瞻目标点: ({target_point[0]:.2f}, {target_point[1]:.2f})")
            
            # 计算到目标的距离和角度
            dx = target_point[0] - current_state.x
            dy = target_point[1] - current_state.y
            dist = np.sqrt(dx**2 + dy**2)
            angle = np.arctan2(dy, dx)
            print(f"到目标距离: {dist:.2f}m, 角度: {np.degrees(angle):.1f}°")
        
        # 更新状态
        dt = 0.1
        current_state.x += v * np.cos(current_state.yaw) * dt
        current_state.y += v * np.sin(current_state.yaw) * dt
        current_state.yaw += omega * dt
        current_state.v = v
        current_state.omega = omega
        
        path_index = new_path_index
        
        # 绘制当前状态
        plt.subplot(2, 5, step + 1)
        
        # 绘制全局路径
        path_x = [p[0] for p in global_path]
        path_y = [p[1] for p in global_path]
        plt.plot(path_x, path_y, 'b--', alpha=0.7, label='Global Path')
        
        # 绘制障碍物
        for ox, oy, radius in obstacles:
            circle = plt.Circle((ox, oy), radius, color='red', alpha=0.5)
            plt.gca().add_patch(circle)
        
        # 绘制USV
        plt.plot(current_state.x, current_state.y, 'go', markersize=8)
        
        # 绘制朝向
        dx = 2 * np.cos(current_state.yaw)
        dy = 2 * np.sin(current_state.yaw)
        plt.arrow(current_state.x, current_state.y, dx, dy, 
                 head_width=0.5, head_length=0.5, fc='green', ec='green')
        
        # 绘制目标点
        if new_path_index < len(global_path):
            target_point = local_planner._find_lookahead_point(
                current_state, global_path, new_path_index, 5.0)
            plt.plot(target_point[0], target_point[1], 'ro', markersize=6)
        
        plt.xlim(0, 50)
        plt.ylim(0, 50)
        plt.grid(True, alpha=0.3)
        plt.title(f'Step {step}\nv={v:.2f}, ω={omega:.2f}')
        
        if step == 0:
            plt.legend()
    
    plt.tight_layout()
    plt.savefig('dwa_debug_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print(f"\n调试图像已保存为 'dwa_debug_analysis.png'")

if __name__ == "__main__":
    debug_dwa_behavior() 