#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USV避障系统主程序
VRX项目中的全局路径规划+局部路径规划+VLM障碍物检测+PID轨迹跟踪

作者: 项目团队
日期: 2025-01-21
"""

import sys
import os
import argparse
import time
import numpy as np

# 添加src目录到Python路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.simulation.vrx_simulator import VRXSimulator
from src.experiments.dynamic_avoidance_test import DynamicAvoidanceExperiment, run_complete_experiment
from src.perception.vlm_detector import VLMObstacleDetector
from src.utils.config import Config

def test_vlm_connection():
    """测试VLM API连接"""
    print("正在测试VLM API连接...")
    detector = VLMObstacleDetector()
    
    if detector.test_connection():
        # 避免Windows控制台GBK编码问题，使用ASCII
        print("[OK] VLM API连接成功")
        return True
    else:
        print("[FAIL] VLM API连接失败")
        print("请检查API密钥和网络连接")
        return False

def run_basic_demo(enable_vlm=True, scenario: str = "basic"):
    """运行基础演示"""
    print("\n=== USV避障系统基础演示 ===")
    
    # 创建仿真器
    simulator = VRXSimulator(enable_vlm=enable_vlm)
    
    # 添加场景障碍物
    simulator.add_scenario_obstacles(scenario)
    
    # 设置目标点
    goal = (45.0, 45.0)
    simulator.set_goal(goal[0], goal[1])
    
    print(f"起始位置: (5.0, 5.0)")
    print(f"目标位置: {goal}")
    print(f"场景: {scenario}")
    
    # 运行仿真
    print("\n开始仿真...")
    max_time = 90.0 if scenario in ["dense", "complex"] else 60.0
    results = simulator.run_simulation(max_time=max_time, goal=goal)
    
    # 显示结果
    print(f"\n仿真结果:")
    print(f"- 是否到达目标: {'是' if results['goal_reached'] else '否'}")
    print(f"- 仿真时间: {results['total_time']:.1f}s")
    print(f"- 轨迹长度: {results['path_length']} 个点")
    
    if results['planning_statistics']:
        stats = results['planning_statistics']
        print(f"- 平均规划时间: {stats.get('average_time_ms', 0):.2f}ms")
        print(f"- 最大规划时间: {stats.get('max_time_ms', 0):.2f}ms")
    
    # 可视化结果
    simulator.visualize_simulation(save_path="demo_result.png")
    
    return results

def run_dynamic_demo():
    """运行动态避障演示"""
    print("\n=== USV动态避障演示 ===")
    
    # 创建仿真器
    simulator = VRXSimulator(enable_vlm=True)
    
    # 添加动态场景障碍物
    simulator.add_scenario_obstacles("dynamic")
    
    # 设置目标点
    goal = (45.0, 45.0)
    simulator.set_goal(goal[0], goal[1])
    
    print(f"起始位置: (5.0, 5.0)")
    print(f"目标位置: {goal}")
    print("场景: 动态移动障碍物")
    
    # 运行仿真
    print("\n开始仿真...")
    results = simulator.run_simulation(max_time=60.0, goal=goal)
    
    # 显示结果
    print(f"\n仿真结果:")
    print(f"- 是否到达目标: {'是' if results['goal_reached'] else '否'}")
    print(f"- 仿真时间: {results['total_time']:.1f}s")
    print(f"- 轨迹长度: {results['path_length']} 个点")
    
    if results['planning_statistics']:
        stats = results['planning_statistics']
        print(f"- 平均规划时间: {stats.get('average_time_ms', 0):.2f}ms")
        print(f"- 最大规划时间: {stats.get('max_time_ms', 0):.2f}ms")
    
    # 可视化结果
    simulator.visualize_simulation(save_path="dynamic_demo_result.png")
    
    return results

def run_vlm_comparison():
    """运行VLM效果对比"""
    print("\n=== VLM效果对比测试 ===")
    
    experiment = DynamicAvoidanceExperiment()
    results = experiment.run_vlm_comparison_test()
    
    return results

def run_performance_test():
    """运行性能测试"""
    print("\n=== 性能测试 ===")
    
    # 创建仿真器（关闭VLM以测试基础性能）
    simulator = VRXSimulator(enable_vlm=False)
    
    # 添加复杂场景
    simulator.add_scenario_obstacles("complex")
    
    # 多次运行测试
    num_tests = 10
    planning_times = []
    success_count = 0
    
    print(f"运行 {num_tests} 次测试...")
    
    for i in range(num_tests):
        print(f"测试 {i+1}/{num_tests}", end=" ")
        
        # 重置仿真器
        simulator.reset()
        simulator.add_scenario_obstacles("complex")
        
        # 随机目标点
        goal_x = np.random.uniform(35, 45)
        goal_y = np.random.uniform(35, 45)
        
        # 运行仿真
        results = simulator.run_simulation(max_time=30.0, goal=(goal_x, goal_y))
        
        if results['goal_reached']:
            success_count += 1
            print("✓")
        else:
            print("✗")
        
        # 记录性能
        if results['planning_statistics']:
            avg_time = results['planning_statistics'].get('average_time_ms', 0)
            planning_times.append(avg_time)
    
    # 分析结果
    if planning_times:
        print(f"\n性能统计:")
        print(f"- 成功率: {success_count}/{num_tests} ({success_count/num_tests*100:.1f}%)")
        print(f"- 平均规划时间: {np.mean(planning_times):.2f}ms")
        print(f"- 规划时间标准差: {np.std(planning_times):.2f}ms")
        print(f"- 最大规划时间: {np.max(planning_times):.2f}ms")
        print(f"- 最小规划时间: {np.min(planning_times):.2f}ms")

def print_system_info():
    """打印系统信息"""
    print("=" * 60)
    print("USV避障系统 - VRX项目")
    print("=" * 60)
    print(f"版本: 1.0.0")
    print(f"功能: A*全局规划 + DWA局部规划 + VLM障碍物检测 + PID控制")
    print(f"VLM模型: {Config.VLM_MODEL}")
    print(f"最大速度: {Config.DWA_MAX_SPEED} m/s")
    print(f"机器人半径: {Config.DWA_ROBOT_RADIUS} m")
    print("=" * 60)

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='USV避障系统演示程序')
    parser.add_argument('--mode', choices=['demo', 'dynamic', 'experiment', 'performance', 'test-vlm'], 
                       default='demo', help='运行模式')
    parser.add_argument('--no-vlm', action='store_true', help='禁用VLM检测')
    parser.add_argument('--scenario', choices=['basic','dynamic','complex','dense'], default='basic', help='演示场景')
    parser.add_argument('--save-results', action='store_true', help='保存结果')
    
    args = parser.parse_args()
    
    # 打印系统信息
    print_system_info()
    
    # 测试VLM连接（如果启用）
    if not args.no_vlm and args.mode != 'performance':
        if not test_vlm_connection():
            print("\n警告: VLM连接失败，将使用模拟数据继续演示")
            time.sleep(2)
    
    try:
        if args.mode == 'demo':
            print("\n运行基础演示...")
            run_basic_demo(enable_vlm=not args.no_vlm, scenario=args.scenario)
            
        elif args.mode == 'dynamic':
            print("\n运行动态避障演示...")
            run_dynamic_demo()
            
        elif args.mode == 'experiment':
            print("\n运行完整实验...")
            run_complete_experiment()
            
        elif args.mode == 'performance':
            print("\n运行性能测试...")
            run_performance_test()
            
        elif args.mode == 'test-vlm':
            print("\n测试VLM功能...")
            if test_vlm_connection():
                run_vlm_comparison()
            else:
                print("VLM连接失败，无法进行测试")
        
        print("\n程序执行完成!")
        
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 