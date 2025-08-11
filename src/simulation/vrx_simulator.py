import numpy as np
import math
import time
from typing import List, Tuple, Optional, Dict
import matplotlib.pyplot as plt
import cv2
from ..utils.config import Config
from ..planning.global_planner import AStarPlanner
from ..planning.local_planner import DWALocalPlanner, DynamicState
from ..control.pid_controller import TrajectoryTracker
from ..perception.vlm_detector import VLMObstacleDetector
from .realistic_image_generator import RealisticMarineImageGenerator

class USVModel:
    """USV运动学模型"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.state = DynamicState(x, y, yaw, 0.0, 0.0)
        
        # USV物理参数
        self.length = 3.0  # 船体长度(米)
        self.width = 1.5   # 船体宽度(米)
        self.mass = 100.0  # 质量(kg)
        self.inertia = 50.0  # 转动惯量
        
        # 运动限制
        self.max_speed = Config.DWA_MAX_SPEED
        self.max_angular_speed = Config.DWA_MAX_ANGULAR_SPEED
        
        # 噪声参数
        self.position_noise = 0.05
        self.velocity_noise = 0.02
        
    def update(self, linear_velocity: float, angular_velocity: float, dt: float):
        """更新USV状态"""
        # 添加执行器噪声
        v_actual = linear_velocity + np.random.normal(0, self.velocity_noise)
        omega_actual = angular_velocity + np.random.normal(0, self.velocity_noise)
        
        # 限制速度
        v_actual = np.clip(v_actual, -self.max_speed, self.max_speed)
        omega_actual = np.clip(omega_actual, -self.max_angular_speed, self.max_angular_speed)
        
        # 运动学模型
        dx = v_actual * math.cos(self.state.yaw) * dt
        dy = v_actual * math.sin(self.state.yaw) * dt
        dyaw = omega_actual * dt
        
        # 更新状态
        self.state.x += dx + np.random.normal(0, self.position_noise)
        self.state.y += dy + np.random.normal(0, self.position_noise)
        self.state.yaw += dyaw
        self.state.v = v_actual
        self.state.omega = omega_actual
        
        # 角度归一化
        self.state.yaw = self._normalize_angle(self.state.yaw)
    
    def _normalize_angle(self, angle: float) -> float:
        """角度归一化"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_pose(self) -> Tuple[float, float, float]:
        """获取位姿"""
        return self.state.x, self.state.y, self.state.yaw
    
    def get_velocity(self) -> Tuple[float, float]:
        """获取速度"""
        return self.state.v, self.state.omega

class Environment:
    """仿真环境"""
    
    def __init__(self, width: float = 50.0, height: float = 50.0):
        self.width = width
        self.height = height
        
        # 静态障碍物
        self.static_obstacles = []
        
        # 动态障碍物
        self.dynamic_obstacles = []
        
        # 地图
        self.obstacle_map = np.zeros((int(height), int(width)), dtype=np.uint8)
        
        # 创建真实图像生成器
        self.image_generator = RealisticMarineImageGenerator(image_size=(1024, 768))
        
        # 环境参数
        self.current_weather = 'clear'
        self.current_time = 'day'
        
        # 边界作为障碍物
        self._add_boundaries()
    
    def _add_boundaries(self):
        """添加环境边界"""
        # 上下边界
        self.obstacle_map[0, :] = 1
        self.obstacle_map[-1, :] = 1
        # 左右边界
        self.obstacle_map[:, 0] = 1
        self.obstacle_map[:, -1] = 1
    
    def add_static_obstacle(self, x: float, y: float, radius: float):
        """添加静态障碍物"""
        self.static_obstacles.append((x, y, radius))
        self._update_obstacle_map()
    
    def add_dynamic_obstacle(self, x: float, y: float, radius: float, 
                           vx: float = 0.0, vy: float = 0.0):
        """添加动态障碍物"""
        obstacle = {
            'x': x, 'y': y, 'radius': radius,
            'vx': vx, 'vy': vy
        }
        self.dynamic_obstacles.append(obstacle)
    
    def update_dynamic_obstacles(self, dt: float):
        """更新动态障碍物位置"""
        for obs in self.dynamic_obstacles:
            obs['x'] += obs['vx'] * dt
            obs['y'] += obs['vy'] * dt
            
            # 边界反弹
            if obs['x'] <= obs['radius'] or obs['x'] >= self.width - obs['radius']:
                obs['vx'] = -obs['vx']
            if obs['y'] <= obs['radius'] or obs['y'] >= self.height - obs['radius']:
                obs['vy'] = -obs['vy']
        
        self._update_obstacle_map()
    
    def _update_obstacle_map(self):
        """更新障碍物地图"""
        self.obstacle_map = np.zeros((int(self.height), int(self.width)), dtype=np.uint8)
        self._add_boundaries()
        
        # 添加静态障碍物
        for x, y, radius in self.static_obstacles:
            self._add_circle_to_map(x, y, radius)
        
        # 添加动态障碍物
        for obs in self.dynamic_obstacles:
            self._add_circle_to_map(obs['x'], obs['y'], obs['radius'])
    
    def _add_circle_to_map(self, x: float, y: float, radius: float):
        """在地图上添加圆形障碍物(包含安全余量与机器人半径膨胀)"""
        center_x, center_y = int(x), int(y)
        # 将障碍半径进行额外膨胀，考虑机器人半径与安全余量
        inflate = int(math.ceil(radius + Config.DWA_ROBOT_RADIUS + Config.SAFETY_MARGIN))
        r = max(1, inflate)
        
        for i in range(max(0, center_y - r), min(self.height, center_y + r + 1)):
            for j in range(max(0, center_x - r), min(self.width, center_x + r + 1)):
                if (j - center_x)**2 + (i - center_y)**2 <= r**2:
                    self.obstacle_map[i, j] = 1
    
    def get_all_obstacles(self) -> List[Tuple[float, float, float]]:
        """获取所有障碍物"""
        all_obstacles = self.static_obstacles.copy()
        for obs in self.dynamic_obstacles:
            all_obstacles.append((obs['x'], obs['y'], obs['radius']))
        return all_obstacles
    
    def generate_camera_image(self, usv_pose: Tuple[float, float, float], 
                            image_size: Tuple[int, int] = (1024, 768)) -> np.ndarray:
        """生成真实的USV摄像头图像"""
        # 准备障碍物数据
        obstacles_for_image = []
        
        # 添加静态障碍物
        for obs in self.static_obstacles:
            obstacles_for_image.append({
                'x': obs[0], 'y': obs[1], 'radius': obs[2], 'type': 'static'
            })
        
        # 添加动态障碍物
        for obs in self.dynamic_obstacles:
            obstacles_for_image.append({
                'x': obs['x'], 'y': obs['y'], 'radius': obs['radius'], 'type': 'dynamic'
            })
        
        # 使用真实图像生成器
        realistic_image = self.image_generator.generate_realistic_marine_image(
            usv_pose, obstacles_for_image, self.current_weather, self.current_time
        )
        
        # 如果需要特定尺寸，进行缩放
        if image_size != (1024, 768):
            realistic_image = cv2.resize(realistic_image, image_size)
        
        return realistic_image
    
    def set_weather(self, weather: str):
        """设置天气条件"""
        valid_weather = ['clear', 'cloudy', 'foggy', 'rainy']
        if weather in valid_weather:
            self.current_weather = weather
    
    def set_time_of_day(self, time_of_day: str):
        """设置时间"""
        valid_times = ['day', 'dawn', 'dusk', 'night']
        if time_of_day in valid_times:
            self.current_time = time_of_day

class VRXSimulator:
    """VRX仿真器主类"""
    
    def __init__(self, enable_vlm: bool = True):
        # 仿真参数
        self.dt = Config.SIM_STEP_TIME
        self.current_time = 0.0
        self.enable_vlm = enable_vlm
        
        # 创建环境和USV
        self.environment = Environment(Config.SIM_MAP_WIDTH, Config.SIM_MAP_HEIGHT)
        self.usv = USVModel(5.0, 5.0, 0.0)  # 起始位置
        
        # 创建规划和控制模块
        self.global_planner = AStarPlanner()
        self.local_planner = DWALocalPlanner(enable_vlm=enable_vlm)
        self.trajectory_tracker = TrajectoryTracker()
        
        # 仿真状态
        self.is_running = False
        self.goal = None
        self.global_path = None
        self.path_index = 0
        
        # 数据记录
        self.trajectory_history = []
        self.planning_time_history = []
        self.control_history = []
        
    def set_goal(self, x: float, y: float):
        """设置目标点"""
        self.goal = (x, y)
        
        # 重新规划全局路径
        start_pose = self.usv.get_pose()
        self.global_path = self.global_planner.plan(
            (start_pose[0], start_pose[1]), self.goal, self.environment.obstacle_map)
        
        if self.global_path:
            self.global_path = self.global_planner.smooth_path(
                self.global_path, self.environment.obstacle_map)
            self.path_index = 0
            print(f"全局路径规划成功，包含 {len(self.global_path)} 个路径点")
        else:
            print("全局路径规划失败")
    
    def add_scenario_obstacles(self, scenario: str = "basic"):
        """添加测试场景的障碍物"""
        if scenario == "basic":
            # 基础场景：几个静态障碍物
            self.environment.add_static_obstacle(15, 15, 2)
            self.environment.add_static_obstacle(25, 30, 1.5)
            self.environment.add_static_obstacle(35, 20, 2.5)
            
        elif scenario == "dynamic":
            # 动态场景：移动障碍物
            self.environment.add_dynamic_obstacle(20, 25, 1.5, 0.5, 0.3)
            self.environment.add_dynamic_obstacle(30, 35, 1.8, -0.4, 0.6)
            self.environment.add_dynamic_obstacle(40, 15, 1.2, 0.3, -0.5)
            
        elif scenario == "complex":
            # 复杂场景：静态+动态障碍物
            # 静态障碍物
            self.environment.add_static_obstacle(12, 20, 1.5)
            self.environment.add_static_obstacle(28, 15, 2.0)
            self.environment.add_static_obstacle(35, 35, 1.8)
            
            # 动态障碍物
            self.environment.add_dynamic_obstacle(18, 30, 1.2, 0.6, -0.4)
            self.environment.add_dynamic_obstacle(32, 25, 1.5, -0.5, 0.7)
        
        elif scenario == "dense":
            # 高密度静态障碍物 + 少量动态障碍物，保持可通行性
            rng = np.random.default_rng(42)
            
            # 放置若干较大的固定障碍物，避开起终点与直线连线，形成走廊
            large_obstacles = [
                (14, 16, 2.2), (20, 30, 2.0), (27, 22, 2.4),
                (33, 32, 2.2), (38, 18, 2.6)
            ]
            for x, y, r in large_obstacles:
                self.environment.add_static_obstacle(x, y, r)
            
            # 在地图中部随机撒布小型浮标，半径较小，留出宽通道
            for _ in range(18):
                x = float(rng.integers(8, 44))
                y = float(rng.integers(8, 44))
                # 避免离起点、终点过近，保持主对角线附近留出通道
                if (abs(x - y) < 6) or (x < 8 and y < 8) or (x > 42 and y > 42):
                    continue
                radius = float(rng.uniform(0.8, 1.3))
                self.environment.add_static_obstacle(x, y, radius)
            
            # 少量慢速动态障碍物
            self.environment.add_dynamic_obstacle(22, 27, 1.2, 0.2, -0.15)
            self.environment.add_dynamic_obstacle(36, 24, 1.0, -0.18, 0.2)
    
    def step(self) -> Dict:
        """仿真一步"""
        # 更新动态障碍物
        self.environment.update_dynamic_obstacles(self.dt)
        
        # 获取当前状态
        current_pose = self.usv.get_pose()
        current_velocity = self.usv.get_velocity()
        
        # 记录轨迹
        self.trajectory_history.append(current_pose)
        
        # 检查是否到达目标
        if self.goal and self.trajectory_tracker.is_goal_reached(current_pose, self.goal):
            print("到达目标!")
            self.is_running = False
            return {'status': 'goal_reached'}
        
        # 生成摄像头图像(只有在启用VLM时才生成)
        camera_image = None
        if self.enable_vlm:
            camera_image = self.environment.generate_camera_image(current_pose)
        
        # 局部路径规划
        if self.global_path:
            start_time = time.time()
            
            # 使用DWA进行局部规划
            v_ref, omega_ref, self.path_index = self.local_planner.plan_with_global_path(
                self.usv.state, self.global_path, self.path_index,
                self.environment.get_all_obstacles(), camera_image)
            
            planning_time = (time.time() - start_time) * 1000
            self.planning_time_history.append(planning_time)
            
            # 轨迹跟踪控制
            reference_velocity = (v_ref, omega_ref)
            linear_vel, angular_vel = self.trajectory_tracker.dwa_tracking_control(
                current_pose, reference_velocity, current_velocity, self.current_time)
            
            # 记录控制指令
            self.control_history.append((linear_vel, angular_vel))
            
        else:
            linear_vel, angular_vel = 0.0, 0.0
        
        # 更新USV状态
        self.usv.update(linear_vel, angular_vel, self.dt)
        
        # 更新时间
        self.current_time += self.dt
        
        return {
            'status': 'running',
            'pose': current_pose,
            'velocity': current_velocity,
            'camera_image': camera_image,
            'planning_time': planning_time if 'planning_time' in locals() else 0.0
        }
    
    def run_simulation(self, max_time: float = 60.0, goal: Tuple[float, float] = None):
        """运行仿真"""
        if goal:
            self.set_goal(goal[0], goal[1])
        
        if not self.global_path:
            print("没有有效的全局路径")
            return
        
        self.is_running = True
        step_count = 0
        max_steps = int(max_time / self.dt)
        
        print(f"开始仿真，目标: {self.goal}, 最大时间: {max_time}s")
        
        while self.is_running and step_count < max_steps:
            result = self.step()
            step_count += 1
            
            if step_count % 50 == 0:  # 每5秒打印一次状态
                pose = result['pose']
                print(f"时间: {self.current_time:.1f}s, 位置: ({pose[0]:.2f}, {pose[1]:.2f})")
        
        if step_count >= max_steps:
            print("仿真超时")
        
        return self.get_simulation_results()
    
    def get_simulation_results(self) -> Dict:
        """获取仿真结果"""
        planning_stats = self.local_planner.get_planning_statistics()
        
        return {
            'trajectory': self.trajectory_history,
            'planning_times': self.planning_time_history,
            'control_history': self.control_history,
            'planning_statistics': planning_stats,
            'total_time': self.current_time,
            'path_length': len(self.trajectory_history),
            'goal_reached': not self.is_running
        }
    
    def visualize_simulation(self, show_trajectory: bool = True, 
                           show_obstacles: bool = True, save_path: str = None):
        """可视化仿真结果"""
        plt.figure(figsize=(12, 10))
        
        # 绘制地图
        if show_obstacles:
            plt.imshow(self.environment.obstacle_map, cmap='Greys', origin='lower',
                      extent=[0, self.environment.width, 0, self.environment.height],
                      alpha=0.7)
        
        # 绘制全局路径
        if self.global_path:
            path_x = [p[0] for p in self.global_path]
            path_y = [p[1] for p in self.global_path]
            plt.plot(path_x, path_y, 'b--', linewidth=2, label='Global Path', alpha=0.7)
        
        # 绘制实际轨迹
        if show_trajectory and self.trajectory_history:
            traj_x = [p[0] for p in self.trajectory_history]
            traj_y = [p[1] for p in self.trajectory_history]
            plt.plot(traj_x, traj_y, 'r-', linewidth=3, label='Actual Trajectory')
        
        # 绘制动态障碍物
        for obs in self.environment.dynamic_obstacles:
            circle = plt.Circle((obs['x'], obs['y']), obs['radius'], 
                              color='orange', alpha=0.6, label='Dynamic Obstacle')
            plt.gca().add_patch(circle)
        
        # 绘制起始点和目标点
        if self.trajectory_history:
            start = self.trajectory_history[0]
            plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
        
        if self.goal:
            plt.plot(self.goal[0], self.goal[1], 'ro', markersize=10, label='Goal')
        
        # 绘制当前USV位置
        current_pose = self.usv.get_pose()
        plt.plot(current_pose[0], current_pose[1], 'bs', markersize=8, label='Current USV')
        
        plt.title('USV Navigation Simulation with VLM-Enhanced Obstacle Avoidance')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def reset(self):
        """重置仿真"""
        self.current_time = 0.0
        self.usv = USVModel(5.0, 5.0, 0.0)
        self.is_running = False
        self.goal = None
        self.global_path = None
        self.path_index = 0
        
        # 清空记录
        self.trajectory_history = []
        self.planning_time_history = []
        self.control_history = []
        
        # 重置控制器
        self.trajectory_tracker.reset_controllers()
        self.local_planner.planning_times = [] 