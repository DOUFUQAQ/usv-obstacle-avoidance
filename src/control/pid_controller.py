import numpy as np
import math
from typing import Tuple, List
from ..utils.config import Config

class PIDController:
    """PID控制器类"""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limit: Tuple[float, float] = (-float('inf'), float('inf'))):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        # 内部状态
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        
    def update(self, error: float, current_time: float) -> float:
        """更新PID控制器
        
        Args:
            error: 误差值
            current_time: 当前时间戳
        
        Returns:
            控制输出
        """
        if self.previous_time is None:
            self.previous_time = current_time
            dt = 0.1  # 默认时间步长
        else:
            dt = current_time - self.previous_time
            if dt <= 0:
                dt = 0.1
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        # 微分项
        derivative = (error - self.previous_error) / dt
        derivative_term = self.kd * derivative
        
        # 总输出
        output = proportional + integral_term + derivative_term
        
        # 输出限制
        output = max(min(output, self.output_limit[1]), self.output_limit[0])
        
        # 抗积分饱和
        if output == self.output_limit[0] or output == self.output_limit[1]:
            self.integral -= error * dt  # 回退积分项
        
        # 更新状态
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def reset(self):
        """重置PID控制器状态"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

class TrajectoryTracker:
    """轨迹跟踪控制器"""
    
    def __init__(self):
        # 初始化PID控制器
        self.linear_pid = PIDController(
            kp=Config.PID_KP_LINEAR,
            ki=Config.PID_KI_LINEAR,
            kd=Config.PID_KD_LINEAR,
            output_limit=(-Config.DWA_MAX_SPEED, Config.DWA_MAX_SPEED)
        )
        
        self.angular_pid = PIDController(
            kp=Config.PID_KP_ANGULAR,
            ki=Config.PID_KI_ANGULAR,
            kd=Config.PID_KD_ANGULAR,
            output_limit=(-Config.DWA_MAX_ANGULAR_SPEED, Config.DWA_MAX_ANGULAR_SPEED)
        )
        
        # 跟踪参数
        self.lookahead_distance = 2.0  # 前瞻距离
        self.goal_tolerance = 0.5  # 目标容差
        
    def pure_pursuit_control(self, current_pose: Tuple[float, float, float],
                           path: List[Tuple[float, float]],
                           current_time: float) -> Tuple[float, float]:
        """Pure Pursuit轨迹跟踪控制
        
        Args:
            current_pose: 当前位姿 (x, y, yaw)
            path: 路径点列表 [(x1, y1), (x2, y2), ...]
            current_time: 当前时间戳
        
        Returns:
            控制指令 (linear_velocity, angular_velocity)
        """
        if not path:
            return 0.0, 0.0
        
        current_x, current_y, current_yaw = current_pose
        
        # 寻找前瞻点
        lookahead_point = self._find_lookahead_point(current_pose, path)
        
        if lookahead_point is None:
            return 0.0, 0.0
        
        target_x, target_y = lookahead_point
        
        # 计算到目标点的距离和角度
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        
        # 角度误差
        angle_error = self._normalize_angle(target_angle - current_yaw)
        
        # 使用PID控制器计算控制指令
        linear_velocity = self.linear_pid.update(distance, current_time)
        angular_velocity = self.angular_pid.update(angle_error, current_time)
        
        # 如果角度误差较大，降低线速度
        if abs(angle_error) > math.pi / 4:
            linear_velocity *= 0.5
        
        return linear_velocity, angular_velocity
    
    def stanley_control(self, current_pose: Tuple[float, float, float],
                       path: List[Tuple[float, float]],
                       current_velocity: float,
                       current_time: float) -> Tuple[float, float]:
        """Stanley轨迹跟踪控制
        
        Args:
            current_pose: 当前位姿 (x, y, yaw)
            path: 路径点列表
            current_velocity: 当前速度
            current_time: 当前时间戳
        
        Returns:
            控制指令 (linear_velocity, angular_velocity)
        """
        if not path or len(path) < 2:
            return 0.0, 0.0
        
        current_x, current_y, current_yaw = current_pose
        
        # 寻找最近的路径点
        closest_point_index = self._find_closest_point(current_pose, path)
        
        if closest_point_index >= len(path) - 1:
            closest_point_index = len(path) - 2
        
        # 计算路径切线方向
        p1 = path[closest_point_index]
        p2 = path[closest_point_index + 1]
        path_angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        
        # 计算横向误差
        dx = current_x - p1[0]
        dy = current_y - p1[1]
        cross_track_error = -math.sin(path_angle) * dx + math.cos(path_angle) * dy
        
        # Stanley控制律
        heading_error = self._normalize_angle(current_yaw - path_angle)
        
        # 横向误差对应的角度修正
        k_e = 0.3  # 横向误差增益
        cross_track_steering = math.atan(k_e * cross_track_error / (current_velocity + 0.1))
        
        # 总的角度误差
        total_angle_error = heading_error + cross_track_steering
        
        # 使用PID控制器
        linear_velocity = self.linear_pid.update(
            Config.DWA_MAX_SPEED - current_velocity, current_time)
        angular_velocity = self.angular_pid.update(total_angle_error, current_time)
        
        return linear_velocity, angular_velocity
    
    def dwa_tracking_control(self, current_pose: Tuple[float, float, float],
                           reference_velocity: Tuple[float, float],
                           current_velocity: Tuple[float, float],
                           current_time: float) -> Tuple[float, float]:
        """DWA规划结果的跟踪控制
        
        Args:
            current_pose: 当前位姿
            reference_velocity: 参考速度 (v_ref, omega_ref)
            current_velocity: 当前速度 (v_current, omega_current)
            current_time: 当前时间戳
        
        Returns:
            控制指令 (linear_velocity, angular_velocity)
        """
        v_ref, omega_ref = reference_velocity
        v_current, omega_current = current_velocity
        
        # 速度误差
        linear_error = v_ref - v_current
        angular_error = omega_ref - omega_current
        
        # PID控制
        linear_velocity = self.linear_pid.update(linear_error, current_time)
        angular_velocity = self.angular_pid.update(angular_error, current_time)
        
        # 添加前馈项
        linear_velocity += v_ref
        angular_velocity += omega_ref
        
        return linear_velocity, angular_velocity
    
    def _find_lookahead_point(self, current_pose: Tuple[float, float, float],
                             path: List[Tuple[float, float]]) -> Tuple[float, float]:
        """寻找前瞻点"""
        current_x, current_y, _ = current_pose
        
        # 从路径中寻找距离当前位置lookahead_distance的点
        for i, (path_x, path_y) in enumerate(path):
            distance = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            if distance >= self.lookahead_distance:
                return path_x, path_y
        
        # 如果没有找到合适的前瞻点，返回路径终点
        return path[-1] if path else None
    
    def _find_closest_point(self, current_pose: Tuple[float, float, float],
                           path: List[Tuple[float, float]]) -> int:
        """寻找路径上最近的点"""
        current_x, current_y, _ = current_pose
        min_distance = float('inf')
        closest_index = 0
        
        for i, (path_x, path_y) in enumerate(path):
            distance = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        return closest_index
    
    def _normalize_angle(self, angle: float) -> float:
        """角度归一化到[-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_goal_reached(self, current_pose: Tuple[float, float, float],
                       goal: Tuple[float, float]) -> bool:
        """检查是否到达目标"""
        current_x, current_y, _ = current_pose
        goal_x, goal_y = goal
        
        distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        return distance < self.goal_tolerance
    
    def reset_controllers(self):
        """重置所有PID控制器"""
        self.linear_pid.reset()
        self.angular_pid.reset()
    
    def set_parameters(self, linear_gains: Tuple[float, float, float] = None,
                      angular_gains: Tuple[float, float, float] = None,
                      lookahead_distance: float = None,
                      goal_tolerance: float = None):
        """设置控制器参数"""
        if linear_gains:
            kp, ki, kd = linear_gains
            self.linear_pid.kp = kp
            self.linear_pid.ki = ki
            self.linear_pid.kd = kd
        
        if angular_gains:
            kp, ki, kd = angular_gains
            self.angular_pid.kp = kp
            self.angular_pid.ki = ki
            self.angular_pid.kd = kd
        
        if lookahead_distance is not None:
            self.lookahead_distance = lookahead_distance
        
        if goal_tolerance is not None:
            self.goal_tolerance = goal_tolerance 