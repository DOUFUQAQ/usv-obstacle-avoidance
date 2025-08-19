import numpy as np
import math
from typing import List, Tuple, Optional
try:  # 可视化是可选依赖：没有 matplotlib 时核心算法依然可以导入和测试
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - 仅在缺少依赖时触发
    plt = None

PLT_HELP = "可视化功能需要 matplotlib，请先执行 pip install matplotlib"
import time
from ..utils.config import Config
from ..perception.vlm_detector import VLMObstacleDetector

class DynamicState:
    """动态状态类"""
    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, 
                 v: float = 0.0, omega: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v  # 线速度
        self.omega = omega  # 角速度

class DWALocalPlanner:
    """DWA局部路径规划器"""
    
    def __init__(self, enable_vlm: bool = True):
        # DWA参数
        self.max_speed = Config.DWA_MAX_SPEED
        self.min_speed = Config.DWA_MIN_SPEED
        self.max_angular_speed = Config.DWA_MAX_ANGULAR_SPEED
        self.min_angular_speed = Config.DWA_MIN_ANGULAR_SPEED
        self.max_accel = Config.DWA_MAX_ACCEL
        self.max_angular_accel = Config.DWA_MAX_ANGULAR_ACCEL
        self.velocity_resolution = Config.DWA_VELOCITY_RESOLUTION
        self.angular_resolution = Config.DWA_ANGULAR_RESOLUTION
        self.dt = Config.DWA_DT
        self.predict_time = Config.DWA_PREDICT_TIME
        self.to_goal_cost_gain = Config.DWA_TO_GOAL_COST_GAIN
        self.speed_cost_gain = Config.DWA_SPEED_COST_GAIN
        self.obstacle_cost_gain = Config.DWA_OBSTACLE_COST_GAIN
        self.path_cost_gain = getattr(Config, 'DWA_PATH_COST_GAIN', 0.0)
        self.robot_radius = Config.DWA_ROBOT_RADIUS
        
        # VLM检测器
        self.enable_vlm = enable_vlm
        if enable_vlm:
            self.vlm_detector = VLMObstacleDetector()
        else:
            self.vlm_detector = None
        
        # 性能统计
        self.planning_times = []
        
    def motion_model(self, state: DynamicState, v: float, omega: float) -> DynamicState:
        """运动模型"""
        new_state = DynamicState()
        new_state.x = state.x + v * math.cos(state.yaw) * self.dt
        new_state.y = state.y + v * math.sin(state.yaw) * self.dt
        new_state.yaw = state.yaw + omega * self.dt
        new_state.v = v
        new_state.omega = omega
        return new_state
    
    def calc_dynamic_window(self, state: DynamicState) -> Tuple[float, float, float, float]:
        """计算动态窗口"""
        # 基于速度限制的窗口
        vs = [self.min_speed, self.max_speed,
              self.min_angular_speed, self.max_angular_speed]
        
        # 基于加速度限制的窗口
        vd = [state.v - self.max_accel * self.dt,
              state.v + self.max_accel * self.dt,
              state.omega - self.max_angular_accel * self.dt,
              state.omega + self.max_angular_accel * self.dt]
        
        # 取交集
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        
        return dw
    
    def predict_trajectory(self, state: DynamicState, v: float, omega: float) -> List[DynamicState]:
        """预测轨迹"""
        trajectory = []
        current_state = DynamicState(state.x, state.y, state.yaw, state.v, state.omega)
        
        time = 0
        while time <= self.predict_time:
            current_state = self.motion_model(current_state, v, omega)
            trajectory.append(DynamicState(current_state.x, current_state.y, 
                                         current_state.yaw, current_state.v, 
                                         current_state.omega))
            time += self.dt
        
        return trajectory
    
    def calc_obstacle_cost(self, trajectory: List[DynamicState], 
                          obstacles: List[Tuple[float, float, float]]) -> float:
        """计算障碍物代价：加入强制清距，离障越近代价急剧上升"""
        min_clearance = float("inf")
        required_clearance = self.robot_radius + Config.SAFETY_MARGIN
        
        for state in trajectory:
            for ox, oy, radius in obstacles:
                dx = state.x - ox
                dy = state.y - oy
                dist = math.sqrt(dx * dx + dy * dy)
                # 真实清距：到障碍边界的距离
                clearance = dist - radius
                if clearance <= required_clearance:
                    # 违反安全清距，视作高代价近似碰撞
                    return float("inf")
                if clearance < min_clearance:
                    min_clearance = clearance
        
        if min_clearance == float("inf"):
            return 0.0
        # 采用平方反比，使得越近代价增长更快；
        # 并在安全距离之外增加缓冲区，使规划偏好与障碍保持更大间距
        buffer = 0.4
        effective_clearance = max(1e-6, (min_clearance - required_clearance + buffer))
        return 1.0 / (effective_clearance ** 2)
    
    def calc_to_goal_cost(self, trajectory: List[DynamicState], goal: Tuple[float, float]) -> float:
        """计算到达目标的代价"""
        if not trajectory:
            return float("inf")
        
        # 使用轨迹末端状态
        end_state = trajectory[-1]
        
        # 距离代价 - 距离越近越好
        dx = goal[0] - end_state.x
        dy = goal[1] - end_state.y
        distance = math.sqrt(dx * dx + dy * dy)
        
        # 角度代价 - 朝向目标方向越好
        target_angle = math.atan2(dy, dx)
        angle_diff = abs(math.atan2(math.sin(target_angle - end_state.yaw), 
                                   math.cos(target_angle - end_state.yaw)))
        
        # 综合代价：距离代价 + 角度代价
        return distance + angle_diff
    
    def calc_speed_cost(self, trajectory: List[DynamicState]) -> float:
        """计算速度代价 - 鼓励更高的速度"""
        if not trajectory:
            return float("inf")
        # 速度代价：速度越高，代价越低
        speed = trajectory[-1].v
        return (self.max_speed - speed) / self.max_speed
    
    def integrate_vlm_obstacles(self, current_image: Optional[np.ndarray],
                               robot_pose: Tuple[float, float, float],
                               static_obstacles: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """集成VLM检测的障碍物"""
        all_obstacles = static_obstacles.copy()
        
        if self.enable_vlm and self.vlm_detector and current_image is not None:
            try:
                # VLM障碍物检测
                detection_result = self.vlm_detector.detect_obstacles(current_image)
                
                if detection_result:
                    # 处理检测结果
                    vlm_obstacles, safe_directions = self.vlm_detector.process_detection_result(
                        detection_result, current_image.shape[:2])
                    
                    # 转换为世界坐标
                    world_obstacles = self.vlm_detector.convert_to_world_coordinates(
                        vlm_obstacles, robot_pose)
                    
                    # 添加到障碍物列表
                    all_obstacles.extend(world_obstacles)
                    
                    # 根据安全方向调整障碍物权重（可选）
                    # 这里可以添加基于安全方向的逻辑
                    
            except Exception as e:
                print(f"VLM障碍物检测失败: {e}")
        
        return all_obstacles
    
    def plan(self, state: DynamicState, goal: Tuple[float, float],
             static_obstacles: List[Tuple[float, float, float]],
             current_image: Optional[np.ndarray] = None,
             global_path: Optional[List[Tuple[float, float]]] = None,
             path_index: int = 0) -> Tuple[float, float]:
        """DWA局部路径规划主函数
        
        Args:
            state: 当前状态
            goal: 目标点 (x, y)
            static_obstacles: 静态障碍物列表 [(x, y, radius), ...]
            current_image: 当前摄像头图像 (可选)
        
        Returns:
            最优控制指令 (v, omega)
        """
        start_time = time.time()
        
        # 集成VLM检测的障碍物
        all_obstacles = self.integrate_vlm_obstacles(
            current_image, (state.x, state.y, state.yaw), static_obstacles)
        
        # 计算动态窗口
        dw = self.calc_dynamic_window(state)
        
        min_cost = float("inf")
        best_v = 0.0
        best_omega = 0.0
        best_trajectory = []
        
        # 遍历速度空间
        v = dw[0]
        while v <= dw[1]:
            omega = dw[2]
            while omega <= dw[3]:
                # 预测轨迹
                trajectory = self.predict_trajectory(state, v, omega)
                
                # 计算代价
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.calc_speed_cost(trajectory)
                obstacle_cost = self.calc_obstacle_cost(trajectory, all_obstacles)
                # 贴合全局路径代价（可选）
                path_cost = 0.0
                if self.path_cost_gain > 0.0 and global_path is not None:
                    path_cost = self.calc_path_adherence_cost(trajectory, global_path, path_index)
                
                # 总代价
                final_cost = (self.to_goal_cost_gain * to_goal_cost +
                            self.speed_cost_gain * speed_cost +
                            self.obstacle_cost_gain * obstacle_cost +
                            self.path_cost_gain * path_cost)
                
                # 更新最优解
                if final_cost < min_cost:
                    min_cost = final_cost
                    best_v = v
                    best_omega = omega
                    best_trajectory = trajectory
                
                omega += self.angular_resolution
            v += self.velocity_resolution

        # 若最优解仍为零速度且存在障碍过近情况，尝试注入小幅线速度以摆脱局部最小
        if min_cost == float("inf") or (abs(best_v) < 1e-3 and abs(best_omega) < 1e-3):
            v_try = max(self.velocity_resolution, 0.2)
            omega_try = 0.0
            traj_try = self.predict_trajectory(state, v_try, omega_try)
            if self.calc_obstacle_cost(traj_try, all_obstacles) < float("inf"):
                best_v, best_omega = v_try, omega_try
        
        # 记录规划时间
        planning_time = (time.time() - start_time) * 1000  # 转换为毫秒
        self.planning_times.append(planning_time)
        
        return best_v, best_omega
    
    def plan_with_global_path(self, state: DynamicState, 
                             global_path: List[Tuple[float, float]],
                             path_index: int,
                             static_obstacles: List[Tuple[float, float, float]],
                             current_image: Optional[np.ndarray] = None,
                             lookahead_distance: float = 5.0) -> Tuple[float, float, int]:
        """基于全局路径的局部规划
        
        Args:
            state: 当前状态
            global_path: 全局路径
            path_index: 当前路径索引
            static_obstacles: 静态障碍物
            current_image: 当前图像
            lookahead_distance: 前瞻距离
        
        Returns:
            控制指令和更新的路径索引 (v, omega, new_path_index)
        """
        if not global_path or path_index >= len(global_path):
            return 0.0, 0.0, path_index
        
        # 首先更新路径索引到最近点
        new_path_index = self._update_path_index(state, global_path, path_index)
        
        # 自适应前瞻距离：若索引未推进或局部曲率大，适度增加
        adaptive_lookahead = lookahead_distance
        if new_path_index <= path_index:
            adaptive_lookahead = min(12.0, lookahead_distance * 1.6)
        # 寻找前瞻点
        target_point = self._find_lookahead_point(
            state, global_path, new_path_index, adaptive_lookahead)
        
        # 使用前瞻点作为局部目标
        v, omega = self.plan(state, target_point, static_obstacles, current_image,
                             global_path=global_path, path_index=new_path_index)
        
        return v, omega, new_path_index

    def calc_path_adherence_cost(self, trajectory: List[DynamicState],
                                 global_path: List[Tuple[float, float]],
                                 start_index: int) -> float:
        """计算轨迹终点与全局路径的贴合代价（距离越大代价越高）"""
        if not trajectory or not global_path:
            return 0.0
        end_state = trajectory[-1]
        window_end = min(len(global_path), start_index + 30)
        if start_index >= window_end:
            start_index = max(0, window_end - 1)
        min_dist = float('inf')
        for px, py in global_path[start_index:window_end]:
            d = math.hypot(px - end_state.x, py - end_state.y)
            if d < min_dist:
                min_dist = d
        return 0.0 if min_dist == float('inf') else min_dist
    
    def _find_lookahead_point(self, state: DynamicState, 
                             global_path: List[Tuple[float, float]],
                             start_index: int, 
                             lookahead_distance: float) -> Tuple[float, float]:
        """寻找前瞻点"""
        current_x, current_y = state.x, state.y
        
        # 确保至少前进一个点
        min_index = min(start_index + 1, len(global_path) - 1)
        
        for i in range(min_index, len(global_path)):
            path_x, path_y = global_path[i]
            dist = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            if dist >= lookahead_distance:
                return path_x, path_y
        
        # 如果没有找到足够远的点，返回路径终点
        return global_path[-1]
    
    def _update_path_index(self, state: DynamicState, 
                          global_path: List[Tuple[float, float]],
                          current_index: int) -> int:
        """更新路径索引"""
        current_x, current_y = state.x, state.y
        min_dist = float("inf")
        closest_index = current_index
        
        # 在当前索引附近寻找最近点
        search_range = min(10, len(global_path) - current_index)
        for i in range(current_index, current_index + search_range):
            if i >= len(global_path):
                break
            
            path_x, path_y = global_path[i]
            dist = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        
        return closest_index
    
    def get_planning_statistics(self) -> dict:
        """获取规划性能统计"""
        if not self.planning_times:
            return {}
        
        times = np.array(self.planning_times)
        return {
            "average_time_ms": np.mean(times),
            "max_time_ms": np.max(times),
            "min_time_ms": np.min(times),
            "std_time_ms": np.std(times),
            "total_calls": len(times)
        }
    
    def visualize_planning(self, state: DynamicState, goal: Tuple[float, float],
                          obstacles: List[Tuple[float, float, float]],
                          trajectory: List[DynamicState] = None,
                          title: str = "DWA Local Planning"):
        """可视化局部规划结果"""
        if plt is None:
            raise RuntimeError(PLT_HELP)
        plt.figure(figsize=(10, 8))
        
        # 绘制障碍物
        for ox, oy, radius in obstacles:
            circle = plt.Circle((ox, oy), radius, color='red', alpha=0.7)
            plt.gca().add_patch(circle)
        
        # 绘制机器人
        robot_circle = plt.Circle((state.x, state.y), self.robot_radius, 
                                color='blue', alpha=0.7)
        plt.gca().add_patch(robot_circle)
        
        # 绘制机器人朝向
        dx = self.robot_radius * math.cos(state.yaw)
        dy = self.robot_radius * math.sin(state.yaw)
        plt.arrow(state.x, state.y, dx, dy, head_width=0.2, 
                 head_length=0.2, fc='blue', ec='blue')
        
        # 绘制目标点
        plt.plot(goal[0], goal[1], 'go', markersize=10, label='Goal')
        
        # 绘制预测轨迹
        if trajectory:
            traj_x = [s.x for s in trajectory]
            traj_y = [s.y for s in trajectory]
            plt.plot(traj_x, traj_y, 'g-', linewidth=2, label='Predicted Trajectory')
        
        plt.title(title)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.show() 