import numpy as np
import heapq
from typing import List, Tuple, Optional
try:  # 可视化是可选依赖：没有 matplotlib 时核心算法依然可以导入和测试
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - 仅在缺少依赖时触发
    plt = None

PLT_HELP = "可视化功能需要 matplotlib，请先执行 pip install matplotlib"
from ..utils.config import Config

class Node:
    """A*算法节点类"""
    def __init__(self, x: int, y: int, cost: float = 0.0, parent: Optional['Node'] = None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = 0.0
        self.parent = parent
        
    @property
    def f_cost(self) -> float:
        return self.cost + self.heuristic
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class AStarPlanner:
    """A*全局路径规划器"""
    
    def __init__(self, grid_size: float = Config.ASTAR_GRID_SIZE):
        self.grid_size = grid_size
        self.heuristic_weight = Config.ASTAR_HEURISTIC_WEIGHT
        self.obstacle_cost = Config.ASTAR_OBSTACLE_COST
        
        # 8个方向的移动 (dx, dy, cost)
        self.motions = [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, np.sqrt(2)),
            (-1, 1, np.sqrt(2)),
            (-1, -1, np.sqrt(2)),
            (1, -1, np.sqrt(2))
        ]
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转换为网格坐标"""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """网格坐标转换为世界坐标"""
        x = grid_x * self.grid_size + self.grid_size / 2
        y = grid_y * self.grid_size + self.grid_size / 2
        return x, y
    
    def heuristic(self, node: Node, goal: Node) -> float:
        """启发式函数 - 欧几里得距离"""
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        return self.heuristic_weight * np.sqrt(dx*dx + dy*dy)
    
    def is_valid(self, x: int, y: int, obstacle_map: np.ndarray) -> bool:
        """检查节点是否有效"""
        height, width = obstacle_map.shape
        if x < 0 or x >= width or y < 0 or y >= height:
            return False
        return obstacle_map[y, x] == 0
    
    def get_neighbors(self, node: Node, obstacle_map: np.ndarray) -> List[Node]:
        """获取邻近节点"""
        neighbors = []
        
        for dx, dy, cost in self.motions:
            new_x = node.x + dx
            new_y = node.y + dy
            
            if self.is_valid(new_x, new_y, obstacle_map):
                new_cost = node.cost + cost
                neighbor = Node(new_x, new_y, new_cost, node)
                neighbors.append(neighbor)
        
        return neighbors
    
    def reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """重构路径"""
        path = []
        current = node
        
        while current is not None:
            x, y = self.grid_to_world(current.x, current.y)
            path.append((x, y))
            current = current.parent
        
        path.reverse()
        return path
    
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float], 
             obstacle_map: np.ndarray) -> Optional[List[Tuple[float, float]]]:
        """A*路径规划主函数
        
        Args:
            start: 起始点世界坐标 (x, y)
            goal: 目标点世界坐标 (x, y)
            obstacle_map: 障碍物地图 (0: 自由空间, 1: 障碍物)
        
        Returns:
            路径点列表 [(x1, y1), (x2, y2), ...] 或 None
        """
        # 转换为网格坐标
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        # 检查起始点和目标点是否有效
        if not self.is_valid(start_grid[0], start_grid[1], obstacle_map):
            print("起始点位于障碍物中")
            return None
        
        if not self.is_valid(goal_grid[0], goal_grid[1], obstacle_map):
            print("目标点位于障碍物中")
            return None
        
        # 初始化
        start_node = Node(start_grid[0], start_grid[1])
        goal_node = Node(goal_grid[0], goal_grid[1])
        
        open_set = []
        closed_set = set()
        
        start_node.heuristic = self.heuristic(start_node, goal_node)
        heapq.heappush(open_set, start_node)
        
        # A*主循环
        while open_set:
            current = heapq.heappop(open_set)
            
            # 到达目标
            if current == goal_node:
                return self.reconstruct_path(current)
            
            closed_set.add((current.x, current.y))
            
            # 扩展邻近节点
            for neighbor in self.get_neighbors(current, obstacle_map):
                if (neighbor.x, neighbor.y) in closed_set:
                    continue
                
                neighbor.heuristic = self.heuristic(neighbor, goal_node)
                
                # 检查是否已在开放集中
                in_open = False
                for i, open_node in enumerate(open_set):
                    if neighbor == open_node:
                        if neighbor.f_cost < open_node.f_cost:
                            open_set[i] = neighbor
                            heapq.heapify(open_set)
                        in_open = True
                        break
                
                if not in_open:
                    heapq.heappush(open_set, neighbor)
        
        print("未找到路径")
        return None
    
    def smooth_path(self, path: List[Tuple[float, float]], 
                   obstacle_map: np.ndarray) -> List[Tuple[float, float]]:
        """路径平滑化"""
        if len(path) <= 2:
            return path
        
        smoothed_path = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            
            # 从最远点开始检查直线路径
            while j > i + 1:
                if self._is_line_clear(path[i], path[j], obstacle_map):
                    smoothed_path.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                smoothed_path.append(path[i + 1])
                i += 1
        
        return smoothed_path
    
    def _is_line_clear(self, start: Tuple[float, float], end: Tuple[float, float],
                      obstacle_map: np.ndarray) -> bool:
        """检查两点间直线是否无障碍"""
        x1, y1 = start
        x2, y2 = end
        
        # Bresenham直线算法检查
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        steps = int(max(dx, dy) / self.grid_size) + 1
        
        for i in range(steps + 1):
            t = i / steps if steps > 0 else 0
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            grid_x, grid_y = self.world_to_grid(x, y)
            if not self.is_valid(grid_x, grid_y, obstacle_map):
                return False
        
        return True
    
    def visualize_planning(self, start: Tuple[float, float], goal: Tuple[float, float],
                          path: List[Tuple[float, float]], obstacle_map: np.ndarray,
                          title: str = "A* Global Path Planning"):
        """可视化路径规划结果"""
        if plt is None:
            raise RuntimeError(PLT_HELP)
        plt.figure(figsize=(10, 10))
        
        # 显示地图
        plt.imshow(obstacle_map, cmap='binary', origin='lower', 
                  extent=[0, obstacle_map.shape[1] * self.grid_size,
                         0, obstacle_map.shape[0] * self.grid_size])
        
        # 显示路径
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            plt.plot(path_x, path_y, 'r-', linewidth=3, label='A* Path')
            plt.plot(path_x, path_y, 'ro', markersize=5)
        
        # 显示起始点和目标点
        plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
        plt.plot(goal[0], goal[1], 'bo', markersize=10, label='Goal')
        
        plt.title(title)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.show() 