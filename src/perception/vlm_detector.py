import base64
import io
import json
import os
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

from ..utils.config import Config

try:  # OpenCV 只用于图像编码与可视化，缺失时回退到 numpy / PIL
    import cv2
except Exception:  # pragma: no cover - 仅在缺少依赖时触发
    cv2 = None

CV2_HELP = "该功能需要 OpenCV，请先执行 pip install opencv-python"
REQUESTS_HELP = "调用 VLM API 需要 requests，请先执行 pip install requests"

class VLMObstacleDetector:
    """VLM障碍物检测器"""
    
    def __init__(self, model: str = Config.VLM_MODEL):
        # 使用配置文件中的API密钥
        self.api_key = Config.OPENAI_API_KEY
        self.base_url = Config.OPENAI_BASE_URL
        self.model = model
        self.headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        # 检测提示词
        self.detection_prompt = """分析USV摄像头图像，检测障碍物和安全区域。严格按JSON格式回复，不要其他内容：

{
  "obstacles": [
    {
      "type": "船只",
      "confidence": 0.95,
      "bbox": [100, 200, 300, 400],
      "distance": "中距离",
      "risk_level": "中"
    }
  ],
  "safe_areas": [
    {
      "direction": "左侧",
      "confidence": 0.9
    }
  ]
}"""
    
    def encode_image(self, image: np.ndarray) -> str:
        """将图像编码为base64字符串"""
        # 转换BGR到RGB（没有 OpenCV 时用 numpy 翻转通道）
        if len(image.shape) == 3 and image.shape[2] == 3:
            if cv2 is not None:
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                image_rgb = image[:, :, ::-1]
        else:
            image_rgb = image
        
        # 转换为PIL图像
        pil_image = Image.fromarray(image_rgb)
        
        # 压缩图像以减少API调用成本
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=85)
        image_bytes = buffer.getvalue()
        
        # 编码为base64
        return base64.b64encode(image_bytes).decode('utf-8')
    
    def detect_obstacles(self, image: np.ndarray, 
                        async_mode: bool = False) -> Optional[Dict]:
        """检测图像中的障碍物
        
        Args:
            image: 输入图像 (BGR格式)
            async_mode: 是否使用异步模式
        
        Returns:
            检测结果字典或None
        """
        # 局部导入：只做 JSON 解析 / 坐标投影的调用方不应该被迫安装 requests
        try:
            import requests
        except ImportError as exc:  # pragma: no cover - 依赖缺失路径
            raise RuntimeError(REQUESTS_HELP) from exc

        try:
            # 编码图像
            base64_image = self.encode_image(image)
            
            # 构建请求数据
            data = {
                "model": self.model,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": self.detection_prompt
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}"
                                }
                            }
                        ]
                    }
                ],
                "max_tokens": 1000,
                "temperature": 0.1
            }
            
            # 发送请求 (增加重试机制)
            max_retries = 2
            for attempt in range(max_retries):
                try:
                    response = requests.post(
                        f"{self.base_url}/chat/completions",
                        headers=self.headers,
                        json=data,
                        timeout=20  # 减少超时时间
                    )
                    break
                except (requests.exceptions.Timeout, requests.exceptions.ConnectionError) as e:
                    if attempt == max_retries - 1:
                        raise e
                    print(f"VLM请求超时，重试 {attempt + 1}/{max_retries}")
                    time.sleep(1)
            
            if response.status_code == 200:
                result = response.json()
                content = result['choices'][0]['message']['content']
                
                # 解析JSON结果
                try:
                    # 多种方式提取JSON
                    json_str = self._extract_json_content(content)
                    if json_str:
                        detection_result = json.loads(json_str)
                        return detection_result
                    else:
                        print("VLM返回结果中未找到有效JSON")
                        print(f"原始内容: {content[:200]}...")
                        return None
                except json.JSONDecodeError as e:
                    print(f"JSON解析错误: {e}")
                    print(f"VLM原始返回: {content[:200]}...")
                    return None
            else:
                print(f"VLM API请求失败: {response.status_code}")
                print(f"错误信息: {response.text}")
                return None
                
        except Exception as e:
            print(f"VLM检测出错: {e}")
            return None
    
    def _extract_json_content(self, content: str) -> Optional[str]:
        """从响应内容中提取JSON字符串"""
        import re
        
        # 方法1: 查找第一个完整的JSON对象
        json_start = content.find('{')
        if json_start != -1:
            brace_count = 0
            for i, char in enumerate(content[json_start:], json_start):
                if char == '{':
                    brace_count += 1
                elif char == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        return content[json_start:i+1]
        
        # 方法2: 使用正则表达式查找JSON模式
        json_pattern = r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}'
        matches = re.findall(json_pattern, content)
        if matches:
            return matches[0]
        
        # 方法3: 查找代码块中的JSON
        code_block_pattern = r'```(?:json)?\s*(\{.*?\})\s*```'
        matches = re.findall(code_block_pattern, content, re.DOTALL)
        if matches:
            return matches[0]
        
        return None
    
    def process_detection_result(self, detection_result: Dict, 
                               image_shape: Tuple[int, int]) -> Tuple[List[Tuple[float, float, float]], List[str]]:
        """处理VLM检测结果，转换为局部规划器可用的格式
        
        Args:
            detection_result: VLM检测结果
            image_shape: 图像尺寸 (height, width)
        
        Returns:
            obstacles: 障碍物列表 [(x, y, radius), ...]
            safe_directions: 安全方向列表
        """
        obstacles = []
        safe_directions = []
        
        if not detection_result or 'obstacles' not in detection_result:
            return obstacles, safe_directions
        
        height, width = image_shape
        
        # 处理障碍物
        for obs in detection_result.get('obstacles', []):
            if 'bbox' in obs and len(obs['bbox']) == 4:
                x_min, y_min, x_max, y_max = obs['bbox']
                
                # 计算障碍物中心和大小
                center_x = (x_min + x_max) / 2
                center_y = (y_min + y_max) / 2
                
                # 转换为归一化坐标 (0-1)
                norm_x = center_x / width
                norm_y = center_y / height
                
                # 估算障碍物半径
                bbox_width = x_max - x_min
                bbox_height = y_max - y_min
                radius = max(bbox_width, bbox_height) / (2 * max(width, height))
                
                # 根据风险级别调整半径
                risk_level = obs.get('risk_level', '中')
                if risk_level == '高':
                    radius *= 1.5
                elif risk_level == '低':
                    radius *= 0.8
                
                obstacles.append((norm_x, norm_y, radius))
        
        # 处理安全区域
        for safe_area in detection_result.get('safe_areas', []):
            direction = safe_area.get('direction', '')
            if direction:
                safe_directions.append(direction)
        
        return obstacles, safe_directions
    
    def convert_to_world_coordinates(self, obstacles: List[Tuple[float, float, float]], 
                                   robot_pose: Tuple[float, float, float],
                                   camera_range: float = None) -> List[Tuple[float, float, float]]:
        """将归一化的障碍物坐标转换为世界坐标
        
        Args:
            obstacles: 归一化障碍物列表 [(norm_x, norm_y, norm_radius), ...]
            robot_pose: 机器人位姿 (x, y, yaw)
            camera_range: 摄像头检测范围 (米)
        
        Returns:
            世界坐标系下的障碍物列表 [(world_x, world_y, world_radius), ...]
        """
        world_obstacles = []
        if camera_range is None:
            camera_range = getattr(Config, 'VLM_CAMERA_RANGE', 10.0)
        robot_x, robot_y, robot_yaw = robot_pose
        
        for norm_x, norm_y, norm_radius in obstacles:
            # 转换为相对坐标（前方为x轴正方向）
            rel_x = (norm_y - 0.5) * camera_range  # 前方距离
            rel_y = (0.5 - norm_x) * camera_range  # 左右偏移
            
            # 转换为世界坐标
            cos_yaw = np.cos(robot_yaw)
            sin_yaw = np.sin(robot_yaw)
            
            world_x = robot_x + rel_x * cos_yaw - rel_y * sin_yaw
            world_y = robot_y + rel_x * sin_yaw + rel_y * cos_yaw
            # 对VLM障碍进行更保守的膨胀处理
            inflation_factor = getattr(Config, 'VLM_OBS_INFLATION_FACTOR', 1.0)
            extra_margin = getattr(Config, 'VLM_OBS_EXTRA_MARGIN', 0.0)
            world_radius = norm_radius * camera_range * inflation_factor + extra_margin
            
            world_obstacles.append((world_x, world_y, world_radius))
        
        return world_obstacles
    
    def visualize_detection(self, image: np.ndarray, detection_result: Dict) -> np.ndarray:
        """可视化检测结果"""
        if cv2 is None:
            raise RuntimeError(CV2_HELP)
        vis_image = image.copy()
        
        if not detection_result or 'obstacles' not in detection_result:
            return vis_image
        
        # 绘制障碍物边框
        for obs in detection_result.get('obstacles', []):
            if 'bbox' in obs and len(obs['bbox']) == 4:
                x_min, y_min, x_max, y_max = obs['bbox']
                
                # 根据风险级别选择颜色
                risk_level = obs.get('risk_level', '中')
                if risk_level == '高':
                    color = (0, 0, 255)  # 红色
                elif risk_level == '中':
                    color = (0, 165, 255)  # 橙色
                else:
                    color = (0, 255, 255)  # 黄色
                
                # 绘制边框
                cv2.rectangle(vis_image, (int(x_min), int(y_min)), 
                            (int(x_max), int(y_max)), color, 2)
                
                # 添加标签
                label = f"{obs.get('type', '未知')} ({risk_level})"
                cv2.putText(vis_image, label, (int(x_min), int(y_min) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return vis_image
    
    def test_connection(self) -> bool:
        """测试VLM API连接"""
        try:
            # 创建测试图像
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            if cv2 is not None:
                cv2.putText(test_image, "VLM Connection Test", (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # 测试检测
            result = self.detect_obstacles(test_image)
            return result is not None
            
        except Exception as e:
            print(f"VLM连接测试失败: {e}")
            return False 