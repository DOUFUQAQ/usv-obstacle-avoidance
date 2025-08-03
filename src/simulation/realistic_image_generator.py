#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
真实的USV摄像头图像生成器
生成VRX风格的海洋环境图像，包含真实的障碍物和海洋特征
"""

import numpy as np
import cv2
import math
import random
from typing import Tuple, List, Dict, Any
from PIL import Image, ImageDraw, ImageFont
import os

class RealisticMarineImageGenerator:
    """真实海洋环境图像生成器"""
    
    def __init__(self, image_size: Tuple[int, int] = (1024, 768)):
        self.width, self.height = image_size
        self.sky_height = int(self.height * 0.3)  # 天空占30%
        self.water_height = self.height - self.sky_height
        
        # 海洋颜色配置
        self.sky_colors = [
            (135, 206, 235),  # 天蓝色
            (173, 216, 230),  # 浅蓝色
            (240, 248, 255),  # 爱丽丝蓝
        ]
        
        self.water_colors = [
            (25, 25, 112),   # 深海蓝
            (0, 105, 148),   # 深青色
            (4, 139, 154),   # 暗青色
            (0, 119, 190),   # 蓝色
        ]
        
        # 障碍物模板
        self.obstacle_templates = {
            'ship': {'color': (80, 80, 80), 'height_ratio': 0.4},
            'buoy': {'color': (255, 165, 0), 'height_ratio': 0.2},
            'rock': {'color': (105, 105, 105), 'height_ratio': 0.3},
            'pier': {'color': (139, 69, 19), 'height_ratio': 0.5},
            'island': {'color': (34, 139, 34), 'height_ratio': 0.6},
        }
    
    def generate_realistic_marine_image(self, usv_pose: Tuple[float, float, float],
                                      obstacles: List[Dict[str, Any]],
                                      weather: str = 'clear',
                                      time_of_day: str = 'day') -> np.ndarray:
        """
        生成真实的海洋环境图像
        
        Args:
            usv_pose: USV位姿 (x, y, yaw)
            obstacles: 障碍物列表
            weather: 天气条件 ('clear', 'cloudy', 'foggy', 'rainy')
            time_of_day: 时间 ('day', 'dawn', 'dusk', 'night')
        
        Returns:
            生成的图像
        """
        # 创建基础图像
        image = self._create_base_marine_scene(weather, time_of_day)
        
        # 添加海面波浪
        image = self._add_ocean_waves(image, weather)
        
        # 添加障碍物
        image = self._add_obstacles(image, usv_pose, obstacles)
        
        # 添加环境细节
        image = self._add_environmental_details(image, weather, time_of_day)
        
        # 添加相机效果
        image = self._apply_camera_effects(image, weather)
        
        return image
    
    def _create_base_marine_scene(self, weather: str, time_of_day: str) -> np.ndarray:
        """创建基础海洋场景"""
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # 绘制天空
        sky_color = self._get_sky_color(weather, time_of_day)
        image[:self.sky_height, :] = sky_color
        
        # 绘制地平线
        horizon_y = self.sky_height
        cv2.line(image, (0, horizon_y), (self.width, horizon_y), (200, 200, 200), 2)
        
        # 绘制海面基础颜色
        water_color = self._get_water_color(weather, time_of_day)
        image[self.sky_height:, :] = water_color
        
        return image
    
    def _get_sky_color(self, weather: str, time_of_day: str) -> Tuple[int, int, int]:
        """获取天空颜色"""
        base_color = self.sky_colors[0]
        
        if time_of_day == 'dawn':
            base_color = (255, 182, 193)  # 粉红色
        elif time_of_day == 'dusk':
            base_color = (255, 140, 0)    # 橙色
        elif time_of_day == 'night':
            base_color = (25, 25, 112)    # 深蓝色
        
        if weather == 'cloudy':
            # 添加灰色调
            base_color = tuple(int(c * 0.8) for c in base_color)
        elif weather == 'foggy':
            # 添加白色调
            base_color = tuple(min(255, int(c * 0.9 + 50)) for c in base_color)
        
        return base_color
    
    def _get_water_color(self, weather: str, time_of_day: str) -> Tuple[int, int, int]:
        """获取水面颜色"""
        base_color = self.water_colors[1]
        
        if time_of_day == 'dawn' or time_of_day == 'dusk':
            # 反射天空颜色
            base_color = tuple(min(255, int(c * 1.2)) for c in base_color)
        elif time_of_day == 'night':
            base_color = self.water_colors[0]  # 更深的蓝色
        
        if weather == 'foggy':
            # 雾化效果
            base_color = tuple(min(255, int(c * 0.8 + 40)) for c in base_color)
        
        return base_color
    
    def _add_ocean_waves(self, image: np.ndarray, weather: str) -> np.ndarray:
        """添加海浪和水面纹理"""
        wave_intensity = {'clear': 0.3, 'cloudy': 0.5, 'foggy': 0.2, 'rainy': 0.8}
        intensity = wave_intensity.get(weather, 0.3)
        
        # 添加波浪纹理
        for y in range(self.sky_height, self.height, 3):
            wave_offset = int(10 * intensity * math.sin(y * 0.1))
            noise = np.random.randint(-5, 6, self.width)
            
            for x in range(self.width):
                if y + wave_offset < self.height:
                    # 添加波浪高光
                    if random.random() < intensity * 0.1:
                        brightness = min(255, int(image[y, x, 0] + 40))
                        image[y, x] = (brightness, brightness, min(255, brightness + 20))
                    
                    # 添加噪声
                    for c in range(3):
                        image[y, x, c] = np.clip(image[y, x, c] + noise[x], 0, 255)
        
        return image
    
    def _add_obstacles(self, image: np.ndarray, usv_pose: Tuple[float, float, float],
                      obstacles: List[Dict[str, Any]]) -> np.ndarray:
        """添加障碍物到图像中"""
        usv_x, usv_y, usv_yaw = usv_pose
        
        # 摄像头参数
        fov = math.pi / 2.5  # 72度视野
        view_range = 50.0    # 50米视野范围
        
        for i, obs in enumerate(obstacles):
            # 计算障碍物相对位置
            dx = obs['x'] - usv_x
            dy = obs['y'] - usv_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > view_range:
                continue
            
            # 转换到摄像头坐标系
            local_x = dx * math.cos(-usv_yaw) - dy * math.sin(-usv_yaw)
            local_y = dx * math.sin(-usv_yaw) + dy * math.cos(-usv_yaw)
            
            # 检查是否在视野内
            if local_x <= 0:
                continue
                
            angle = math.atan2(local_y, local_x)
            if abs(angle) > fov / 2:
                continue
            
            # 投影到图像平面
            img_x = int(self.width / 2 + (angle / (fov / 2)) * self.width / 2)
            
            # 根据距离计算大小和位置
            scale = min(1.0, 10.0 / distance)
            obj_width = int(obs['radius'] * scale * 80)
            obj_height = int(obj_width * self.obstacle_templates['ship']['height_ratio'])
            
            # 计算在水面的位置
            water_line = int(self.sky_height + (1.0 - scale) * self.water_height * 0.3)
            img_y = water_line - obj_height // 2
            
            # 选择障碍物类型
            obstacle_type = self._determine_obstacle_type(obs, i)
            
            # 绘制障碍物
            self._draw_obstacle(image, img_x, img_y, obj_width, obj_height, 
                              obstacle_type, distance, scale)
        
        return image
    
    def _determine_obstacle_type(self, obs: Dict[str, Any], index: int) -> str:
        """确定障碍物类型"""
        # 根据大小和位置确定类型
        radius = obs['radius']
        
        if radius > 3.0:
            return 'ship'
        elif radius > 2.0:
            return 'island' if index % 4 == 0 else 'pier'
        elif radius > 1.0:
            return 'rock' if index % 3 == 0 else 'buoy'
        else:
            return 'buoy'
    
    def _draw_obstacle(self, image: np.ndarray, x: int, y: int, width: int, height: int,
                      obstacle_type: str, distance: float, scale: float):
        """绘制具体的障碍物"""
        template = self.obstacle_templates[obstacle_type]
        base_color = template['color']
        
        # 根据距离调整颜色（距离越远越模糊）
        fog_factor = min(1.0, distance / 30.0)
        color = tuple(int(c * (1 - fog_factor * 0.5)) for c in base_color)
        
        x1, y1 = max(0, x - width//2), max(0, y - height//2)
        x2, y2 = min(self.width, x + width//2), min(self.height, y + height//2)
        
        if obstacle_type == 'ship':
            # 绘制船体
            cv2.rectangle(image, (x1, y1), (x2, y2), color, -1)
            # 绘制船桅
            mast_height = int(height * 0.5)
            cv2.line(image, (x, y1), (x, max(0, y1 - mast_height)), 
                    (60, 60, 60), max(1, int(scale * 3)))
            
        elif obstacle_type == 'buoy':
            # 绘制浮标（圆形）
            cv2.circle(image, (x, y), width//2, color, -1)
            # 添加反光条纹
            cv2.circle(image, (x, y), width//2, (255, 255, 255), max(1, int(scale * 2)))
            
        elif obstacle_type == 'rock':
            # 绘制岩石（不规则形状）
            points = []
            for i in range(6):
                angle = i * math.pi / 3
                r = width//2 + random.randint(-width//4, width//4)
                px = x + int(r * math.cos(angle))
                py = y + int(r * math.sin(angle))
                points.append([px, py])
            cv2.fillPoly(image, [np.array(points)], color)
            
        elif obstacle_type == 'pier':
            # 绘制码头（矩形结构）
            cv2.rectangle(image, (x1, y1), (x2, y2), color, -1)
            # 添加支柱
            for i in range(3):
                pillar_x = x1 + (i + 1) * width // 4
                cv2.line(image, (pillar_x, y2), (pillar_x, min(self.height, y2 + height//2)),
                        (100, 100, 100), max(1, int(scale * 2)))
        
        elif obstacle_type == 'island':
            # 绘制岛屿（椭圆形）
            cv2.ellipse(image, (x, y), (width//2, height//2), 0, 0, 360, color, -1)
            # 添加植被
            tree_color = (0, 100, 0)
            for i in range(max(1, int(scale * 3))):
                tree_x = x + random.randint(-width//3, width//3)
                tree_y = y + random.randint(-height//4, height//4)
                cv2.circle(image, (tree_x, tree_y), max(1, int(scale * 5)), tree_color, -1)
        
        # 添加水面反射
        if y2 < self.height - 10:
            reflection_height = min(height//3, self.height - y2 - 5)
            reflection_color = tuple(int(c * 0.3) for c in color)
            cv2.rectangle(image, (x1, y2 + 5), (x2, y2 + 5 + reflection_height), 
                         reflection_color, -1)
    
    def _add_environmental_details(self, image: np.ndarray, weather: str, time_of_day: str) -> np.ndarray:
        """添加环境细节"""
        # 添加云层
        if weather in ['cloudy', 'rainy']:
            self._add_clouds(image)
        
        # 添加太阳/月亮
        if time_of_day != 'night':
            self._add_sun(image, time_of_day)
        else:
            self._add_moon(image)
        
        # 添加海鸟
        if weather == 'clear' and random.random() < 0.3:
            self._add_seabirds(image)
        
        return image
    
    def _add_clouds(self, image: np.ndarray):
        """添加云层"""
        for _ in range(random.randint(3, 8)):
            cloud_x = random.randint(0, self.width)
            cloud_y = random.randint(0, self.sky_height//2)
            cloud_size = random.randint(30, 80)
            
            # 创建云的遮罩
            overlay = image.copy()
            cv2.circle(overlay, (cloud_x, cloud_y), cloud_size, (220, 220, 220), -1)
            cv2.circle(overlay, (cloud_x + 20, cloud_y - 10), cloud_size//2, (240, 240, 240), -1)
            cv2.circle(overlay, (cloud_x - 15, cloud_y + 5), cloud_size//3, (200, 200, 200), -1)
            
            # 混合云层
            alpha = 0.6
            image[:self.sky_height, :] = cv2.addWeighted(
                image[:self.sky_height, :], 1-alpha, 
                overlay[:self.sky_height, :], alpha, 0
            )
    
    def _add_sun(self, image: np.ndarray, time_of_day: str):
        """添加太阳"""
        if time_of_day == 'dawn':
            sun_x, sun_y = self.width // 4, self.sky_height // 3
            sun_color = (255, 223, 0)
        elif time_of_day == 'dusk':
            sun_x, sun_y = 3 * self.width // 4, self.sky_height // 2
            sun_color = (255, 140, 0)
        else:  # day
            sun_x, sun_y = self.width // 2, self.sky_height // 4
            sun_color = (255, 255, 0)
        
        # 绘制太阳
        cv2.circle(image, (sun_x, sun_y), 25, sun_color, -1)
        
        # 添加光晕
        for i in range(3):
            radius = 30 + i * 10
            alpha = 0.1 / (i + 1)
            overlay = image.copy()
            cv2.circle(overlay, (sun_x, sun_y), radius, sun_color, -1)
            image = cv2.addWeighted(image, 1-alpha, overlay, alpha, 0)
    
    def _add_moon(self, image: np.ndarray):
        """添加月亮"""
        moon_x = 3 * self.width // 4
        moon_y = self.sky_height // 3
        cv2.circle(image, (moon_x, moon_y), 20, (240, 240, 240), -1)
    
    def _add_seabirds(self, image: np.ndarray):
        """添加海鸟"""
        for _ in range(random.randint(1, 4)):
            bird_x = random.randint(0, self.width)
            bird_y = random.randint(0, self.sky_height)
            
            # 简单的鸟形状
            cv2.ellipse(image, (bird_x, bird_y), (8, 3), 0, 0, 360, (50, 50, 50), -1)
            cv2.ellipse(image, (bird_x + 6, bird_y), (6, 2), 30, 0, 360, (50, 50, 50), -1)
    
    def _apply_camera_effects(self, image: np.ndarray, weather: str) -> np.ndarray:
        """应用相机效果"""
        # 添加镜头光晕
        if random.random() < 0.2:
            flare_x = random.randint(0, self.width)
            flare_y = random.randint(0, self.height)
            cv2.circle(image, (flare_x, flare_y), 30, (255, 255, 255), -1)
            image = cv2.addWeighted(image, 0.9, image, 0.1, 0)
        
        # 根据天气添加效果
        if weather == 'foggy':
            # 雾化效果
            fog_overlay = np.ones_like(image) * 200
            image = cv2.addWeighted(image, 0.7, fog_overlay, 0.3, 0)
            
        elif weather == 'rainy':
            # 雨滴效果
            for _ in range(random.randint(50, 150)):
                rain_x = random.randint(0, self.width)
                rain_y = random.randint(0, self.height)
                cv2.line(image, (rain_x, rain_y), (rain_x + 2, rain_y + 8), 
                        (200, 200, 255), 1)
        
        # 添加轻微的噪声
        noise = np.random.randint(-5, 6, image.shape, dtype=np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        return image

def create_sample_marine_images():
    """创建示例海洋图像用于VLM测试"""
    generator = RealisticMarineImageGenerator()
    
    # 定义测试场景
    scenarios = [
        {
            'name': 'clear_day_with_ships',
            'obstacles': [
                {'x': 20, 'y': 10, 'radius': 4.0},  # 大船
                {'x': 35, 'y': -5, 'radius': 2.0},  # 小船
                {'x': 15, 'y': 20, 'radius': 1.5},  # 浮标
            ],
            'weather': 'clear',
            'time': 'day'
        },
        {
            'name': 'foggy_with_pier',
            'obstacles': [
                {'x': 25, 'y': 0, 'radius': 6.0},   # 码头
                {'x': 40, 'y': 10, 'radius': 1.0},  # 浮标
                {'x': 18, 'y': -8, 'radius': 3.0},  # 岩石
            ],
            'weather': 'foggy',
            'time': 'day'
        },
        {
            'name': 'storm_with_island',
            'obstacles': [
                {'x': 30, 'y': 5, 'radius': 8.0},   # 岛屿
                {'x': 45, 'y': -3, 'radius': 2.5},  # 船只
                {'x': 12, 'y': 15, 'radius': 1.2},  # 浮标
            ],
            'weather': 'rainy',
            'time': 'dusk'
        }
    ]
    
    # 创建图像目录
    os.makedirs('marine_images', exist_ok=True)
    
    usv_pose = (0, 0, 0)  # USV在原点
    
    for scenario in scenarios:
        image = generator.generate_realistic_marine_image(
            usv_pose, scenario['obstacles'], 
            scenario['weather'], scenario['time']
        )
        
        # 保存图像
        filename = f"marine_images/{scenario['name']}.jpg"
        cv2.imwrite(filename, image)
        print(f"生成图像: {filename}")
    
    return scenarios

if __name__ == "__main__":
    # 生成示例图像
    create_sample_marine_images()
    print("海洋图像生成完成！") 