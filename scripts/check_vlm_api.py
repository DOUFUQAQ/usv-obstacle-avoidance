#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试新的API密钥
"""

import os
import requests
import json
import base64
from PIL import Image
import numpy as np

# 从环境变量读取 API 密钥：export OPENAI_API_KEY=xxx
if not os.environ.get("OPENAI_API_KEY"):
    raise SystemExit("请先设置环境变量 OPENAI_API_KEY")

def test_vlm_api():
    """测试VLM API连接"""
    
    # API配置
    api_key = os.environ["OPENAI_API_KEY"]
    base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1"
    model = "qwen-vl-max"
    
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }
    
    # 创建一个简单的测试图像
    test_image = np.ones((100, 100, 3), dtype=np.uint8) * 128  # 灰色图像
    test_image = Image.fromarray(test_image)
    
    # 编码图像
    import io
    buffer = io.BytesIO()
    test_image.save(buffer, format='PNG')
    image_data = base64.b64encode(buffer.getvalue()).decode('utf-8')
    
    # 构建请求
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "请分析这张USV摄像头图像，检测障碍物和安全区域。请只返回JSON格式的结果，不要其他解释。"
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/png;base64,{image_data}"
                        }
                    }
                ]
            }
        ],
        "max_tokens": 1000
    }
    
    try:
        print("正在测试新的API密钥...")
        response = requests.post(
            f"{base_url}/chat/completions",
            headers=headers,
            json=payload,
            timeout=30
        )
        
        print(f"状态码: {response.status_code}")
        
        if response.status_code == 200:
            result = response.json()
            print("✓ API连接成功!")
            print(f"响应内容: {result['choices'][0]['message']['content'][:200]}...")
            return True
        else:
            print(f"✗ API连接失败: {response.status_code}")
            print(f"错误信息: {response.text}")
            return False
            
    except Exception as e:
        print(f"✗ 连接异常: {e}")
        return False

if __name__ == "__main__":
    test_vlm_api() 