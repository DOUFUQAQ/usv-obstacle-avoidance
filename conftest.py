"""pytest 入口：把仓库根目录放进 sys.path，并保证测试在无图形环境下运行。

本仓库的测试同时支持两种运行方式，任选其一：

    python -m unittest discover -s tests -t .    # 零依赖（只需要 numpy）
    python -m pytest -q                          # 需要 pytest

测试本身只用标准库 unittest 编写，这样 CI 和本地都不必为了跑测试而装额外依赖。
"""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# 无显示器环境下 matplotlib 必须走 Agg 后端
os.environ.setdefault("MPLBACKEND", "Agg")
# VLM 检测器构造时需要一个密钥占位符，测试本身不会发起网络请求
os.environ.setdefault("OPENAI_API_KEY", "test-key-never-used")
