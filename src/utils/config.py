import os

class Config:
    """配置参数类"""
    
    # VLM API配置（只从环境变量读取，禁止硬编码密钥）
    # 同时兼容带空格与下划线的环境变量名
    OPENAI_API_KEY = (
        os.environ.get("OPENAI_API_KEY")
        or os.environ.get("OPENAI API KEY")
        or ""
    )
    OPENAI_BASE_URL = (
        os.environ.get("OPENAI_BASE_URL")
        or os.environ.get("OPENAI BASE URL")
        or "https://dashscope.aliyuncs.com/compatible-mode/v1"
    )
    VLM_MODEL = "qwen-vl-max-2025-08-13"
    
    # A*全局路径规划参数
    ASTAR_GRID_SIZE = 1.0  # 网格大小(米)
    ASTAR_HEURISTIC_WEIGHT = 1.0  # 启发式权重
    ASTAR_OBSTACLE_COST = 1000  # 障碍物代价
    
    # DWA局部路径规划参数
    DWA_MAX_SPEED = 2.0  # 最大线速度 (m/s)
    DWA_MIN_SPEED = 0.0  # 最小线速度 (m/s)
    DWA_MAX_ANGULAR_SPEED = 1.0  # 最大角速度 (rad/s)
    DWA_MIN_ANGULAR_SPEED = -1.0  # 最小角速度 (rad/s)
    DWA_MAX_ACCEL = 2.0  # 最大线加速度 (m/s²)
    DWA_MAX_ANGULAR_ACCEL = 2.0  # 最大角加速度 (rad/s²)
    DWA_VELOCITY_RESOLUTION = 0.05  # 速度分辨率 (减小以提高精度)
    DWA_ANGULAR_RESOLUTION = 0.05  # 角速度分辨率(更细)
    DWA_DT = 0.1  # 时间步长
    DWA_PREDICT_TIME = 3.0  # 预测时间 (更早预见潜在碰撞)
    DWA_TO_GOAL_COST_GAIN = 3.0  # 目标代价权重
    DWA_SPEED_COST_GAIN = 0.1  # 速度代价权重 (减少以允许更快的移动)
    DWA_OBSTACLE_COST_GAIN = 8.0  # 障碍物代价权重（提高以拉开安全距离）
    DWA_PATH_COST_GAIN = 1.2  # 贴合全局路径代价权重（适度降低，允许为避障偏离路径）
    DWA_ROBOT_RADIUS = 0.5  # 机器人半径
    # 额外安全余量(米)，用于地图膨胀与DWA清距
    SAFETY_MARGIN = 0.6

    # VLM障碍物相关配置
    VLM_CAMERA_RANGE = 12.0  # 摄像头有效探测距离(米)
    VLM_OBS_INFLATION_FACTOR = 1.6  # 对VLM识别障碍的半径乘性膨胀（更保守）
    VLM_OBS_EXTRA_MARGIN = 0.4  # 对VLM识别障碍的附加米级膨胀
    
    # 仿真参数
    SIM_STEP_TIME = 0.1  # 仿真时间步长
    SIM_MAP_WIDTH = 50  # 地图宽度
    SIM_MAP_HEIGHT = 50  # 地图高度
    SIM_GOAL_TOLERANCE = 1.0  # 目标容忍距离
    
    # PID控制器参数
    PID_KP_LINEAR = 1.0  # 线速度比例增益
    PID_KI_LINEAR = 0.1  # 线速度积分增益
    PID_KD_LINEAR = 0.05  # 线速度微分增益
    PID_KP_ANGULAR = 2.0  # 角速度比例增益
    PID_KI_ANGULAR = 0.1  # 角速度积分增益
    PID_KD_ANGULAR = 0.1  # 角速度微分增益
    
    # 仿真参数
    SIM_STEP_TIME = 0.1  # 仿真步长
    SIM_MAP_WIDTH = 50  # 地图宽度
    SIM_MAP_HEIGHT = 50  # 地图高度
    SIM_OBSTACLE_RADIUS = 1.0  # 动态障碍物半径
    
    # 性能测试参数
    TEST_ITERATIONS = 10  # 测试迭代次数
    SAVE_RESULTS = True  # 是否保存结果 