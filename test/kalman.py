import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import random
from math import sin, cos, radians

class AngleKalmanFilter:
    """
    用于角度跟踪的卡尔曼滤波器
    状态向量: [角度, 角速度]
    """
    
    def __init__(self, initial_angle=0, initial_velocity=0, 
                 process_noise=0.01, measurement_noise=0.1, dt=0.1):
        """
        初始化角度卡尔曼滤波器
        
        参数:
        initial_angle -- 初始角度估计 (度)
        initial_velocity -- 初始角速度估计 (度/秒)
        process_noise -- 过程噪声方差
        measurement_noise -- 测量噪声方差
        dt -- 采样时间间隔 (秒)
        """
        # 状态向量: [角度, 角速度]
        self.x = np.array([[initial_angle], 
                          [initial_velocity]])
        
        # 状态协方差矩阵
        self.P = np.eye(2) * 1.0
        
        # 过程噪声协方差矩阵
        self.Q = np.eye(2) * process_noise
        
        # 测量噪声方差
        self.R = measurement_noise
        
        # 状态转移矩阵
        self.F = np.array([[1, dt],
                           [0, 1]])
        
        # 观测矩阵 (我们只观测角度)
        self.H = np.array([[1, 0]])
        
        # 当前时间步
        self.timestamp = time.time()
        
        # 历史数据存储
        self.angles = []
        self.velocities = []
        self.measurements = []
        self.estimates = []
        self.timestamps = []
        self.covariances = []
        self.gains = []

    def predict(self):
        """预测步骤：根据系统模型预测下一状态"""
        # 更新状态: x = F * x
        self.x = self.F @ self.x
        
        # 更新协方差: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        # 记录时间戳
        current_time = time.time()
        self.timestamps.append(current_time)
        self.timestamp = current_time
        
        # 返回预测的角度和角速度
        return self.x[0, 0], self.x[1, 0]

    def update(self, measurement):
        """更新步骤：用新的测量值修正预测"""
        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T / S
        
        # 更新状态估计
        y = measurement - self.H @ self.x
        self.x = self.x + K * y
        
        # 更新协方差估计
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P
        
        # 保存当前状态
        self.angles.append(self.x[0, 0])
        self.velocities.append(self.x[1, 0])
        self.measurements.append(measurement)
        self.estimates.append(self.x[0, 0])
        self.covariances.append(self.P[0, 0])  # 角度方差
        self.gains.append(K[0, 0])  # 角度卡尔曼增益
        
        # 返回更新后的角度和角速度
        return self.x[0, 0], self.x[1, 0]

    def get_elapsed_time(self):
        """获取从第一个数据点到现在的时间"""
        if not self.timestamps:
            return 0
        return time.time() - self.timestamps[0]

# 模拟角度传感器
class AngleSensorSimulator:
    """模拟角度传感器，生成带噪声的角度测量值"""
    
    def __init__(self, base_angle=0, amplitude=30, frequency=0.2, noise_level=5):
        """
        初始化角度传感器模拟器
        
        参数:
        base_angle -- 基础角度 (度)
        amplitude -- 角度变化幅度 (度)
        frequency -- 角度变化频率 (Hz)
        noise_level -- 测量噪声水平 (度)
        """
        self.base_angle = base_angle
        self.amplitude = amplitude
        self.frequency = frequency
        self.noise_level = noise_level
        self.start_time = time.time()
        
    def read_angle(self):
        """读取当前角度测量值"""
        # 计算经过的时间 (秒)
        elapsed = time.time() - self.start_time
        
        # 计算真实角度 (正弦变化)
        true_angle = self.base_angle + self.amplitude * sin(2 * np.pi * self.frequency * elapsed)
        
        # 添加噪声
        noise = random.gauss(0, self.noise_level)
        measured_angle = true_angle + noise
        
        # 确保角度在0-360度范围内
        measured_angle %= 360
        
        return measured_angle, true_angle

# 创建传感器和滤波器
sensor = AngleSensorSimulator(base_angle=45, amplitude=40, frequency=0.3, noise_level=8)
kf = AngleKalmanFilter(initial_angle=45, initial_velocity=0, 
                      process_noise=0.1, measurement_noise=25, dt=0.1)

# 设置实时绘图
plt.figure(figsize=(14, 10))
plt.suptitle('实时角度跟踪 - 卡尔曼滤波', fontsize=16)

# 创建角度跟踪子图
ax1 = plt.subplot(3, 1, 1)
plt.title('角度跟踪')
plt.xlabel('时间 (秒)')
plt.ylabel('角度 (度)')
plt.grid(True)
plt.ylim(0, 360)

# 创建角速度跟踪子图
ax2 = plt.subplot(3, 1, 2)
plt.title('角速度跟踪')
plt.xlabel('时间 (秒)')
plt.ylabel('角速度 (度/秒)')
plt.grid(True)
plt.ylim(-100, 100)

# 创建协方差和增益子图
ax3 = plt.subplot(3, 1, 3)
plt.title('滤波器参数')
plt.xlabel('时间 (秒)')
plt.grid(True)

# 初始化数据容器
time_data = []
true_angles, meas_angles, est_angles = [], [], []
est_velocities = []
cov_data, gain_data = [], []

# 创建绘图对象
true_line, = ax1.plot([], [], 'g-', label='真实角度', linewidth=2)
meas_line, = ax1.plot([], [], 'ro', label='测量值', markersize=4, alpha=0.5)
est_line, = ax1.plot([], [], 'b-', label='卡尔曼估计', linewidth=2)
vel_line, = ax2.plot([], [], 'm-', label='估计角速度', linewidth=2)
cov_line, = ax3.plot([], [], 'r-', label='角度协方差')
gain_line, = ax3.plot([], [], 'b-', label='卡尔曼增益')

# 添加图例
ax1.legend(loc='upper right')
ax2.legend(loc='upper right')
ax3.legend(loc='upper right')

# 添加文本信息
info_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, 
                    bbox=dict(facecolor='white', alpha=0.8))

# 动画更新函数
def update(frame):
    # 从传感器读取角度
    measured_angle, true_angle = sensor.read_angle()
    
    # 卡尔曼滤波
    kf.predict()
    est_angle, est_velocity = kf.update(measured_angle)
    
    # 获取经过的时间
    elapsed = kf.get_elapsed_time()
    
    # 更新数据
    time_data.append(elapsed)
    true_angles.append(true_angle)
    meas_angles.append(measured_angle)
    est_angles.append(est_angle)
    est_velocities.append(est_velocity)
    cov_data.append(kf.covariances[-1])
    gain_data.append(kf.gains[-1])
    
    # 更新角度图
    true_line.set_data(time_data, true_angles)
    meas_line.set_data(time_data, meas_angles)
    est_line.set_data(time_data, est_angles)
    
    # 更新角速度图
    vel_line.set_data(time_data, est_velocities)
    
    # 更新滤波器参数图
    cov_line.set_data(time_data, cov_data)
    gain_line.set_data(time_data, gain_data)
    
    # 更新文本信息
    info_text.set_text(f'时间: {elapsed:.1f}秒\n'
                      f'真实角度: {true_angle:.1f}°\n'
                      f'测量角度: {measured_angle:.1f}°\n'
                      f'估计角度: {est_angle:.1f}°\n'
                      f'估计角速度: {est_velocity:.1f}°/s\n'
                      f'卡尔曼增益: {kf.gains[-1]:.4f}\n'
                      f'角度协方差: {kf.covariances[-1]:.4f}')
    
    # 动态调整坐标轴范围
    if elapsed > 5:
        ax1.set_xlim(elapsed-5, elapsed+1)
        ax2.set_xlim(elapsed-5, elapsed+1)
        ax3.set_xlim(elapsed-5, elapsed+1)
    
    # 自动调整y轴范围
    if len(est_velocities) > 1:
        vel_min = min(est_velocities[-20:])
        vel_max = max(est_velocities[-20:])
        ax2.set_ylim(vel_min-10, vel_max+10)
    
    return true_line, meas_line, est_line, vel_line, cov_line, gain_line, info_text

# 创建动画
ani = FuncAnimation(plt.gcf(), update, interval=100, blit=True)

plt.tight_layout()
plt.subplots_adjust(top=0.92)
plt.show()