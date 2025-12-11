import random
import math
import numpy as np
import matplotlib.pyplot as plt

n = 500
real = []  # 真值
mear = []  # 观测值
pred_realtime = []  # 实时滤波值

# 建立真值和观测值
for i in range(n):
    num = 0.003 * i 
    real.append(num)
    num += 0.1 * np.random.standard_normal()  # 本身的不确定性
    num += 0.5 * np.random.standard_normal()  # 观测的不确定性
    mear.append(num)

# 实时均值滤波类
class RealtimeMovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.window = []
        self.sum = 0.0
        self.count = 0
    
    def update(self, new_value):
        # 添加新值
        self.window.append(new_value)
        self.sum += new_value
        self.count += 1
        
        # 如果窗口已满，移除最旧的值
        if len(self.window) > self.window_size:
            removed = self.window.pop(0)
            self.sum -= removed
        
        # 计算平均值
        if self.count < self.window_size:
            # 窗口未满时，使用当前所有数据的平均值
            return self.sum / len(self.window)
        else:
            # 窗口已满，使用固定窗口大小的平均值
            return self.sum / self.window_size

# 使用窗口大小为11的实时滤波器
window_size = 11
realtime_filter = RealtimeMovingAverage(window_size)

# 实时处理每个观测值
for value in mear:
    filtered_value = realtime_filter.update(value)
    pred_realtime.append(filtered_value)

# 绘制结果
plt.figure(figsize=(12, 8))

# 原始观测值和真值
plt.plot(range(n), mear, 'c-', alpha=0.3, label='观测值')
plt.plot(range(n), real, 'k-', linewidth=2, label='真值')

# 实时滤波结果
plt.plot(range(n), pred_realtime, 'r-', linewidth=1.5, label=f'实时滤波 (窗口={window_size})')

# 添加图例和标题
plt.legend()
plt.title('实时均值滤波效果')
plt.xlabel('时间步')
plt.ylabel('值')
plt.grid(True, alpha=0.3)

# 显示前100个点的细节图
plt.figure(figsize=(12, 4))
plt.plot(range(100), mear[:100], 'c-', alpha=0.3, label='观测值')
plt.plot(range(100), real[:100], 'k-', linewidth=2, label='真值')
plt.plot(range(100), pred_realtime[:100], 'r-', linewidth=1.5, label='实时滤波')
plt.title('前100个点的细节')
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()