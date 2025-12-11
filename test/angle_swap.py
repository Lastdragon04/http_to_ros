import math

class AngleUnwrapper:
    def __init__(self):
        self.prev_angle = None      # 存储前一个原始角度
        self.offset = 0            # 累积的偏移量（2π的整数倍）
        self.first_value = True    # 标记是否为第一个数据点

    def unwrap(self, angle):
        if self.first_value:
            # 第一个数据点，直接返回并初始化状态
            self.prev_angle = angle
            self.first_value = False
            return angle
        
        # 计算与前一个角度的差值
        delta = angle - self.prev_angle
        
        # 调整差值到 [-π, π) 区间
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi
        
        # 更新当前解包后的角度（累加实际变化量）
        current_unwrapped = self.prev_angle + self.offset + delta
        
        # 更新状态
        self.prev_angle = angle
        self.offset = current_unwrapped - angle  # 确保下次计算使用正确的偏移
        
        return current_unwrapped


# ===== 使用示例 =====
unwrapper = AngleUnwrapper()

# 模拟IMU数据序列（单位：弧度）
# 注意：当角度 <-π 时，IMU原始数据会跳变到 +π 附近
imu_angles = [-3.14, 3.13, 3.12, 3.11, -3.15, -3.1664656,1.2,0.2,-3.1123]
imu_angles = [ 3.13, 3.12, 3.11, -3.15, -3.1664656,1.2,0.2,-3.1123]
print("原始角度\t解包后角度")
for angle in imu_angles:
    unwrapped = unwrapper.unwrap(angle)
    print(f"{angle:.5f}\t{unwrapped:.5f}")