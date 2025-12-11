#!/usr/bin/env python3
import subprocess
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

# -------------------------- 原arm.py代码（完全保留，未改动） --------------------------
class ArmControl(Node):
    """机器人手臂控制器，按指定流程执行动作（与头部控制脚本消息类型保持一致）"""
    
    def __init__(self):
        super().__init__('arm_control')
        self.get_logger().info("初始化手臂控制器...")
        
        # 运行状态标记
        self.running = True  
        
        # 核心配置参数（所有数值型参数严格遵循float类型规范）
        self.cfg = {
            "joints_left": [11, 12, 13, 14],  # 左臂关节1-4
            "joints_right": [21, 22, 23, 24], # 右臂关节1-4
            "move_sequence": [2, 1, 3, 4],    # 运动顺序(关节序号)
            "safe_sequence": [4, 3, 1, 2],    # 安全回归顺序(关节序号)
            "cycles": 2,                      # 循环次数（整数不影响消息字段）
            "speed": 0.5,                     # 运动速度（float类型）
            "current": 5.0,                  # 运动电流（修复为float类型）
            "step_delay": 0.5,                # 步骤间隔(秒)（float类型）
            "action_delay": 0.5,              # 动作执行等待时间(秒)（float类型）
            
            # 运动位置参数(关节号:位置)（均为float类型）
            "move_positions": {
                12: 0.2,
                11: -0.5,
                13: -0.4,
                14: -0.4,
                22: -0.2,
                21: -0.5,
                23: 0.4,
                24: -0.4
            },
            
            # 安全位置参数(关节号:位置)（均为float类型）
            "safe_positions": {
                11: -0.1,
                12: 0.1,
                13: -0.1,
                14: -0.1,
                21: -0.1,
                22: -0.1,
                23: 0.1,
                24: -0.1
            }
        }

        try:
            # 创建发布者(控制手臂，话题路径严格保持要求)
            self.pub = self.create_publisher(
                CmdSetMotorPosition, '/arm/cmd_pos', 10)
            
            # 等待控制器连接
            self._wait_for_controller()
            
            # 执行主流程
            self.run()
            
        except Exception as e:
            self.get_logger().error(f"初始化失败: {e}")
            self._emergency_stop()

    # 核心工具方法（确保消息字段类型正确）
    def _create_msg(self, joint_id, target_pos):
        """创建关节控制消息，严格匹配SetMotorPosition类型要求"""
        # 验证关节ID有效性
        if not ((11 <= joint_id <= 14) or (21 <= joint_id <= 24)):
            self.get_logger().error(f"无效关节ID: {joint_id}")
            return None
        
        # 强制转换位置为float类型（双重保障）
        try:
            target_pos = float(target_pos)
        except ValueError:
            self.get_logger().error(f"位置参数无法转换为float: {target_pos}")
            return None
        
        # 构建电机控制指令（所有字段均为正确类型）
        motor_cmd = SetMotorPosition()
        motor_cmd.name = joint_id                  # 关节ID（整数，符合name字段要求）
        motor_cmd.pos = target_pos                 # 位置（float类型）
        motor_cmd.spd = self.cfg["speed"]          # 速度（float类型）
        motor_cmd.cur = self.cfg["current"]        # 电流（float类型，已修复）
        
        # 构建指令集合消息
        msg = CmdSetMotorPosition()
        msg.cmds = [motor_cmd]                     # cmds字段为SetMotorPosition对象列表
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm'
        return msg

    def _publish(self, msg):
        """发布消息到手臂控制器"""
        if not msg or not self.running:
            return False
        try:
            self.pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"发布失败: {e}")
            return False

    def _control_joint(self, joint_id, target_pos, desc):
        """控制单个关节"""
        self.get_logger().info(f"{desc} (目标: {target_pos})")
        msg = self._create_msg(joint_id, target_pos)
        if not self._publish(msg):
            raise RuntimeError(f"{desc}失败")
        time.sleep(self.cfg["action_delay"])

    # 初始化与安全控制
    def _wait_for_controller(self, timeout=10):
        """等待控制器连接"""
        start = time.time()
        while self.running and self.pub.get_subscription_count() == 0:
            if time.time() - start > timeout:
                raise TimeoutError("控制器连接超时")
            time.sleep(1)
        self.get_logger().info("手臂控制器已连接")

    # 主流程步骤
    def step1_left_arm_move(self):
        """步骤1: 左臂按2→1→3→4关节顺序运动"""
        for seq in self.cfg["move_sequence"]:
            joint_id = self.cfg["joints_left"][seq - 1]  # 转换为实际关节号
            self._control_joint(
                joint_id,
                self.cfg["move_positions"][joint_id],
                f"步骤1: 左臂关节{joint_id}运动"
            )

    def step2_right_arm_move(self):
        """步骤2: 右臂按2→1→3→4关节顺序运动"""
        for seq in self.cfg["move_sequence"]:
            joint_id = self.cfg["joints_right"][seq - 1]  # 转换为实际关节号
            self._control_joint(
                joint_id,
                self.cfg["move_positions"][joint_id],
                f"步骤2: 右臂关节{joint_id}运动"
            )

    def step3_left_arm_safe(self):
        """步骤3: 左臂按4→3→1→2关节顺序回归安全位"""
        for seq in self.cfg["safe_sequence"]:
            joint_id = self.cfg["joints_left"][seq - 1]  # 转换为实际关节号
            self._control_joint(
                joint_id,
                self.cfg["safe_positions"][joint_id],
                f"步骤3: 左臂关节{joint_id}回归安全位"
            )

    def step4_right_arm_safe(self):
        """步骤4: 右臂按4→3→1→2关节顺序回归安全位"""
        for seq in self.cfg["safe_sequence"]:
            joint_id = self.cfg["joints_right"][seq - 1]  # 转换为实际关节号
            self._control_joint(
                joint_id,
                self.cfg["safe_positions"][joint_id],
                f"步骤4: 右臂关节{joint_id}回归安全位"
            )

    # 主循环与退出处理
    def run(self):
        """执行主循环(2次流程)"""
        self.get_logger().info(f"开始循环({self.cfg['cycles']}次)...")
        try:
            for cycle in range(1, self.cfg["cycles"] + 1):
                if not self.running:
                    break
                self.get_logger().info(f"\n===== 第{cycle}次循环 =====")
                
                self.step1_left_arm_move()
                time.sleep(self.cfg["step_delay"])
                
                self.step2_right_arm_move()
                time.sleep(self.cfg["step_delay"])
                
                self.step3_left_arm_safe()
                time.sleep(self.cfg["step_delay"])
                
                self.step4_right_arm_safe()
                time.sleep(self.cfg["step_delay"])

            self.get_logger().info("循环完成")
            self._normal_exit()
            
        except Exception as e:
            self.get_logger().error(f"流程出错: {e}")
            self._emergency_stop()

    def _emergency_stop(self):
        """紧急停止: 强制回归安全位置"""
        self.get_logger().info("执行紧急停止...")
        self.running = False
        try:
            # 左臂紧急回归安全位
            for seq in self.cfg["safe_sequence"]:
                joint_id = self.cfg["joints_left"][seq - 1]
                self._control_joint(joint_id, self.cfg["safe_positions"][joint_id], f"紧急复位左臂关节{joint_id}")
            
            # 右臂紧急回归安全位
            for seq in self.cfg["safe_sequence"]:
                joint_id = self.cfg["joints_right"][seq - 1]
                self._control_joint(joint_id, self.cfg["safe_positions"][joint_id], f"紧急复位右臂关节{joint_id}")
        except:
            pass
        sys.exit(1)

    def _normal_exit(self):
        """正常退出"""
        self.running = False
        self.get_logger().info("程序正常退出")
        sys.exit(0)


# 信号处理与主入口（原arm.py代码结束）
def arm_signal_handler(signal_num, frame):
    """处理中断信号(Ctrl+C)"""
    if 'node' in globals():
        node.get_logger().info(f"\n收到中断信号，准备退出...")
        node.running = False
    else:
        print("收到中断信号，退出")
        sys.exit(1)

def run_arm_control():
    """执行arm控制逻辑（原arm.py的main函数功能）"""
    signal.signal(signal.SIGINT, arm_signal_handler)
    signal.signal(signal.SIGTERM, arm_signal_handler)
    
    global node
    try:
        rclpy.init()
        node = ArmControl()
        rclpy.spin_once(node)
    except Exception as e:
        print(f"启动失败: {e}")
        sys.exit(1)
    finally:
        if 'node' in globals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# -------------------------- 系统命令执行逻辑（修复退出问题） --------------------------
def run_system_command(cmd, sudo=False, capture_output=False):
    """执行系统命令（含空值判断）"""
    try:
        args = []
        if sudo:
            args = ['sudo', '-S'] + cmd
            result = subprocess.run(
                args,
                input='123\n'.encode(),  # 自动输入密码
                stdout=subprocess.PIPE if capture_output else None,
                stderr=subprocess.PIPE if capture_output else None
            )
        else:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE if capture_output else None,
                stderr=subprocess.PIPE if capture_output else None
            )
        
        if result.returncode != 0:
            print(f"命令执行失败: {' '.join(cmd)}")
            err_msg = result.stderr.decode() if result.stderr else "无详细错误信息"
            print(f"错误信息: {err_msg}")
            sys.exit(1)
        return result
    except Exception as e:
        print(f"执行命令时出错: {e}")
        sys.exit(1)


# -------------------------- 脚本入口 --------------------------
if __name__ == '__main__':
    def global_signal_handler(sig, frame):
        # 强制退出时清理临时进程
        print("\n收到强制中断信号，清理资源...")
        # 终止可能残留的tail进程
        subprocess.run(['sudo', '-S', 'pkill', '-f', 'tail -f /home/ubuntu/arm.log'], 
                      input='123\n'.encode(), stderr=subprocess.DEVNULL)
        print("已清理日志监听进程，脚本退出")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, global_signal_handler)
