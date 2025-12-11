import threading
import json
import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node


class RightSideCamera(Node):
    def __init__(self):
        super().__init__('right_side_camera_node')
        self.robot_pose_position = {"right_hip": 0, "right_knee": 0, "right_ankle": 0}
        self.i = 1
        self.out = None
        self.cap = None
        self.annotated_out = None  # 新增：用于保存带注释的视频
        self.video_filename = "/home/zck/workspace/website/Media/right.mp4"  # 带注释的视频文件名

    def run(self):
        mp_pose = mp.solutions.pose
        pose = mp_pose.Pose()
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return

        # 获取原始尺寸并计算放大后的尺寸
        orig_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        orig_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        scaled_width = orig_width * 2
        scaled_height = orig_height * 2
        fps = 30

        # 创建原始视频写入器
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter('/home/zck/workspace/website/Media/raw_right.mp4', fourcc, fps, (scaled_width, scaled_height))
        
        # 创建带注释的视频写入器
        self.annotated_out = cv2.VideoWriter(self.video_filename, fourcc, fps, (scaled_width, scaled_height))

        while self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                self.get_logger().warning("Ignoring empty camera frame.")
                continue
            
            # 调整大小和翻转
            image = cv2.resize(image, (0, 0), fy=2, fx=2)
            image = cv2.flip(image, 1)
            
            # 保存原始图像到视频（注意：OpenCV使用BGR格式）
            self.out.write(image)
            
            # 转换图像从 BGR 到 RGB 供Mediapipe处理
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # 处理图像并识别姿态
            results = pose.process(rgb_image)
            
            # 创建用于显示的带注释图像
            annotated_image = image.copy()  # 使用原始BGR图像进行标注
            
            if results.pose_landmarks:
                # 绘制姿势关键点
                mp_drawing = mp.solutions.drawing_utils
                mp_drawing.draw_landmarks(
                    annotated_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                
                # 计算并显示角度
                right_hip = self.draw_angle(annotated_image,
                                          results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER],
                                          results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP],
                                          results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_KNEE],
                                          (255, 0, 0), "right_hip")
                
                right_knee = self.draw_angle(annotated_image,
                                           results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER],
                                           results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_KNEE],
                                           results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ANKLE],
                                           (255, 0, 0), "right_knee")
                
                right_ankle = self.draw_angle(annotated_image,
                                            results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HEEL],
                                            results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ANKLE],
                                            results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_FOOT_INDEX],
                                            (255, 0, 0), "right_ankle")
            
            # 保存带注释的图像到视频 (在绘制完所有角度后)
            self.annotated_out.write(annotated_image)
            
            # 显示图像
            cv2.imshow('right Pose', annotated_image)
            
            # 按 'q' 退出循环
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        # 释放资源
        self.release_resources()
        self.get_logger().info(f"原始视频已保存为 raw_right.mp4")
        self.get_logger().info(f"带注释视频已保存为 {self.video_filename}")

    def draw_angle(self, image, p1, p2, p3, color, joint_name):
        """在图像上绘制角度并返回角度值"""
        # 获取三个点的坐标
        img_height, img_width = image.shape[:2]
        x1, y1 = int(p1.x * img_width), int(p1.y * img_height)
        x2, y2 = int(p2.x * img_width), int(p2.y * img_height)
        x3, y3 = int(p3.x * img_width), int(p3.y * img_height)
        
        # 计算向量
        v1 = np.array([x1 - x2, y1 - y2])
        v2 = np.array([x3 - x2, y3 - y2])
        
        # 计算点积和模
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        # 避免除以零
        if norm_v1 > 0.001 and norm_v2 > 0.001:
            # 计算角度
            cos_angle = dot_product / (norm_v1 * norm_v2)
            # 处理可能的数值误差
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.degrees(np.arccos(cos_angle))
            
            # 在图像上绘制角度
            cv2.putText(image, f"{joint_name}: {int(angle)}°", (x2, y2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(image, f"Vis: {p2.visibility:.2f}", (x2, y2 + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 绘制连接线
            cv2.line(image, (x1, y1), (x2, y2), color, 2)
            cv2.line(image, (x3, y3), (x2, y2), color, 2)
            cv2.circle(image, (x2, y2), 5, (0, 255, 0), -1)  # 关节点
            
            return angle
        
        return 0
    
    def release_resources(self):
        """释放所有资源"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.out:
            self.out.release()
        if self.annotated_out:
            self.annotated_out.release()
        cv2.destroyAllWindows()


def save_list(data, filename):
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(data, f)
    print("保存成功")

def main(args=None):
    rclpy.init(args=args)
    right_side_camera_node = RightSideCamera()
    
    # 创建并启动线程
    processing_thread = threading.Thread(target=right_side_camera_node.run)
    processing_thread.daemon = True  # 设置为守护线程
    processing_thread.start()
    
    try:
        # ROS2 主循环
        rclpy.spin(right_side_camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保资源释放
        right_side_camera_node.release_resources()
        right_side_camera_node.destroy_node()
        rclpy.shutdown()
    
    # 等待处理线程结束
    processing_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()