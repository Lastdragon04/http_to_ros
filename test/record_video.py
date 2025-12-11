import cv2

# 初始化摄像头（0 表示默认摄像头）
cap = cv2.VideoCapture(0)

# 设置视频参数
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # 获取摄像头分辨率
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = 30.0  # 帧率
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 编码器（XVID兼容性好）

# 创建 VideoWriter 对象
out = cv2.VideoWriter('output.avi', fourcc, fps, (width, height))

# 检查是否初始化成功
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

print("录制中... 按 'q' 键停止")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("无法获取画面")
        break
    
    # 写入帧
    out.write(frame)
    
    # 显示预览
    cv2.imshow('Recording', frame)
    
    # 按q键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
out.release()
cv2.destroyAllWindows()
print("视频已保存为 output.avi")