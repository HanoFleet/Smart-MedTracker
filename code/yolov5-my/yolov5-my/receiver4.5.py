import socket
import struct
import cv2
import torch
import numpy as np
import time

from models.yolo import Model

# 创建模型对象，传入配置文件或参数
model = Model('D:\\my\\yolov5-my\\models\\yolov5s.yaml')

# 加载权重
weights = torch.load(r'D:\\my\\yolov5-my\\best.pt', map_location='cpu')
model.load_state_dict(weights['model'].state_dict())  # 加载模型权重
model.eval()  # 切换到评估模式

# TCP 服务器设置，用于接收视频流
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('', 9999))  # 绑定端口 9999
server_socket.listen(1)
print("等待接收视频流...")

conn, addr = server_socket.accept()
coordinate_count = 0
max_coordinates_to_send  = 20
pause_duration = 1  # 暂停时间（秒）

print(f"接收到来自 {addr} 的连接")

# 准备与树莓派建立连接，用于发送检测到的中心坐标
raspi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def connect_to_raspi():
    """连接树莓派"""
    global raspi_socket
    while True:
        try:
            raspi_socket.connect(('192.168.12.1', 9996))  # 替换为树莓派的 IP 和端口
            print("连接树莓派成功")
            break
        except Exception as e:
            print(f"与树莓派连接失败: {e}, 正在重试...")
            time.sleep(5)  # 等待5秒再重试

# 首次连接
connect_to_raspi()

payload_size = struct.calcsize("!I")  # 定义接收数据的大小
data = b""

def receive_and_process_frame():
    global data

    while True:  # 无限循环接收和处理帧
        try:
            # 接收完整的数据包
            while len(data) < payload_size:
                packet = conn.recv(4096)
                if not packet:
                    break
                data += packet

            # 解析消息大小
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("!I", packed_msg_size)[0]

            # 确保接收完整的一帧数据
            while len(data) < msg_size:
                data += conn.recv(4096)

            # 获取并解码帧数据
            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            # 进行目标检测和推理
            process_frame_and_send_coordinates(frame)

            # 显示结果图像
            cv2.imshow('Result Image', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"接收或处理数据时出错: {e}")
            break

def process_frame_and_send_coordinates(frame):
    original_h, original_w = frame.shape[:2]

    # 转换图像为张量
    frame_tensor = torch.from_numpy(frame).permute(2, 0, 1).unsqueeze(0).float() / 255.0

    # 将图像传入模型进行检测
    results = model(frame_tensor)

    # 如果模型的输出是 tuple，提取第一个元素作为张量
    if isinstance(results, tuple):
        results = results[0]

    # 获取检测结果
    detections = results[0]  # 假设 results 形状为 (1, 18900, 11)
    boxes = detections[:, :4]  # 边界框 (x_center, y_center, width, height)
    scores = detections[:, 4]  # 置信度分数 
    labels = torch.argmax(detections[:, 5:], dim=1)  # 类别标签

    # 设置置信度阈值
    confidence_threshold = 0.7
    high_conf_idx = scores > confidence_threshold
    boxes = boxes[high_conf_idx]
    scores = scores[high_conf_idx]
    labels = labels[high_conf_idx]

    # 偏移量设置
    y_offset = 50  # 调整检测框在Y轴上的偏移，向下移动50个像素W

    # 绘制检测框
    for i in range(boxes.shape[0]):
        box = boxes[i].detach().cpu().numpy()

        # 将比例坐标转换为原始图像的坐标
        x_center = box[0] * original_w / 640
        y_center = box[1] * original_h / 640
        width = box[2] * original_w / 640
        height = box[3] * original_h / 640

        # 将中心坐标转换为左上角和右下角的坐标
        x1 = int(x_center - width / 2)
        y1 = int(y_center - height / 2) + y_offset  # 加上偏移量使框向下移动
        x2 = int(x_center + width / 2)
        y2 = int(y_center + height / 2) + y_offset  # 同样对右下角Y坐标偏移

        label = labels[i].item()
        score = scores[i].item()

    # 绘制边界框和标签
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'Label: {label}, Score: {score:.2f}', (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # 只在有有效检测结果时发送坐标
    if boxes.shape[0] > 0:
        send_coordinates(frame, boxes, scores, labels)

def send_coordinates(frame, boxes, scores, labels):
    global raspi_socket,coordinate_count
    try:
        for i in range(boxes.shape[0]):
            box = boxes[i].detach().cpu().numpy()
            label = labels[i].detach().cpu().item()  # Get the label for this detection

            x_center = box[0] * frame.shape[1] / 640
            y_center = box[1] * frame.shape[0] / 640
            width = box[2] * frame.shape[1] / 640
            height = box[3] * frame.shape[0] / 640

            # 将浮点数坐标转换为整数
            x_center = int(round(x_center))
            y_center = int(round(y_center))
            width = int(round(width))
            height = int(round(height))

            # 生成要发送的坐标
            coords = f"{x_center},{y_center},{width},{height},{label}"
            print(f"Sending coordinates: {coords}")

            # 发送坐标数据
            raspi_socket.sendall(coords.encode())
            time.sleep(0.3)

            coordinate_count += 1

            if coordinate_count >= max_coordinates_to_send:
                print(f"已发送 {max_coordinates_to_send} 组坐标，暂停 {pause_duration} 秒。")
                time.sleep(pause_duration)  # 暂停
                coordinate_count = 0  # 重置计数器
 
        time.sleep(0.3)  # 添加延时以降低发送频率，避免过载

    except Exception as e:
        print(f"发送坐标时出错: {e}")
        # 关闭并重连
        raspi_socket.close()
        raspi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connect_to_raspi()

# 开始接收和处理帧
receive_and_process_frame()

# 关闭连接
conn.close()
raspi_socket.close()