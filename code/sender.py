import cv2
import socket
import struct
import pickle
import Camera
import threading
import time

# 创建线程锁
frame_lock = threading.Lock()

# 打开摄像头
cam = Camera.Camera()  # 摄像头库实例化
cam.camera_open()      # 打开摄像头

# TCP 连接设置（用于发送视频流）
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.12.239', 9999))  # 替换为电脑的 IP 地址
connection = client_socket.makefile('wb')


try:
    while True:
        # 等待获取帧，防止过快操作摄像头
        time.sleep(0.1)

        # 使用线程锁保护摄像头帧访问
        with frame_lock:
            frame = cam.frame
        
        if frame is None:
            print("未能获取到帧，结束发送")
            break

        # 在窗口显示摄像头画面
        cv2.imshow('Camera Feed', frame)

        # 检测是否按下ESC键 (27是ESC的ASCII码)，退出显示
        if cv2.waitKey(5) & 0xFF == 27:
            print("按下ESC键，退出显示")
            break

        # 对图像进行编码并发送
        # data = pickle.dumps(frame)

          # 将帧压缩为 JPEG 格式
        success, encoded_frame = cv2.imencode('.jpg', frame)
        if not success:
            print("JPEG 编码失败")
            continue
        data = encoded_frame.tobytes()  # 直接转换为字节流
        print(f"发送的数据长度: {len(data)}")
        print(f"发送的数据前几个字节: {data[:10]}")

        # 发送数据长度，确保接收端能够识别数据大小
        message_size = struct.pack("!I", len(data))

        # 发送数据
        client_socket.sendall(message_size + data)
except Exception as e:
    print("发送视频流时发生错误：", e)
finally:
    cam.camera_close()  # 使用 Camera 类中的方法关闭摄像头
    client_socket.close()
    cv2.destroyAllWindows()  # 关闭所有OpenCV窗口