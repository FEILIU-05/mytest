# client.py
import cv2
import socket
import numpy as np
import time
import json  # 使用JSON来格式化和解析数据，更健壮

# --- 配置 ---
SERVER_IP = "127.0.0.1"  # 重要：请修改为您的服务器的实际IP地址
SERVER_PORT = 9999       # 必须与服务器的端口号一致
JPEG_QUALITY = 85        # JPEG压缩质量 (0-100)，越高图像越清晰但体积越大
MAX_UDP_PACKET_SIZE = 65507 # UDP数据包的最大理论负载大小

def run_client():
    """运行UDP客户端，发送图像并接收结果"""
    # 1. 设置UDP客户端套接字
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (SERVER_IP, SERVER_PORT)

    # 2. 从摄像头获取图像
    print("正在从摄像头捕获图像...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 无法打开摄像头")
        return

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("❌ 摄像头读取失败")
        return
    print("✅ 图像捕获成功!")

    # 3. 将图像编码为JPEG格式
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ret:
        print("❌ 图像编码失败")
        return

    image_data = buffer.tobytes()
    data_size = len(image_data)

    print(f"准备发送图像，压缩后大小: {data_size / 1024:.2f} KB")

    try:
        # 4. 发送图像数据
        # 首先，发送一个包含总大小的起始数据包，让服务器知道要接收多少数据
        client_socket.sendto(f"SIZE:{data_size}".encode(), server_address)

        # 然后，以分块的方式发送整个图像数据
        for i in range(0, data_size, MAX_UDP_PACKET_SIZE):
            chunk = image_data[i:i + MAX_UDP_PACKET_SIZE]
            client_socket.sendto(chunk, server_address)
            time.sleep(0.0001) # 短暂延时，有助于防止在某些网络下丢包

        print("✅ 图像发送完毕.")

        # 5. 等待并接收服务器的识别结果
        print("等待服务器返回结果...")
        client_socket.settimeout(10.0)  # 设置10秒超时
        response_data, _ = client_socket.recvfrom(4096)  # 为结果预留4KB缓冲区

        # 6. 解析并打印结果
        detections = json.loads(response_data.decode())

        print("\n--- 识别结果 ---")
        if detections:
            print(f"检测到 {len(detections)} 个物体:")
            for obj in detections:
                label = obj['label']
                score = obj['score']
                box = obj['box']
                print(f"  - 类别: {label}, 置信度: {score:.2f}, 边界框 (x,y,w,h): {box}")
        else:
            print("未检测到任何物体。")

    except socket.timeout:
        print("❌ 服务器响应超时，请检查服务器是否运行或网络连接是否正常。")
    except Exception as e:
        print(f"❌ 发生未知错误: {e}")
    finally:
        client_socket.close()
        print("连接已关闭。")

if __name__ == '__main__':
    run_client()