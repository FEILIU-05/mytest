import socket
import pickle
import struct
import cv2
import numpy as np

def receive_numpy_matrix():
    # 1. 创建 TCP 服务器 socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 绑定地址（空字符串表示监听所有本地 IP，端口 12345）
    host = ''
    port = 12345
    server_socket.bind((host, port))
    server_socket.listen(1)  # 最多允许 1 个客户端连接
    print(f"服务器已启动，监听 {host}:{port}，等待客户端连接...")

    # 2. 接受客户端连接
    client_socket, client_addr = server_socket.accept()
    print(f"客户端 {client_addr} 已连接，开始接收图像帧...")

    # 3. 接收缓冲区（用于拼接数据）
    data_buffer = b''
    # 数据长度的格式（4 字节无符号整数）
    payload_size = struct.calcsize('I')

    try:
        while True:
            # 4. 接收数据长度（先获取 4 字节的长度信息）
            while len(data_buffer) < payload_size:
                # 每次接收 4096 字节（可根据网络调整）
                data_buffer += client_socket.recv(4096)

            # 提取数据长度（struct.unpack 解析 4 字节为整数）
            packed_size = data_buffer[:payload_size]
            data_buffer = data_buffer[payload_size:]
            data_size = struct.unpack('I', packed_size)[0]

            # 5. 接收完整数据（直到收到指定长度的数据）
            while len(data_buffer) < data_size:
                data_buffer += client_socket.recv(4096)

            # 提取当前帧的数据
            frame_data = data_buffer[:data_size]
            data_buffer = data_buffer[data_size:]  # 剩余数据留到下一帧

            # 6. 反序列化（恢复为 numpy 矩阵）
            frame = pickle.loads(frame_data)

            # 7. 处理接收的矩阵（示例：显示图像、保存等）
            print(f"已接收帧 | 矩阵形状：{frame.shape} | 数据类型：{frame.dtype}")
            # 显示接收的图像
            # cv2.imshow("接收的图像", frame)
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("用户请求退出")
                break

    except Exception as e:
        print(f"接收过程出错：{e}")
    finally:
        # 释放资源
        cv2.destroyAllWindows()
        client_socket.close()
        server_socket.close()
        print("服务器已关闭")

if __name__ == "__main__":
    receive_numpy_matrix()