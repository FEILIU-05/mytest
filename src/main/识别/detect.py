import cv2
import subprocess
import os

# 获取摄像头图像
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cap.release()

if not ret:
    print("❌ 摄像头读取失败")
    exit(1)

# 保存图像到临时文件
image_path = "/home/ucar/Desktop/ucar/uvtest/test_jpg/temp_input.jpg"
cv2.imwrite(image_path, frame)

# 调用识别模块脚本（通过 uv run）
command = ["uv", "run", "/home/ucar/Desktop/ucar/uvtest/detect_camera.py", image_path]

try:
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,  # python3.6兼容
        env=os.environ.copy()      # 传递当前环境变量
    )
except Exception as e:
    print("调用识别模块时出错: {}".format(e))
    exit(1)

print("识别模块返回：")
print(result.stdout)

if result.stderr:
    print("⚠️ 识别模块错误输出：")
    print(result.stderr)
