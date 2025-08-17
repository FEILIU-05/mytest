import cv2
import subprocess
import os
from datetime import datetime

os.environ["LD_LIBRARY_PATH"] = f"/usr/local/cuda/lib64:{os.environ.get('LD_LIBRARY_PATH', '')}"
frame_id = 1

# 1. 获取摄像头图像
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cap.release()

if not ret:
    print("❌ 摄像头读取失败")
    exit(1)

# 2. 保存图像
now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
# image_path = f"/home/ucar/Desktop/ucar/uvtest/test_jpg/input_{now_str}.jpg"
image_path = f"/home/ucar/Desktop/ucar/src/main/识别/frames/{frame_id}.jpg"
print(f"------------------ {frame_id} --------------------")
frame_id += 1
print(frame)
cv2.imwrite(image_path, frame)

# 生成输出文件名
output_dir = "/home/ucar/Desktop/ucar/uvtest/results"
base_name = os.path.splitext(os.path.basename(image_path))[0]
output_path = os.path.join(output_dir, f"{base_name}_det.jpg")

# 3. 使用 uv 虚拟环境下的解释器
uv_python ="/home/ucar/Desktop/ucar/uvtest/.venv/bin/python3.8"# ← 用上面查到的路径替换这里

command = [uv_python, "/home/ucar/Desktop/ucar/uvtest/detect6.py", 
            "--img", image_path,  # 添加必需的 --img 参数
            "--output", output_path]

# 4. 保持环境一致
env = os.environ.copy()

# 5. 调用识别模块
result = subprocess.run(
    command,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    universal_newlines=True,
    env=env
)

# 6. 输出结果
print("识别模块返回：")
print(result.stdout)
if result.stderr:
    print("⚠️ 识别模块错误输出：")
    print(result.stderr)
