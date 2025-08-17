import cv2
import os
from datetime import datetime

def capture_and_save_image():
    """
    摄像头拍照并保存图片功能
    返回: 
        - 成功: 返回保存的图片路径
        - 失败: 返回None
    """
    # 1. 初始化摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 错误：无法打开摄像头")
        return None
    
    print("📷 摄像头已就绪，准备拍照...")
    
    # 2. 拍照
    ret, frame = cap.read()
    cap.release()  # 立即释放摄像头
    
    if not ret:
        print("❌ 错误：无法从摄像头获取图像")
        return None
    
    # 3. 创建保存目录（如果不存在）
    save_dir = "/home/ucar/Desktop/ucar/src/main/jpg"
    os.makedirs(save_dir, exist_ok=True)
    
    # 4. 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    image_path = os.path.join(save_dir, f"capture_{timestamp}.jpg")
    
    # 5. 保存图片
    if cv2.imwrite(image_path, frame):
        print(f"✅ 图片保存成功：{image_path}")
        return image_path
    else:
        print(f"❌ 错误：无法保存图片到 {image_path}")
        return None

if __name__ == "__main__":
    print("📷 拍照程序已启动")
    print("🔼 按回车键拍照")
    print("⏹️ 输入 'q' 然后按回车退出程序")
    
    while True:
        user_input = input("等待输入: ")
        
        if user_input.lower() == 'q':
            print("👋 程序退出")
            break
            
        saved_path = capture_and_save_image()
        if saved_path:
            print(f"🖼️ 图片已保存至：{saved_path}")
        else:
            print("⚠️ 图片保存失败，请检查摄像头和存储权限")
            
        print("\n🔼 按回车键继续拍照，或输入 'q' 退出")