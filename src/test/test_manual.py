import sys
from pathlib import Path

# 将当前脚本的父目录（即 src）添加到 sys.path
sys.path.insert(0, str(Path(__file__).parent.parent))

# 然后再导入
from image_source import ImageSource
# ... 其余代码
import cv2
import numpy as np
from image_source import ImageSource

# 1. 生成一个临时测试图片 (避免依赖外部文件)
def create_dummy_image(path="test_img.jpg"):
    img = np.full((480, 640, 3), 100, dtype=np.uint8)  # 灰色背景
    cv2.putText(img, "Test", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    cv2.imwrite(path, img)
    return path

# 2. 生成一个临时测试视频 (包含几帧)
def create_dummy_video(path="test_vid.mp4"):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(path, fourcc, 10.0, (640, 480))
    for i in range(10):  # 10帧
        frame = np.full((480, 640, 3), 50 + i*20, dtype=np.uint8)
        out.write(frame)
    out.release()
    return path

if __name__ == "__main__":
    # --- 测试 1: 静态图片 ---
    print(">>> 测试静态图片...")
    img_path = create_dummy_image()
    with ImageSource(img_path) as src:
        print(f"  是否静态图: {src.is_still_image}")  # 应为 True
        print(f"  属性: {src.properties()}")         # 应显示宽高
        packet1 = src.read()
        print(f"  第1次读取帧 ID: {packet1.frame_id}, 形状: {packet1.image.shape}")
        packet2 = src.read()
        print(f"  第2次读取帧 ID: {packet2.frame_id}, 形状: {packet2.image.shape}")
        # 验证帧号递增（而不是返回 None）
        assert packet2.frame_id == 1  # 因为第一次是0，第二次是1
        # 并且两张图像的内容相同（因为是复制的副本）
        assert (packet1.image == packet2.image).all()
        print("  静态图片：连续读取两次均返回图像，帧号递增 ✓")
    print("图片测试通过！\n")

    # --- 测试 2: 视频文件 (并测试循环) ---
    print(">>> 测试视频文件 (循环模式)...")
    vid_path = create_dummy_video()
    with ImageSource(vid_path, loop_video=True) as src:
        max_frames = 25   # 读取 25 帧（超过 10 帧，会循环 2.5 次）
        count = 0
        last_frame_id = -1
        while count < max_frames:
            packet = src.read()
            if packet is None:
                break  # 理论上不会发生，但以防万一
            count += 1
            last_frame_id = packet.frame_id
        # 验证是否读了超过总帧数（说明循环生效）
        assert count > 10, f"循环模式未能读到超过10帧，只读到了{count}帧"
        print(f"  循环模式下读取 {count} 帧，最后一帧 frame_id={last_frame_id}，循环成功！")
    print("视频测试通过！\n")

    # --- 测试 3: 上下文管理器和资源释放 ---
    print(">>> 测试上下文管理器 (自动释放)...")
    with ImageSource(img_path) as src:
        packet = src.read()
        print(f"  读到帧 ID: {packet.frame_id}")
    # 退出 with 块后资源应已释放（此处无异常即为成功）
    print("上下文管理测试通过！")
    
    # 清理生成的文件 (可选)
    # import os; os.remove(img_path); os.remove(vid_path)