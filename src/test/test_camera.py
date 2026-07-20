import sys
from pathlib import Path
# 确保可以导入 src 下的 image_source（如果脚本在 src/test 下运行）
sys.path.insert(0, str(Path(__file__).parent.parent))

import cv2
from image_source import ImageSource

def test_camera(source=0, width=640, height=480, fps=30, display=True):
    """
    测试摄像头读取
    
    参数:
        source: 摄像头索引，通常 0 为内置摄像头，1,2... 为外接
        width, height: 期望的分辨率（实际可能不支持）
        fps: 期望的帧率（实际可能不支持）
        display: 是否显示实时画面窗口
    """
    print(f"尝试打开摄像头 {source}，分辨率 {width}x{height}，帧率 {fps}...")
    
    try:
        with ImageSource(source, width=width, height=height, fps=fps) as cam:
            print(f"成功打开: {cam.source_name}")
            
            # 获取实际属性（可能和请求的不同）
            props = cam.properties()
            print(f"实际属性: 宽={props['width']:.0f}, 高={props['height']:.0f}, FPS={props['fps']:.2f}")
            
            if display:
                window_name = f"Camera {source} (Press ESC to exit)"
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                
            frame_count = 0
            while True:
                packet = cam.read()
                if packet is None:
                    print("读取帧失败，可能摄像头已断开或读取超时")
                    break
                
                frame_count += 1
                if frame_count % 30 == 0:
                    print(f"已读取 {frame_count} 帧，最新帧 ID={packet.frame_id}")
                
                if display:
                    # 显示图像
                    cv2.imshow(window_name, packet.image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC 键退出
                        print("用户按下 ESC，退出预览")
                        break
                    # 也可以按 's' 保存当前帧
                    elif key == ord('s'):
                        save_path = f"capture_frame_{packet.frame_id}.jpg"
                        cv2.imwrite(save_path, packet.image)
                        print(f"已保存帧到 {save_path}")
            
            if display:
                cv2.destroyAllWindows()
                
            print(f"摄像头测试结束，共读取 {frame_count} 帧")
            
    except RuntimeError as e:
        print(f"摄像头打开失败: {e}")
        print("请尝试：")
        print("  1. 确保摄像头未被其他软件占用（如 Zoom、OBS）")
        print("  2. 尝试更换 source 参数为 1 或 2")
        print("  3. 检查摄像头驱动是否正常")
    except KeyboardInterrupt:
        print("用户中断测试")
    finally:
        print("测试完成")

if __name__ == "__main__":
    # 可以修改参数来尝试不同的摄像头
    # 例如：test_camera(source=1)  # 尝试外接摄像头
    
    # 默认测试摄像头 0，分辨率 640x480，显示画面
    test_camera(source=0, width=640, height=480, fps=30, display=True)