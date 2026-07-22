# Raspberry Pi 5 双 CSI 摄像头使用指南

本项目通过 Picamera2/libcamera 原生读取 CSI 摄像头。`source` 是
`rpicam-hello --list-cameras` 显示的相机编号，不是 `/dev/video*` 编号。

## 安装和环境

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv rpicam-apps
rpicam-hello --list-cameras
```

Picamera2 依赖系统提供的 libcamera，虚拟环境必须允许读取系统包：

```bash
cd ~/RoboGame_vision
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -c "from picamera2 import Picamera2; print(Picamera2.global_camera_info())"
```

不要使用 `pip install picamera2` 替代树莓派系统包。

## 分别验证两台相机

假设 0 是前视、1 是夹爪：

```bash
python run_viewer.py --source 0 --backend picamera2 --width 1280 --height 720 --fps 30
python run_viewer.py --source 1 --backend picamera2 --width 1280 --height 720 --fps 30
```

无桌面 SSH 会话可先使用官方命令验证：

```bash
rpicam-still --camera 0 -n -o /tmp/front.jpg
rpicam-still --camera 1 -n -o /tmp/gripper.jpg
```

## SSH 无窗口采集内参照片

下面的 `24.85` 必须替换为打印后实测的方格边长：

```bash
python capture_calibration.py \
  --source 0 --backend picamera2 \
  --output-dir data/calibration/front_camera \
  --columns 9 --rows 6 --square-mm 24.85 \
  --width 1280 --height 720 --fps 30 \
  --no-display --interval-s 3 --max-images 40

python capture_calibration.py \
  --source 1 --backend picamera2 \
  --output-dir data/calibration/gripper_camera \
  --columns 9 --rows 6 --square-mm 24.85 \
  --width 1280 --height 720 --fps 30 \
  --no-display --interval-s 3 --max-images 40
```

程序只在检测到完整角点时保存。每次输出 `Saved` 后改变棋盘距离、位置和倾角。
用 `Ctrl+C` 可提前结束。

成像参数确定后可追加手动控制，例如：

```bash
--exposure-time-us 8000 \
--analogue-gain 2.0 \
--colour-gains 1.5 1.4 \
--lens-position 2.5
```

这些数值只是命令格式示例，必须换成各相机的实测值。设置手动值时，代码会关闭对应的
自动曝光、自动白平衡或自动对焦。

## 双摄像头同时测试

```bash
python run_camera_monitor.py \
  --front 0 --gripper 1 \
  --backend picamera2 \
  --width 1280 --height 720 --fps 30
```

两路 CSI 可以同时打开，但最终仍需测量两路实际 FPS、丢帧率、温度和持续运行稳定性。

## 正式配置

在 `configs/hardware_measurements.json` 中为两台相机分别填写：

- `device_path`：libcamera 相机编号，通常为 `0` 或 `1`；
- `backend`：`picamera2`；
- `exposure_time_us`、`analogue_gain`、`colour_gains`；
- 自动对焦相机的 `lens_position`；
- 实际分辨率、FPS、内参文件和外参文件。

`actual_fourcc` 对 Picamera2 不适用；运行时使用 `RGB888` 主流，并直接提供 OpenCV BGR
数组。USB 摄像头和离线图片/视频入口仍然保留。
