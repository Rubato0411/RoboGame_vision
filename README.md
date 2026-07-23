# RoboGame 视觉输入程序

> 树莓派 5 双 CSI 摄像头使用 `--backend picamera2`，安装、标定和双路验证命令见
> `docs/raspberry_pi_csi_camera_guide.md`。

> 正式比赛入口与硬件参数说明：
> `run_competition_system.py`、`configs/hardware_measurements.json`、
> `docs/hardware_measurement_checklist.md`。硬件参数未实测时，正式入口会拒绝启动。
>
> 各视觉模块的独立测试命令、参数对应表和视觉/电控职责边界见：
> `docs/visual_module_test_guide.md`。
>
> 三方块依次装入车载槽位、到搭建区后逐块取出的状态机和电控接口见：
> `docs/three_block_cargo_workflow.md`。

项目提供统一的图片、视频和USB/UVC摄像头输入接口。检测算法始终从 `FramePacket.image` 获取图像，因此从离线视频切换到真实机器人摄像头时不需要重写检测器。

## 环境

Conda环境：

```text
D:\conda_envs\rg_vision
```

VS Code中选择解释器：

```text
D:\conda_envs\rg_vision\python.exe
```

激活并运行：

```powershell
conda activate D:\conda_envs\rg_vision
cd D:\RoboGameVision
python run_viewer.py --source 0
```

## 常用命令

笔记本摄像头：

```powershell
python run_viewer.py --source 0
```

Windows下明确使用DirectShow和MJPEG：

```powershell
python run_viewer.py --source 0 --backend dshow --fourcc MJPG --width 1280 --height 720 --fps 30
```

尝试YUYV无压缩格式：

```powershell
python run_viewer.py --source 0 --backend dshow --fourcc YUYV --width 640 --height 480 --fps 30
```

Linux/Jetson的UVC相机通常使用：

```bash
python run_viewer.py --source 0 --backend v4l2 --fourcc MJPG --width 1280 --height 720 --fps 30
```

读取图片或视频：

```powershell
python run_viewer.py --source "D:\data\test.jpg"
python run_viewer.py --source "D:\data\test.mp4" --loop
```

固定相机控制参数示例：

```powershell
python run_viewer.py --source 0 --backend dshow --fourcc MJPG `
  --width 1280 --height 720 --fps 30 `
  --auto-exposure 0.25 --exposure -6 --gain 20 `
  --auto-white-balance 0 --white-balance 4500 `
  --auto-focus 0 --focus 20
```

注意：曝光等控制值没有跨摄像头、跨驱动统一的范围。上述数字只是DirectShow常见示例，必须根据实物回读和测试调整。按 `P` 可打印当前实际参数。

## 快捷键

| 按键 | 功能 |
|---|---|
| `Q`或`Esc` | 退出 |
| `Space` | 暂停或继续 |
| `S` | 保存原始截图到`captures/` |
| `R` | 开始或停止录制 |
| `P` | 打印驱动实际参数 |

## 真实摄像头需要调整的参数类别

### 1. 设备与驱动

| 参数 | 命令 | 说明 |
|---|---|---|
| 设备编号 | `--source` | Windows常见为0/1/2；插拔顺序可能改变编号 |
| 后端 | `--backend` | Windows优先试`dshow`，再试`msmf`；Linux/Jetson使用`v4l2` |
| 编码格式 | `--fourcc` | 常见`MJPG`、`YUYV`；必须匹配相机支持列表 |

正式机器人不要长期依赖Windows设备编号。Linux中应依据USB序列号建立稳定的`/dev/v4l/by-id/...`设备映射；ROS2相机节点也应使用稳定设备路径。

### 2. 视频模式

| 参数 | 命令 | 调整依据 |
|---|---|---|
| 分辨率 | `--width/--height` | 标签最远识别像素、方块定位精度、算力 |
| 帧率 | `--fps` | 机器人速度、控制频率、USB带宽 |
| 缓冲区 | `--buffer-size` | 优先低延迟；部分驱动不支持设置 |

摄像头是否真的接受请求参数，要看启动时输出的实际`width/height/fps/fourcc`。不要只相信采购页标称值。

### 3. 成像控制

| 参数 | 命令 | 目的 |
|---|---|---|
| 自动曝光模式 | `--auto-exposure` | 比赛时通常关闭，防止HSV颜色漂移 |
| 曝光时间 | `--exposure` | 越短运动模糊越小，但画面更暗 |
| 增益 | `--gain` | 提亮画面，但高增益会增加噪声 |
| 白平衡模式/色温 | `--auto-white-balance/--white-balance` | 固定橙色、紫色的颜色表现 |
| 对焦模式/位置 | `--auto-focus/--focus` | 机器固定工作距离时建议锁定 |
| 亮度、对比度、饱和度、锐度 | 对应同名参数 | 后期微调，不应代替正确曝光和照明 |

这些参数由驱动解释。某些相机不支持回读，或返回`-1/0`，需要同时使用厂商工具、Windows相机属性页或Linux的`v4l2-ctl`确认。

### 4. 稳定性参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--warmup-frames` | 10 | 丢弃刚启动时曝光尚未稳定的帧 |
| `--open-retries` | 3 | 初次打开失败时重试 |
| `--max-read-failures` | 5 | 连续失败多少帧后执行重连 |
| `--reconnect-retries` | 5 | 断线后的最大重连次数 |
| `--reconnect-delay` | 0.5秒 | 重连间隔 |

正式比赛还应由上层监控`FramePacket.timestamp`。如果图像年龄超过控制允许值，即使画面仍能显示，也必须将视觉结果标记为无效并停止视觉伺服。

## 按摄像头角色推荐的初始配置

前视定位相机：

```text
1280x720或1920x1080
30 FPS
MJPG
短曝光、固定增益、固定白平衡、固定焦距
```

夹爪近距相机：

```text
1280x720
30～60 FPS
MJPG或相机支持的低延迟格式
固定曝光、固定白平衡、固定焦距
buffer_size=1
```

若两台相机共享同一USB控制器，需要同时启动测试。单相机能达到的帧率不代表双相机同时工作时仍能达到。

## 代码接口

```python
from src.image_source import CameraConfig, ImageSource

config = CameraConfig(
    width=1280,
    height=720,
    fps=30,
    backend="dshow",
    fourcc="MJPG",
    buffer_size=1,
)

with ImageSource(0, camera_config=config) as source:
    while True:
        packet = source.read()
        if packet is None:
            break

        frame = packet.image
        timestamp = packet.timestamp
        # detections = detector.process(frame)
```

## 到货后的调参顺序

1. 关闭其他占用摄像头的软件。
2. 确认设备编号和后端能打开。
3. 查询相机支持的分辨率、FPS和FOURCC组合。
4. 先固定分辨率、FPS和FOURCC。
5. 固定焦距。
6. 在接近比赛照明下固定曝光、增益和白平衡。
7. 再进行相机内参标定。
8. 固定安装后进行外参标定。
9. 两台相机同时运行30分钟，检查掉帧、断线、温度和USB供电。

标定后改变分辨率、焦距或相机安装角度，通常需要重新验证甚至重新标定。
