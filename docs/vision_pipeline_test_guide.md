# VisionPipeline 离线测试指南

## 1. 测试层次

### 自动单元测试

```powershell
conda activate D:\conda_envs\rg_vision
cd D:\RoboGameVision
python -m unittest tests.test_vision_output tests.test_vision_pipeline -v
```

完整回归：

```powershell
python -m unittest discover -s tests -v
```

### 离线图片/视频测试

使用 `tools/run_pipeline_offline.py`。它不是比赛主程序，不打开真实摄像头，也不连接下位机。每一行输出一个完整 `VisionOutput` JSON，文件扩展名建议使用 `.jsonl`。

---

## 2. 测试单张场地图片

测试 AprilTag 定位模式：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\tag&blocks2.jpg" `
  --mode LOCALIZATION `
  --output "outputs\pipeline_localization.jsonl"
```

静态图片默认重复输入 3 次，用来满足连续帧跟踪器的三帧确认条件。JSONL 前两行通常没有稳定标签，第三行应出现 ID 5 和 ID 6。

测试方块：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\tag&blocks2.jpg" `
  --mode FIND_BLOCKS `
  --output "outputs\pipeline_blocks.jsonl"
```

测试全部模块：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\tag&blocks2.jpg" `
  --mode DEBUG_ALL `
  --output "outputs\pipeline_debug_image.jsonl"
```

---

## 3. 测试视频

完整运行：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\test_video.mp4" `
  --mode DEBUG_ALL `
  --output "outputs\pipeline_video.jsonl"
```

只处理前 100 帧：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\test_video.mp4" `
  --mode DEBUG_ALL `
  --max-frames 100 `
  --output "outputs\pipeline_video_100.jsonl"
```

每隔 3 帧处理一帧：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\test_video.mp4" `
  --mode FIND_BLOCKS `
  --sample-step 3 `
  --output "outputs\pipeline_blocks_step3.jsonl"
```

注意：跳帧会改变连续跟踪的实际时间间隔，不能用跳帧结果代替正式30 FPS稳定性测试。

---

## 4. 分模式验收

### IDLE

预期：只输出 `stream`，`tags/blocks` 为空，`line.valid=false`。

### LOCALIZATION

预期：运行 AprilTag 和标签连续跟踪；没有内外参时标签像素结果有效，但 `robot_pose.valid=false`。

### FIND_BLOCKS

预期：输出稳定方块；没有相机外参时 `position_robot_m=null`。

### FOLLOW_LINE

预期：只输出黑线偏移、方向角、交叉口状态。

### DEBUG_ALL

预期：三个算法同时运行，处理耗时明显高于单模式。该模式用于调试，不能直接代表树莓派比赛性能。

---

## 5. 配置坐标后的测试

只有内参和坐标参数都已经实测时才运行：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\test_video.mp4" `
  --mode LOCALIZATION `
  --calibration "configs\camera_calibration.json" `
  --coordinates "configs\coordinate_frames.json" `
  --output "outputs\pipeline_pose.jsonl"
```

手机视频不能使用机器人相机的内参。标定文件必须与拍摄视频的相机、镜头、分辨率和对焦状态匹配。

---

## 6. 检查 JSONL

JSONL 每行是一帧。重点检查：

- `schema_version` 是否为 `1.2`
- `frame_id` 是否递增
- `timestamp_s` 是否递增
- `stream.status` 是否合理
- `mode` 是否与命令一致
- `predicted=true` 是否只在短时遮挡出现
- 视频异常时顶层 `valid` 是否为 false
- 未配置坐标时米制坐标是否为 null
- `processing.total_ms` 是否低于帧周期
- `errors` 是否为空

30 FPS 的单帧时间预算约为：

```text
1000 / 30 = 33.3 ms
```

正式树莓派上应分别测试每个工作模式，而不是只看开发电脑结果。

---

## 7. 当前阶段不验证的内容

离线管线测试不能验证：

- USB摄像头拔线与重新连接
- CSI相机驱动稳定性
- 双摄像头USB带宽
- 树莓派温度与降频
- 串口或CAN丢包
- 下位机超时保护
- 开机自启动和崩溃重启

这些将在统一视觉主程序和通信层完成后进行硬件在环测试。
