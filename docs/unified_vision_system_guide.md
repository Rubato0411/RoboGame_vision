# 统一视觉输入主程序使用指南

入口文件：`run_vision_system.py`

该程序负责：

1. 使用 `ImageSource` 读取图片、视频或摄像头。
2. 将 `FramePacket` 交给 `VisionPipeline`。
3. 按工作模式运行相应算法。
4. 输出统一 `VisionOutput` JSON。
5. 可选写入JSONL、通信协议字节流和标注录像。
6. 显示可缩放调试窗口。

它不会删除或替代单模块测试程序。

## 图片端到端测试

```powershell
python run_vision_system.py `
  --source "data\tag&blocks2.jpg" `
  --mode LOCALIZATION `
  --jsonl "outputs\unified_image.jsonl" `
  --protocol-output "outputs\unified_image.rg" `
  --save-last "outputs\unified_image.jpg" `
  --quiet
```

## 视频端到端测试

```powershell
python run_vision_system.py `
  --source "data\test_video.mp4" `
  --mode DEBUG_ALL `
  --jsonl "outputs\unified_video.jsonl" `
  --record "outputs\unified_video.mp4"
```

## 摄像头测试

Windows开发电脑：

```powershell
python run_vision_system.py `
  --source 0 `
  --backend dshow `
  --fourcc MJPG `
  --width 1280 `
  --height 720 `
  --fps 30 `
  --mode DEBUG_ALL `
  --jsonl "outputs\camera_test.jsonl"
```

树莓派Linux通常将后端改为 `v4l2`。

## 模式建议

- 首次摄像头验收使用 `IDLE`，只验证输入和视频流。
- AprilTag测试使用 `LOCALIZATION`。
- 方块调参使用 `FIND_BLOCKS`。
- 黑线测试使用 `FOLLOW_LINE`。
- 短时间综合测试使用 `DEBUG_ALL`。

比赛时不建议长期运行 `DEBUG_ALL`。

## 当前边界

`--protocol-output`只把协议帧写入文件，用于验证编码和模拟下位机。真实串口/CAN传输尚未绑定，等待控制组确认硬件链路。
