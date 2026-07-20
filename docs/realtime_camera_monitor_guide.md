# 实时摄像头监控与rosbag说明

## rosbag

rosbag用于记录ROS Topic，不负责识别。若将来采用ROS2，可记录：

- `/camera/front/image_raw`
- `/camera/gripper/image_raw`
- `/camera/*/camera_info`
- `/tf`、`/tf_static`
- 里程计、机械臂关节状态
- 视觉检测和任务状态

当前项目不是ROS节点，因此先使用OpenCV监控、MP4和JSONL健康日志。不要为了录相机画面单独引入ROS。

## 单相机监控

```powershell
python run_camera_monitor.py --front 0 --backend dshow --width 1280 --height 720 --fps 30
```

## 双相机监控

```powershell
python run_camera_monitor.py --front 0 --gripper 1 --backend dshow `
  --width 1280 --height 720 --fps 30 `
  --health-log "outputs\camera_health.jsonl" `
  --record "outputs\camera_dashboard.mp4"
```

Linux/树莓派一般使用 `--backend v4l2`，并应尽量改用稳定的
`/dev/v4l/by-id/...`路径，避免USB插拔后0/1编号互换。

## 按键

- `Q`/`Esc`：退出
- `S`：分别保存当前两台相机原始帧

## 无界面测试

```powershell
python run_camera_monitor.py --front 0 --backend dshow --no-display --duration 60 `
  --health-log "outputs\camera_health.jsonl"
```

监控画面的绿框表示健康，红框表示冻结、超时、断联或其他异常。两台相机使用独立读取线程，一台读取阻塞时不会直接卡住另一台画面的刷新。
