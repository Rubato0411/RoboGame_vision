# 机械臂视觉联调工具

这两个入口仅用于调试。它们不会发送机械臂运动命令，也不会把
`hardware_measurements.json` 强制标记为已验收。

## 动态夹爪三维坐标

前提：手眼标定已经生成 `configs/calibration_results/hand_eye_gripper.json`，STM32通过
既有协议持续发送带递增 `sample_sequence` 的 `RobotFeedback.gripper_pose`。

```bash
python tools/run_gripper_3d_commissioning.py \
  --source 0 --backend picamera2 \
  --calibration configs/camera_gripper_calibration.json \
  --hand-eye configs/calibration_results/hand_eye_gripper.json \
  --serial-port /dev/serial0 --serial-baud 115200 \
  --pose-timeout 0.15 --target-color orange \
  --jsonl outputs/gripper_3d_commissioning.jsonl
```

输出中的关键字段：

- `pose_valid`：当前STM32末端位姿是否新鲜；
- `pose_sequence`、`pose_age_s`、`pose_reason`：序号、年龄和失效原因；
- `blocks[].position_robot_m`、`blocks[].grasp_point_robot_m`：动态机器人坐标；
- `alignment`：二维对准误差。

固定方块后改变机械臂姿态，正确的机器人坐标应基本不变。停止末端位姿更新超过
`--pose-timeout` 后，三维机器人坐标必须失效。工具仍会回传视觉协议包，使用
`--no-publish-vision` 可关闭回传。

## 指定平台槽位

```bash
python tools/test_placement_slot_live.py \
  --slot-id building_1_level_1 \
  --source 1 --backend picamera2 \
  --calibration configs/camera_front_calibration.json \
  --coordinates configs/coordinate_front.json \
  --slots configs/placement_slots.json \
  --jsonl outputs/building_1_level_1.jsonl
```

将 `--slot-id` 依次替换为 `building_1_level_1` 至 `building_3_level_4`。输出中的
`slot.valid`、`slot.position_robot_m`、`slot.rpy_robot_deg` 是当前指定槽位结果。Tag不可见、
槽位未配置或数据流不健康时不得驱动机械臂。

列出当前可测试槽位：

```bash
python - <<'PY'
from tools.test_placement_slot_live import configured_slot_ids
for value in configured_slot_ids('configs/placement_slots.json'):
    print(value)
PY
```

两项测试都应先空夹爪、限速、限步长并保留急停。树莓派输出目标位姿，IK、轨迹规划、
碰撞保护和到位确认由机械臂/电控负责。
