# 树莓派视觉通信协议 v1

## 1. 两端职责

### 树莓派视觉端

- 从 `VisionPipeline` 获取 `VisionOutput`。
- 为每个数据包分配递增序号。
- 将统一输出编码成字节帧。
- 添加长度和 CRC32 校验。
- 接收下位机工作模式命令。
- 对有效命令返回 ACK。

### 模拟下位机

模拟真实 STM32/运动控制器：

- 发送 `IDLE/LOCALIZATION/FIND_BLOCKS/FOLLOW_LINE/DEBUG_ALL` 模式命令。
- 接收并验证视觉数据包。
- 处理串口常见的半包、粘包和随机分块。
- 检查视觉数据是否超时。
- 拒绝 `valid=false` 或视频流不健康的结果。

模拟下位机只用于电脑测试，不控制真实电机。

## 2. 字节帧格式

所有多字节整数使用小端序。

| 字段 | 长度 | 说明 |
|---|---:|---|
| Magic | 2 | 固定 ASCII `RG` |
| Version | 1 | 当前为 1 |
| Message Type | 1 | 消息类型 |
| Sequence | 4 | uint32 递增序号 |
| Payload Length | 2 | uint16 JSON字节长度 |
| Payload | N | UTF-8紧凑JSON |
| CRC32 | 4 | 从Magic到Payload末尾的CRC32 |

消息类型：

- `0x01`：VisionOutput
- `0x02`：Heartbeat
- `0x10`：ModeCommand
- `0x11`：RobotFeedback（下位机到树莓派）
- `0x12`：StartSignal（车载启动信号）
- `0x20`：CompetitionCommand（树莓派到下位机的语义动作指令）
- `0x7E`：ACK
- `0x7F`：Error

## 3. 为什么第一版使用带帧JSON

JSON便于当前阶段查看字段、修改数据合同和定位问题。Magic、长度、序号和CRC已经解决串口的半包、粘包、错位和数据损坏问题。

正式确认字段和带宽后，可以保留相同消息语义，将Payload替换成紧凑二进制。CAN总线还需要增加分片或只发送控制所需的少量字段；本版本不假装已经确定CAN帧映射。

## 4. 超时安全

模拟下位机默认视觉超时为 0.30 秒。只有同时满足以下条件才允许使用：

1. 最近收到VisionOutput不超过0.30秒。
2. 顶层 `valid=true`。
3. `stream.healthy=true`。

真实下位机必须实现同样的超时保护，不能无限使用最后一次坐标。

## 5. 运行模拟

先生成Pipeline JSONL：

```powershell
python tools\run_pipeline_offline.py `
  --source "data\tag&blocks2.jpg" `
  --mode LOCALIZATION `
  --output "outputs\pipeline_localization.jsonl"
```

再模拟通信：

```powershell
python tools\simulate_communication.py `
  --jsonl "outputs\pipeline_localization.jsonl" `
  --mode FIND_BLOCKS `
  --chunk-size 7
```

`chunk-size=7`故意把数据包切成很多小块，模拟串口一次读取不到完整包。解码器仍应恢复出一个完整消息，并且CRC错误为0。

## 6. 比赛闭环消息

`RobotFeedback`当前支持以下布尔字段；未确认的字段必须发送为`false`，不能省略后由上位机猜测成功：

```json
{
  "e_stop_active": false,
  "lower_controller_healthy": true,
  "fault_detected": false,
  "at_material_zone": false,
  "at_material_tag_id": null,
  "at_build_zone": false,
  "grasp_confirmed": false,
  "cargo_stowed_confirmed": false,
  "cargo_stowed_slot_id": null,
  "cargo_retrieved_confirmed": false,
  "cargo_retrieved_slot_id": null,
  "place_pose_reached": false,
  "release_confirmed": false,
  "target_in_slot": false,
  "structure_stable": false,
  "robot_in_start_zone": false,
  "recovery_acknowledged": false
}
```

装车和取出确认必须同时携带实际完成动作的`cargo_*_slot_id`。只有它与上位机当前命令中的
`cargo_slot_id`一致，上位机才接受确认；禁止用上一动作残留的`true`推进状态机。

`CompetitionCommand`包含比赛阶段、视觉模式、运动意图、夹爪意图、目标颜色、车载槽位、
搭建槽位、材料区标签 `material_tag_id`、比赛计时和安全停车标志。运动意图是语义命令，
不是未经限幅的电机PWM。当前材料映射为紫色Tag 3、橙色Tag 4。

STM32到达材料区时应在 `RobotFeedback` 中回传：

```json
{"at_material_zone": true, "at_material_tag_id": 3}
```

`at_material_tag_id` 必须是实际到达并确认的标签ID；离开材料区后应恢复为 `null`，避免旧的
`at_material_zone=true` 让上位机误判已经到达新目标。

`communication_runtime.py`已经提供非阻塞pyserial读写、ACK、心跳和完整写入循环。实际串口名、
波特率及超时从`hardware_measurements.json`读取。正式比赛中该控制链路应为车内有线链路；无线端
只能接收监控信息。

## 7. 机械臂末端实时位姿（眼在手上夹爪相机）

当夹爪视觉需要输出机器人坐标中的三维方块位置时，STM32必须在 `RobotFeedback` 中加入：

```json
{
  "gripper_pose": {
    "valid": true,
    "sample_sequence": 1234,
    "translation_m": [0.250, -0.100, 0.350],
    "rpy_deg": [10.0, -15.0, 25.0]
  }
}
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `valid` | bool | 本周期末端位姿是否可用 |
| `sample_sequence` | uint32 | STM32每生成一个新末端姿态递增一次；重发旧数据不得递增 |
| `translation_m` | 3个有限浮点数 | `T_robot_gripper` 平移，单位m |
| `rpy_deg` | 3个有限浮点数 | Roll/Pitch/Yaw，单位deg，旋转顺序 `Rz(yaw) @ Ry(pitch) @ Rx(roll)` |

这里必须发送 `T_robot_gripper`（夹爪坐标到机器人坐标），不能发送其逆变换
`T_gripper_robot`。姿态无效时应发送：

```json
"gripper_pose": {"valid": false}
```

树莓派只在 `sample_sequence` 改变时刷新姿态接收时间。重复发送相同序号不会延长有效期。
超过 `safety.gripper_pose_timeout_s` 后，三维抓取坐标立即失效并输出 `null`。推荐 STM32
反馈频率不低于 50 Hz，超时初值 0.15 s；最终数值必须结合端到端延迟实测。序号允许
从 `0xFFFFFFFF` 回绕到0。

树莓派动态计算：

```text
T_robot_camera(q) = T_robot_gripper(q) × T_gripper_camera
```

格式错误（数组长度错误、NaN/Infinity、非布尔状态或非法序号）会收到
`ACK accepted=false`。STM32不能因为通信ACK成功就认为机械动作成功。

## 8. 尚未绑定的硬件

当前没有加入：

- RS485方向控制
- CAN ID分配
- CAN分片
- STM32 C语言解析器

这些需要控制组确认物理链路后实现。串口运行层、协议核心、CRC、序号、命令和超时逻辑现在已经
可以独立测试，但硬件安全停车仍必须由下位机独立保证。
