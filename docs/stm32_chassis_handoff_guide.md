# 树莓派视觉与 STM32 底盘运动电控交付指南

## 1. 交付目标与责任边界

本文件是视觉组交给底盘运动电控组的接口合同和联调步骤。双方边界如下。

| 树莓派视觉/任务层 | STM32 底盘运动电控 |
|---|---|
| 前视相机取流、AprilTag 定位、巡线检测 | 电机驱动、编码器、IMU、里程计 |
| 输出场地坐标、航向角、巡线偏差和置信度 | 底盘运动学、速度环、位置环、PID |
| 输出 `STOP/HOLD/GO_TO_*` 等语义意图 | 将意图和视觉量转换为限幅后的底盘运动 |
| 比赛阶段和目标选择 | 到区判定、运动完成判定和故障诊断 |
| 发现断流时标记视觉结果无效 | 急停、通信超时和失效安全停车 |

视觉组不负责电机 PWM、编码器/IMU 驱动、底盘 PID、硬件急停和执行机构
安全。STM32 必须拥有独立于树莓派的停车权限。

## 2. 交付文件

| 文件 | 用途 |
|---|---|
| `src/communication_protocol.py` | 协议帧、消息类型、CRC32 的权威实现 |
| `src/raspberry_pi_endpoint.py` | 树莓派端收发语义和 ACK |
| `src/communication_runtime.py` | 非阻塞串口运行层和心跳 |
| `src/vision_output.py` | `VisionOutput` 数据字段 |
| `src/competition_controller.py` | 比赛阶段、运动意图和反馈字段 |
| `src/simulated_lower_controller.py` | STM32 行为的 Python 参考模型 |
| `docs/communication_protocol_v1.md` | 协议 v1 说明 |
| `docs/vision_output_contract.md` | 视觉输出合同 |
| 本文件 | STM32 实现和联调指南 |

STM32 端需要由电控组实现 C/C++ 增量解析器；Python 文件只作为协议权威实现，
不能直接部署到 STM32。

## 3. 联调前必须共同冻结的参数

以下字段目前在 `configs/hardware_measurements.json` 中仍为空，未确认前不得开始
带电闭环：

| 参数 | 负责人 | 说明 |
|---|---|---|
| 物理链路 | 双方 | 推荐车内有线 UART；如用 RS485，还需方向控制 |
| 树莓派设备 | 视觉组 | 例如 `/dev/serial0` 或 USB 串口稳定路径 |
| UART 电平 | 电控组 | 必须确认 3.3 V TTL，不得将 RS232 电平直接接入 |
| 波特率 | 双方 | 完整 JSON 不建议使用 115200；应测带宽后确定 |
| 视觉发送频率 | 双方 | 不等于相机帧率，建议先从 10 Hz 联调 |
| 电控反馈频率 | 电控组 | 建议至少 20 Hz，最终按闭环需求确认 |
| 视觉超时 | 电控组 | 建议初值 0.30 s，必须实测 |
| 指令超时 | 电控组 | 建议初值 0.30 s，必须实测 |
| 最大速度/加速度 | 电控组 | 由底盘和比赛安全要求确定 |
| 到区容差 | 双方 | `at_material_zone`、`at_build_zone` 的判定阈值 |

协议 v1 每周期发送完整 JSON。若一周期共 1500 字节、发送 30 Hz，则仅数据就约
45 kB/s；8N1 UART 至少需要约 450 kbit/s，尚未计入余量。因此有三种可选方案：

1. 使用 921600 波特率并实测稳定性；
2. 将视觉/命令发送频率限制为 10 Hz 左右；
3. 双方冻结最小字段后增加紧凑二进制控制消息。

正式比赛前必须选择一种，不能默认 115200 能承载完整 30 Hz JSON。

## 4. 字节帧格式

所有多字节整数均为小端序。

| 偏移 | 字段 | 长度 | 类型/说明 |
|---:|---|---:|---|
| 0 | Magic | 2 | 固定 ASCII `RG`，即 `0x52 0x47` |
| 2 | Version | 1 | 当前为 `1` |
| 3 | Message Type | 1 | 见消息类型表 |
| 4 | Sequence | 4 | `uint32_t`，小端递增序号 |
| 8 | Payload Length | 2 | `uint16_t`，JSON UTF-8 字节数 |
| 10 | Payload | N | 紧凑 UTF-8 JSON |
| 10+N | CRC32 | 4 | 从 Magic 到 Payload 末尾 |

CRC 使用与 zlib `crc32` 相同的 IEEE CRC-32，结果按小端 `uint32_t` 放入帧尾。
禁止直接用未打包的 C 结构体覆盖接收字节，以免受到对齐和大小端影响。

### 消息类型

| 值 | 名称 | 方向 |
|---:|---|---|
| `0x01` | VisionOutput | 树莓派 → STM32 |
| `0x02` | Heartbeat | 双向 |
| `0x10` | ModeCommand | STM32 → 树莓派 |
| `0x11` | RobotFeedback | STM32 → 树莓派 |
| `0x12` | StartSignal | STM32 → 树莓派 |
| `0x20` | CompetitionCommand | 树莓派 → STM32 |
| `0x7E` | ACK | 双向 |
| `0x7F` | Error | 双向预留 |

## 5. STM32 增量解析器要求

串口一次接收的数据不保证是一整帧。解析器必须支持半包、粘包、噪声和错位恢复：

1. 将 DMA/中断收到的字节写入环形缓冲区；
2. 搜索连续的 `0x52 0x47`；
3. 缓冲区不足 10 字节时等待；
4. 检查版本、消息类型和 Payload Length；
5. Payload Length 超过约定上限时丢弃一个字节并重新找 Magic；
6. 等待 `10 + Payload Length + 4` 字节完整到达；
7. 计算 CRC32；
8. CRC 错误时不得执行负载内容，并重新同步；
9. CRC 正确后才解析 JSON；
10. 保存序号和本地接收时间；
11. 未识别字段应忽略，以便协议向后兼容；
12. 不得因一帧错误阻塞后续帧。

STM32 至少记录：

```text
decoded_packets
crc_errors
format_errors
discarded_bytes
last_valid_sequence
last_valid_receive_time
```

## 6. 树莓派下发给底盘的数据

### 6.1 CompetitionCommand（0x20）

示例：

```json
{
  "phase": "NAVIGATE_TO_MATERIAL",
  "vision_mode": "FOLLOW_LINE",
  "motion_intent": "GO_TO_MATERIAL",
  "gripper_intent": "HOLD",
  "desired_block_color": null,
  "cargo_slot_id": null,
  "placement_slot_id": null,
  "safe_stop": false,
  "match_elapsed_s": 12.4,
  "reason": "following the marked route to a material zone"
}
```

底盘主要使用：

| 字段 | 要求 |
|---|---|
| `motion_intent` | 选择底盘状态 |
| `safe_stop` | `true` 时无条件停车，优先级最高 |
| `phase` | 诊断和状态一致性检查，不直接代替安全条件 |
| `match_elapsed_s` | 显示/日志，不作为 STM32 唯一比赛计时源 |

### 6.2 底盘运动意图

| motion_intent | STM32 行为 |
|---|---|
| `STOP` | 立即进入安全停车；不得等待视觉坐标 |
| `HOLD` | 保持/刹停，不执行新的平移目标 |
| `GO_TO_MATERIAL` | 按路线前往材料区 |
| `SEARCH_BLOCK` | 执行双方约定的低速搜索动作 |
| `ALIGN_TO_BLOCK` | 按抓取对准量低速调整底盘 |
| `GO_TO_BUILD` | 按路线前往搭建区 |
| `SEARCH_PLACE` | 执行双方约定的低速放置搜索动作 |
| `ALIGN_TO_PLACE` | 按放置目标低速对准 |

这些是语义意图，不是 PWM，也不是 `vx/vy/omega`。采用当前架构时，由 STM32
负责限速、轨迹、运动学和闭环。若电控要求树莓派直接给速度，必须另行冻结
`vx_mps/vy_mps/omega_radps` 消息，不能自行猜测单位或范围。

### 6.3 VisionOutput（0x01）

定位控制最小关注字段：

```json
{
  "schema_version": "1.2",
  "valid": true,
  "frame_id": 123,
  "timestamp_s": 15.42,
  "stream": {
    "healthy": true,
    "frame_age_s": 0.03
  },
  "robot_pose": {
    "valid": true,
    "position_field_m": [1.20, -0.50, 0.0],
    "rpy_field_deg": [0.0, 0.0, 35.0],
    "confidence": 0.91,
    "source_tag_ids": [1, 2]
  },
  "line": {
    "valid": true,
    "lateral_offset_normalized": -0.12,
    "heading_error_deg": 4.3,
    "confidence": 0.88
  }
}
```

单位和方向：

| 数据 | 单位/约定 |
|---|---|
| `position_field_m` | 米，场地右手系 |
| `rpy_field_deg[2]` | 偏航角，度 |
| `lateral_offset_normalized` | 归一化横向偏差 |
| `heading_error_deg` | 度 |
| 像素原点 | 左上，X 向右、Y 向下 |

定位可用条件必须全部满足：

```text
VisionOutput.valid == true
stream.healthy == true
robot_pose.valid == true
本地接收年龄 <= vision_timeout_s
序号是可接受的新数据
```

巡线可用条件必须全部满足：

```text
VisionOutput.valid == true
stream.healthy == true
line.valid == true
本地接收年龄 <= vision_timeout_s
```

顶层 `valid=true` 只代表本帧和视频流没有全局错误，不代表当前一定看见 Tag 或
黑线。STM32 不得只检查顶层 `valid`。

## 7. STM32 回传给树莓派的数据

### 7.1 RobotFeedback（0x11）

完整字段如下；未确认的状态必须显式发送 `false`，不得让树莓派猜测动作完成：

```json
{
  "e_stop_active": false,
  "lower_controller_healthy": true,
  "fault_detected": false,
  "at_material_zone": false,
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

纯底盘负责人至少负责：

```text
e_stop_active
lower_controller_healthy
fault_detected
at_material_zone
at_build_zone
robot_in_start_zone
```

机械臂/夹爪字段可以由其他电控模块产生，但最终应通过统一反馈送达树莓派。

`cargo_stowed_confirmed` 和 `cargo_retrieved_confirmed` 为 `true` 时，必须同时携带
真实完成动作的槽位 ID：

```text
cargo_right
cargo_left
cargo_center
```

禁止沿用上一动作残留的 `true`。建议每个动作完成事件只保持一个反馈周期或使用
明确的动作序号去重；当前树莓派会检查槽位 ID 是否与当前命令一致。

### 7.2 StartSignal（0x12）

车载启动按钮触发后发送：

```json
{"active":true}
```

树莓派会锁存启动状态。急停和故障状态不得通过 StartSignal 表达。

### 7.3 ModeCommand（0x10）

独立视觉调试时可由 STM32 请求：

```json
{"mode":"LOCALIZATION","request_id":42}
```

可选模式：

```text
IDLE
LOCALIZATION
FIND_BLOCKS
FOLLOW_LINE
GRAB_ASSIST
PLACE_ASSIST
SAFE_STOP
DEBUG_ALL
```

正式比赛中视觉模式主要由树莓派比赛状态机决定；电控不得在未约定时持续覆盖。

## 8. ACK 和序号

树莓派收到有效的 ModeCommand、RobotFeedback、StartSignal 或 Heartbeat 后返回：

```json
{
  "ack_sequence": 42,
  "accepted": true,
  "detail": "robot_feedback"
}
```

ACK 帧自身也有独立发送序号。`ack_sequence` 指向被确认帧的序号。

STM32 发送序号从任意 `uint32_t` 起点递增，溢出后回到 0。电控不得用普通有符号
大小比较处理回绕。控制量是否新鲜以“本地接收时间”为主，序号用于查重和诊断。

## 9. 安全状态机最低要求

STM32 中建议至少具有：

```text
BOOT_SAFE
WAIT_LINK
READY_HOLD
EXECUTING
VISION_LOST
FAULT_STOP
ESTOP_LATCHED
```

任何状态下满足以下任一条件都不得继续使用旧视觉量：

- `safe_stop=true`；
- `motion_intent=STOP`；
- 急停有效；
- 下位机自身故障；
- VisionOutput 超时；
- 对应子结果 `valid=false`；
- CRC 连续错误超过阈值；
- 树莓派心跳/命令超时；
- 序号长期不更新；
- 数据出现 NaN、无穷大或超出物理范围。

急停必须由 STM32 和硬件回路独立执行。树莓派崩溃、CSI 断流、串口拔出或 JSON
解析失败时，底盘必须自行进入安全停车。

## 10. 协议黄金测试向量

以下十六进制由当前 Python 权威实现生成，可用于 STM32 单元测试。

### 树莓派心跳

消息：类型 `0x02`，序号 1：

```json
{"role":"raspberry_pi","monotonic_s":1.25,"mode":"IDLE"}
```

完整帧 70 字节：

```text
524701020100000038007b22726f6c65223a227261737062657272795f7069222c226d6f6e6f746f6e69635f73223a312e32352c226d6f6465223a2249444c45227dc1a5c276
```

### 定位模式命令

消息：类型 `0x10`，序号 2：

```json
{"mode":"LOCALIZATION","request_id":2}
```

完整帧 52 字节：

```text
524701100200000026007b226d6f6465223a224c4f43414c495a4154494f4e222c22726571756573745f6964223a327d3e47b47b
```

### 最小安全反馈

消息：类型 `0x11`，序号 3：

```json
{"e_stop_active":false,"lower_controller_healthy":true,"fault_detected":false,"at_material_zone":false,"at_build_zone":false,"robot_in_start_zone":false}
```

完整帧 167 字节：

```text
524701110300000099007b22655f73746f705f616374697665223a66616c73652c226c6f7765725f636f6e74726f6c6c65725f6865616c746879223a747275652c226661756c745f6465746563746564223a66616c73652c2261745f6d6174657269616c5f7a6f6e65223a66616c73652c2261745f6275696c645f7a6f6e65223a66616c73652c22726f626f745f696e5f73746172745f7a6f6e65223a66616c73657d1d4a23fa
```

验收要求：STM32 应能将测试向量分成任意 1～7 字节片段输入解析器，最终仍只解析
出一帧；修改任一 Payload 字节后必须报告 CRC 错误且不得执行。

## 11. 分阶段操作指南

### 阶段 A：STM32 纯软件单元测试

不连接电机，仅测试：

1. 三个黄金向量；
2. 每字节输入；
3. 7 字节随机分片；
4. 两帧粘连；
5. Magic 前插入噪声；
6. Payload 被破坏；
7. CRC 被破坏；
8. `uint32_t` 序号回绕；
9. 超长 Payload 拒绝；
10. 未知 JSON 字段忽略。

### 阶段 B：树莓派协议回环测试

在树莓派仓库执行：

```bash
cd /home/robogame26/vision_part/RoboGame_vision
source .venv/bin/activate

python tools/simulate_communication.py \
  --jsonl captures/front_acceptance/direction_base_corner_fixed.jsonl \
  --mode LOCALIZATION \
  --chunk-size 7
```

至少确认：

```text
pi_mode=LOCALIZATION
vision_usable_at_0.2s=True
vision_usable_at_0.5s=False
crc_errors=0
```

此阶段只验证 Python 协议核心，不代表真实 UART 已完成。

### 阶段 C：真实 UART、不接电机

1. 共地并确认电平；
2. TX/RX 交叉连接；
3. 确认树莓派串口未被登录控制台占用；
4. 双方设置完全相同的波特率、8N1、无硬件流控；
5. STM32 周期发送 Heartbeat 和 RobotFeedback；
6. 树莓派应返回 ACK；
7. STM32 接收树莓派 Heartbeat、VisionOutput、CompetitionCommand；
8. 连续记录 10 分钟 CRC、丢包、最大延迟和缓冲区峰值；
9. 拔掉 RX、TX、停止 Python 进程，验证超时状态；
10. 此阶段所有电机输出保持禁止。

树莓派识别串口：

```bash
ls -l /dev/serial0 /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

确认端口占用：

```bash
sudo lsof /dev/serial0
```

串口和波特率实测后填写：

```json
"lower_controller": {
  "verified": false,
  "transport": "uart",
  "device": "/dev/serial0",
  "baudrate": 921600,
  "protocol_version": 1,
  "feedback_rate_hz": 20,
  "command_rate_hz": 10
}
```

以上数字只是联调候选值，必须由双方确认后才可将 `verified` 改为 `true`。

### 阶段 D：架空轮和低速地面测试

1. 先单独验证 `STOP`；
2. 验证通信中断停车；
3. 验证急停；
4. 验证 `HOLD`；
5. 低速验证巡线偏差正负方向；
6. 低速验证航向误差正负方向；
7. 验证 `GO_TO_MATERIAL` 和 `GO_TO_BUILD`；
8. 验证到区后回传只在真实到达时置位；
9. 设置速度、加速度、角速度和转向限幅；
10. 记录端到端 P95 延迟。

### 阶段 E：整车验收

必须逐项制造故障：

- 遮挡全部 Tag；
- 黑线离开视野；
- CSI 相机断流；
- 停止树莓派视觉进程；
- 拔掉串口；
- 注入 CRC 错误；
- STM32 停止反馈；
- 急停；
- 重启树莓派；
- 重启 STM32。

每项都应记录“触发时间、停车时间、最大移动距离、恢复条件”。安全停车测试未通过前，
不得进行高速自主运动。

## 12. 交接验收表

| 项目 | 负责人 | 结果 |
|---|---|---|
| 坐标系 X 前、Y 左、Z 上已签字确认 | 双方 |  |
| 偏航正方向已确认 | 双方 |  |
| 串口电平和接线已确认 | 电控 |  |
| 设备路径、波特率、8N1 已确认 | 双方 |  |
| CRC 黄金向量通过 | 电控 |  |
| 半包、粘包、噪声恢复通过 | 电控 |  |
| RobotFeedback 字段通过 | 电控 |  |
| ACK 和序号通过 | 双方 |  |
| 定位 X/Y/Yaw 正负方向通过 | 双方 |  |
| 巡线偏差和角度正负方向通过 | 双方 |  |
| 视觉超时停车通过 | 电控 |  |
| 树莓派进程退出停车通过 | 电控 |  |
| 串口拔出停车通过 | 电控 |  |
| 急停独立工作 | 电控 |  |
| 最大速度/加速度已限幅 | 电控 |  |
| 端到端 P95 延迟已记录 | 双方 |  |
| 连续运行 60 分钟无异常 | 双方 |  |

只有双方将表中涉及自身的项目完成并保存测试记录，才能把树莓派—STM32底盘链路标记为
`MEASURED_AND_VERIFIED`。
