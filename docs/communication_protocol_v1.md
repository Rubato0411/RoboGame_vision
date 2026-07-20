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

## 6. 尚未绑定的硬件

当前没有加入：

- 串口名称和波特率
- pyserial读写循环
- RS485方向控制
- CAN ID分配
- CAN分片
- STM32 C语言解析器

这些需要控制组确认物理链路后实现。协议核心、CRC、序号、命令和超时逻辑现在已经可以独立测试。
