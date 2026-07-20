# 统一视觉输出数据合同 v1.2

## 边界

`vision_pipeline.py` 接收一个 `FramePacket`，输出一个 `VisionOutput`。它不打开摄像头、不创建窗口、不发送串口或 CAN，也不负责比赛主循环。

```text
FramePacket + VisionMode
        ↓
VisionPipeline.process()
        ↓
VisionOutput
        ├─ to_dict()
        └─ to_json()
```

## 模式

- `IDLE`：只检查视频流，不运行目标算法。
- `LOCALIZATION`：AprilTag检测、连续跟踪和可选全局定位。
- `FIND_BLOCKS`：方块检测、连续跟踪和可选机器人坐标转换。
- `FOLLOW_LINE`：黑线检测。
- `GRAB_ASSIST`：按请求的橙/紫颜色选择方块并输出夹爪对准量。
- `PLACE_ASSIST`：根据已测槽位和占用状态输出下一放置目标。
- `SAFE_STOP`：不运行目标算法，只保留流健康检查。
- `DEBUG_ALL`：同时运行所有算法，仅用于调试和性能测试。

## 单位和约定

- 距离：m
- 角度：deg
- 时间：s
- 处理耗时：ms
- 像素原点：图像左上角
- 像素 X：向右
- 像素 Y：向下
- `null`：该数值当前不可用
- `valid=false`：下游不得使用该结果控制机器人
- `predicted=true`：短时遮挡期间由跟踪器保留，并非当前帧直接检测

## 顶层字段

```json
{
  "schema_version": "1.2",
  "valid": true,
  "frame_id": 123,
  "timestamp_s": 15.42,
  "source_name": "camera:0",
  "mode": "FIND_BLOCKS",
  "stream": {},
  "robot_pose": {},
  "tags": [],
  "blocks": [],
  "line": {},
  "processing": {},
  "errors": []
}
```

通信层以后只能编码这些公开字段，不应直接序列化检测器内部对象。协议增加字段时应提升 schema 版本并保持兼容。

v1.2在每个`blocks[]`元素中新增：

- `grasp_point_robot_m`：规则尺寸和支撑平面约束得到的方块顶面抓取点；未标定时为`null`。
- `yaw_image_deg`：方块在图像中的旋转矩形角度，仅作对准初值。

## 安全规则

视频流为 `FROZEN`、`STALE_FRAME`、`INVALID_FRAME`、`TIMEOUT` 或 `DISCONNECTED` 时：

1. 清空目标跟踪器。
2. 不运行目标算法。
3. 顶层 `valid=false`。
4. 在 `errors` 中写明原因。

坐标参数未配置时，像素检测仍可输出，但米制坐标必须为 `null`，不能使用全零假坐标。
