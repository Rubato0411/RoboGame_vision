# 三方块装车与搭建流程

默认一趟收集两块橙色和一块紫色，并遵守“最多三块、最多一块紫色”：

```text
识别/抓取
→ STOW_CARGO：按实体编号依次放入 cargo_right、cargo_left、cargo_center
→ 收满三块后 GO_TO_BUILD
→ RETRIEVE_CARGO：从指定车载位置取出一块
→ 视觉定位并放置
→ 连续稳定 3 秒
→ 取出下一块
```

装车命令示例：

```json
{
  "phase": "STOW_CARGO",
  "gripper_intent": "STOW_TO_CARGO",
  "cargo_slot_id": "cargo_right"
}
```

取出命令示例：

```json
{
  "phase": "RETRIEVE_CARGO",
  "gripper_intent": "RETRIEVE_FROM_CARGO",
  "cargo_slot_id": "cargo_right"
}
```

下位机只有在动作真正完成后才能回传：

```json
{
  "cargo_stowed_confirmed": true,
  "cargo_stowed_slot_id": "cargo_right"
}
```

```json
{
  "cargo_retrieved_confirmed": true,
  "cargo_retrieved_slot_id": "cargo_right"
}
```

反馈槽位与当前命令不匹配时，状态机不会推进。

视觉/上位机负责选择颜色、维护三槽清单、输出语义槽位 ID、放置视觉定位和稳定性判断。
机械/电控负责测量车载位置、槽位到机构轨迹的映射、防碰撞/限位以及装入和取出的真实确认。

真实位姿填写到`configs/onboard_cargo_slots.json`和
`configs/hardware_measurements.json`。未实测验证前，正式入口拒绝启动。
