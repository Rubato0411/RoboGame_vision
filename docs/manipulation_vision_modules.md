# 抓取与Tag搭建区视觉模块

## 已新增模块

| 模块 | 作用 |
|---|---|
| `block_pose_estimator.py` | 从方块底部接触像素和支撑平面计算方块中心、顶部抓取点 |
| `target_selector.py` | 按颜色、置信度、距离、画面位置和预测状态选择目标 |
| `gripper_alignment.py` | 输出方块相对夹爪参考点的像素偏差、角度偏差和 `aligned` |
| `placement_tag_locator.py` | 根据搭建区AprilTag和槽位相对Tag的变换计算机器人坐标系放置位姿 |
| `manipulation_verifier.py` | 融合视觉移动、夹爪闭合、接触/压力和槽位占用，验证抓取与释放 |
| `multi_camera_manager.py` | 独立管理前视和夹爪相机输入 |

## 新工作模式

- `GRAB_ASSIST`：方块检测、目标选择和夹爪对准。
- `PLACE_ASSIST`：AprilTag检测并定位下一个未占用搭建槽位。
- `SAFE_STOP`：只保留健康输出，不产生运动目标。

## 搭建区Tag不是放置点本身

Tag提供稳定坐标系。每个建筑槽位仍要测量 `T_tag_slot`。在
`configs/placement_slots.json` 中填写搭建区Tag ID、Tag中心到槽位中心的X/Y/Z、
槽位相对Tag的Roll/Pitch/Yaw、层数和优先级。未测量时必须保持
`configured=false`。

## 夹爪参考点

`configs/gripper_alignment.json` 中的 `grasp_reference_px` 是夹爪真正闭合中心在
图像中的像素，不一定是画面中心。相机安装后要用标记物测量。

夹爪相机倒置 180° 安装时，将同一文件中的 `image_rotation_deg` 设为 `180`。
检测、相机内参和 `grasp_reference_px` 仍使用相机输出的原始倒置画面；对准模块只在
输出阶段同时反转 `dx_px` 和 `dy_px`，使其方向等价于将画面转正后的图像坐标。
`0` 表示不修正，其他角度会被配置校验拒绝。方块角度采用 90° 对称周期，因此 180°
旋转不改变 `angle_error_deg`。该设置只修正视觉坐标方向，视觉误差到机械臂各轴运动的
映射仍需由视觉组和电控组用低速小步运动共同确认。

## 验证原则

抓取成功默认要求：夹爪闭合、接触/压力有效、方块随夹爪移动。释放成功要求：
夹爪张开、接触消失、方块位于目标槽位。传感器数据由下位机采集并交给任务状态机。

## 硬件后填写

- 两台相机各自内参和外参
- 方块真实边长、支撑面高度
- 夹爪参考像素与允许误差
- 每个搭建槽位的Tag相对位姿
- 压力、接触、闭合、张开传感器阈值
