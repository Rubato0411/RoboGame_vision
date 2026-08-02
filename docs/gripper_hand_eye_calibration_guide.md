# 夹爪相机眼在手上手眼标定

## 输出和坐标约定

夹爪相机随末端运动，使用眼在手上（eye-in-hand）模型。工具求解：

```text
T_gripper_camera
```

它把相机坐标转换到夹爪/末端坐标。运行时某一关节姿态 `q` 下：

```text
T_base_camera(q) = T_base_gripper(q) × T_gripper_camera
```

机器人控制器提供的必须是 `T_base_gripper`，不能把 `T_gripper_base` 直接填入。
AprilTag 检测器提供的是 `T_camera_target`。标定期间目标 Tag 必须固定不动。

## 前提

- 夹爪相机内参已经完成，分辨率和正式运行一致。
- 焦距、曝光、白平衡以及相机支架全部固定。
- STM32/机械臂能在静止后给出同一时刻的末端 `X Y Z Roll Pitch Yaw`。
- 明确 STM32 的姿态单位、欧拉角顺序以及末端坐标原点。
- 固定一个平整 AprilTag，尺寸配置与打印实物一致。
- 倒置相机继续使用 AprilTag 配置中的 `pose_image_rotation_deg=180`，不要再修改 Tag 实物姿态补偿。

## 采集姿态

建议采集 15～20 组，最低 5 组。每组都让机械臂完全停止，再读取末端位姿并采集图像。
Tag 在整个过程中绝对不能移动。姿态要同时覆盖：

- 左、右、上、下、近、远；
- 绕至少两个旋转轴的正负倾角；
- 总平移跨度尽量超过 0.10 m；
- 总旋转跨度尽量超过 40°；
- Tag 保持完整、清晰且不过度斜视。

只平移不旋转、所有姿态集中在一个小区域，或在机械臂运动过程中采样，都会得到不可靠结果。

## 树莓派逐姿态采样

以下例子假设 `camera0=gripper`，固定目标使用 Tag 6。命令中的末端位姿只是格式示例，
每次必须替换为 STM32 在该静止姿态下给出的真实 `T_base_gripper`，长度单位 m、角度单位 deg：

```bash
cd /home/robogame26/vision_part/RoboGame_vision
mkdir -p data/hand_eye configs/calibration_results

python tools/capture_hand_eye_sample.py \
  --source 0 --backend picamera2 \
  --width 1280 --height 720 --fps 30 \
  --calibration configs/calibration_gripper.json \
  --tag-config configs/apriltag_detector.json \
  --tag-id 6 \
  --base-gripper-xyz 0.250 -0.100 0.350 \
  --base-gripper-rpy 10.0 -15.0 25.0 \
  --sample-id pose_01 \
  --frames 30 \
  --samples data/hand_eye/gripper_samples.json
```

移动到下一姿态、等待完全静止、读取新的末端位姿，然后把 `sample-id` 改为 `pose_02`，
重复同一命令。工具会对 30 个有效 Tag 位姿取稳健平均并追加一组配对数据。

如果 STM32 输出的是毫米，必须先除以 1000。如果输出的是 `T_gripper_base`，必须先求逆；
不能仅对平移取负号来代替刚体变换求逆。

## 求解

```bash
python tools/calibrate_hand_eye.py \
  --samples data/hand_eye/gripper_samples.json \
  --method PARK \
  --output configs/calibration_results/hand_eye_gripper.json
```

默认推荐先用 `PARK`。也可以分别运行 `TSAI`、`HORAUD` 比较结果，但不能只选择数值看起来最顺眼的一组。
输出包含 `gripper_from_camera.translation_m`、`rpy_deg`、4×4 矩阵以及固定目标一致性残差。

初步质量参考：

- `fixed_target_translation_rms_m` 尽量小于 0.005～0.010 m；
- `fixed_target_rotation_rms_deg` 尽量小于 1～2°；
- 不同算法结果不应出现明显方向翻转或数厘米差异；
- 结果必须与卷尺测得的相机相对夹爪大致位置和朝向一致。

这些只是初期建议，不替代实际抓取精度验收。

## 独立验证

标定后另外采集 5 组未参与求解的姿态。每组计算：

```text
T_base_target(i) = T_base_gripper(i) × T_gripper_camera × T_camera_target(i)
```

固定 Tag 的 `T_base_target` 应在各组之间保持一致。随后必须低速验证机器人坐标中的
目标方向和距离；第一次闭环运动要限速、限步长并准备急停。

## 正式运行接入

`hand_eye_gripper.json` 输出的是静态 `T_gripper_camera`，不能直接伪装成固定的
`T_robot_camera`。正式比赛入口已经通过 STM32 `RobotFeedback.gripper_pose` 实时接收
`T_robot_gripper(q)`，并在每个夹爪视觉周期计算：

```text
T_robot_camera(q) = T_robot_gripper(q) × T_gripper_camera
```

在 `hardware_measurements.json` 中填写 `cameras.gripper.hand_eye_calibration_file` 和
`safety.gripper_pose_timeout_s`。姿态无效、采样序号没有更新或超过时限时，二维对准仍可
输出，但 `blocks[].position_robot_m` 和 `grasp_point_robot_m` 保持 `null`，不得用于三维运动。
