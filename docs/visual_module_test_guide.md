# RoboGame 视觉模块独立调试与集成测试指南

## 1. 结论

所有面向功能的视觉模块现在都可以在不启动完整比赛程序的情况下独立验证。验证方式分为三类：

1. 带图像界面的独立程序：相机输入、双摄监视、标签、方块和巡线。
2. 命令行输出程序：统一流水线、通信、操作验证和比赛状态机模拟。
3. 单元测试：坐标数学、时序跟踪、流健康、协议分包等不适合单独开窗口的底层库。

“可独立运行”不等于“已经具备实机精度”。缺少内参、外参或实际相机时，程序会保留像素级输出，
并把不能确定的米制坐标标为`null`；正式比赛入口则会拒绝启动。

## 2. 建议测试顺序

```text
现有图片/视频
  -> 图像输入
  -> 方块 / 标签 / 巡线
  -> 时序跟踪与流健康
  -> 统一视觉流水线
  -> 状态机与通信模拟

相机到货
  -> 单摄模式与固定曝光
  -> 双摄同时运行
  -> 每台相机内参
  -> 安装外参
  -> 场地标签位姿
  -> 抓取和放置参数
  -> 树莓派性能
  -> 完整比赛程序
```

## 3. 模块独立运行对应表

| 模块 | 独立程序或测试 | 当前可验证内容 | 仍缺什么 |
|---|---|---|---|
| 图片、视频、UVC输入 | `run_viewer.py` | 解码、循环播放、相机属性、掉线重连 | 真实相机和Linux设备路径 |
| 前视/夹爪双摄 | `run_camera_monitor.py` | 双线程读取、健康状态、仪表板 | 两台实际相机、USB拓扑和供电 |
| 标定图采集 | `capture_calibration.py` | 采集流程和棋盘格可见性 | 相机、最终分辨率、最终焦距 |
| 相机内参 | `calibrate_camera.py` | 标定计算、误差和JSON输出 | 每台实机的标定照片 |
| 去畸变 | `undistort_image.py` | 标定文件与图像分辨率匹配 | 真实内参文件 |
| AprilTag识别 | `run_apriltag_demo.py` | ID 1～6、角点和时序稳定性 | 位姿输出需要内参；全场定位还需外参和标签地图 |
| 橙/紫方块识别 | `run_block_demo.py` | HSV、形状、粘连拆分、误检观察 | 新版10 cm EVA方块、正式相机与灯光数据 |
| 方块视频评估 | `tools/evaluate_video.py` | 批量检测和标注视频 | 人工GT、TP/FP/FN标注 |
| 方块参数调节 | `tools/tune_block_detector.py` | HSV、ROI和几何阈值交互调节 | 正式相机固定曝光后的数据 |
| 黑线巡线 | `run_black_line_demo.py` | 偏移、航向、交叉口和丢线 | 5 cm胶带、正式视角、斜坡和阴影数据 |
| 时序跟踪 | `tests/test_temporal_tracker.py` | 确认帧、遮挡保持、跳变拒绝 | 真实车速下的门限调试 |
| 视频流健康 | `tests/test_stream_health.py`、`run_camera_monitor.py` | 低帧率、冻结、超时、重连 | 实机超时阈值和断线测试 |
| 坐标变换 | `tests/test_coordinate_transform.py` | 坐标链、平面求交、多标签融合 | 前视/夹爪外参、场地标签实测位姿 |
| 全场定位 | `run_vision_system.py --mode LOCALIZATION` | 标签跟踪、位姿JSON和异常输出 | 前视内参、前视外参、标签地图 |
| 抓取辅助 | `run_vision_system.py --mode GRAB_ASSIST` | 橙/紫目标选择、像素误差、角度误差 | 夹爪相机内外参、夹爪中心、抓取容差 |
| 方块三维抓取点 | `tests/test_block_pose_estimator.py`、`GRAB_ASSIST` | 10 cm规则几何和平面求交 | 夹爪相机标定、支撑面高度 |
| 放置辅助 | `run_vision_system.py --mode PLACE_ASSIST` | 标签相对槽位变换、占用槽位跳过 | 搭建区参考标签及每层槽位实测值 |
| 抓取/释放/搭建证据 | `tools/run_manipulation_check.py` | 证据融合逻辑 | 接触/压力反馈和真实稳定性视觉证据 |
| 通信协议 | `tools/simulate_communication.py` | CRC、分包、粘包、ACK和超时 | 与电控组约定串口、频率和字段映射 |
| 比赛状态机 | `tools/simulate_competition_controller.py` | 启动、抓取、运输、放置、稳定计时、结束 | 真实区域到达和执行完成反馈 |
| 统一视觉流水线 | `run_vision_system.py --mode DEBUG_ALL` | 单源全部检测、JSON/协议输出 | 对应标定参数和真实数据 |
| 正式双摄比赛程序 | `run_competition_system.py` | 参数门禁、双摄路由、状态机、串口闭环 | 所有硬件实测项及电控接口 |
| 硬件参数完整性 | `tools/check_hardware_readiness.py` | 列出所有缺失和未验证项 | 填写实测数据并完成验收 |

## 4. 常用独立测试命令

以下命令均在项目根目录运行。

### 4.1 图像输入

```powershell
python run_viewer.py --source "data\tag&blocks.jpg"
python run_viewer.py --source "data\test_video.mp4" --loop
```

相机到货后：

```powershell
python run_viewer.py --source 0 --backend v4l2 --fourcc MJPG `
  --width 1280 --height 720 --fps 30
```

### 4.2 方块识别

```powershell
python run_block_demo.py --source "data\test_video.mp4" `
  --config "configs\block_detector_robust.json" --loop
```

### 4.3 AprilTag

只检测ID和角点，不要求标定：

```powershell
python run_apriltag_demo.py --source "data\tag.jpg"
```

相机位姿需要额外提供该相机的内参文件。

### 4.4 巡线

```powershell
python run_black_line_demo.py --source "data\test_video.mp4" --loop
```

### 4.5 抓取辅助

```powershell
python run_vision_system.py --source "data\tag&blocks.jpg" `
  --mode GRAB_ASSIST --target-color orange

python run_vision_system.py --source "data\tag&blocks.jpg" `
  --mode GRAB_ASSIST --target-color purple
```

无标定时可以检查目标选择和像素误差，但三维抓取点为`null`。

### 4.6 定位与放置

```powershell
python run_vision_system.py --source 0 --mode LOCALIZATION `
  --calibration "configs\front_camera_calibration.json" `
  --coordinates "configs\coordinate_frames.json"

python run_vision_system.py --source 0 --mode PLACE_ASSIST `
  --calibration "configs\front_camera_calibration.json" `
  --coordinates "configs\coordinate_frames.json"
```

这里的文件名是将来标定后生成的本地文件，目前不存在且被`.gitignore`忽略。

### 4.7 操作验证逻辑

```powershell
python tools\run_manipulation_check.py --phase VERIFY_GRASP `
  --gripper-closed --contact --target-moved-with-gripper

python tools\run_manipulation_check.py --phase VERIFY_RELEASE `
  --gripper-open --target-in-slot

python tools\run_manipulation_check.py --phase VERIFY_BUILD `
  --target-in-slot --structure-stable
```

### 4.8 比赛状态机

```powershell
python tools\simulate_competition_controller.py --compact
```

该程序不连接电机，只演示一次完整的单方块抓取和稳定放置流程。

### 4.9 通信

```powershell
python tools\simulate_communication.py `
  --jsonl "outputs\pipeline_localization.jsonl" `
  --mode FIND_BLOCKS --chunk-size 7
```

### 4.10 全部单元测试

```powershell
python -m unittest discover -s tests -v
```

## 5. 测量了什么参数，就能运行什么

| 已完成的测量/准备 | 可以新增运行的程序或功能 | 仍不能确认的内容 |
|---|---|---|
| 无硬件，仅有仓库图片/视频 | `run_viewer`、方块、标签ID、巡线、统一离线流水线、全部模拟器 | 实时性、米制坐标、真实准确率 |
| 一台相机的设备路径、分辨率、FPS、FOURCC | `run_viewer --source ...`、单摄实时方块/标签/巡线 | 位姿、距离、抓取点 |
| 固定曝光、增益、白平衡、焦距 | 可建立正式方块和巡线数据集并调参 | 米制坐标仍不可用 |
| 某台相机内参和畸变 | `undistort_image`；AprilTag相机相对位姿 | 机器人坐标和场地坐标 |
| 前视相机安装外参 | 前视像素到机器人平面坐标 | 全场定位仍缺标签地图 |
| 6个标签场地位姿 | `LOCALIZATION`全场位姿、多标签融合 | 取决于现场测量精度 |
| 夹爪相机内参和安装外参 | 方块中心、顶面抓取点 | 精确对准仍缺夹爪中心和容差 |
| 夹爪中心像素和位置/角度容差 | `GRAB_ASSIST`完整视觉输出 | 实际闭合和抓取成功由电控反馈确认 |
| 机械臂末端位姿与固定Tag同步样本 | `calibrate_hand_eye.py`求解 `T_gripper_camera` | 动态机器人坐标仍需实时末端位姿 |
| 搭建区参考标签和槽位变换 | `PLACE_ASSIST`、占用槽位跳过 | 机械臂是否能到达由机械/电控验证 |
| 两台相机同时运行模式和USB验证 | 双摄监视和正式程序的视觉部分 | 树莓派性能仍需实测 |
| 串口设备、波特率、心跳和命令超时 | 视觉端真实通信运行层 | 电机和执行器动作由电控端实现 |
| 树莓派性能与视觉验收指标全部通过 | 可把硬件配置状态改为`MEASURED_AND_VERIFIED`并启动正式入口 | 仍需整车联调和赛场验证 |

## 6. 视觉组与电控组职责边界

### 视觉组负责

- 图像、视频和相机帧的可靠输入；
- 相机内参、畸变和相机安装外参；
- AprilTag、方块和黑线检测；
- 时序跟踪、置信度、时间戳和流健康；
- 机器人场地位姿和视觉目标机器人坐标；
- 橙/紫目标选择；
- 夹爪像素偏差、角度偏差和视觉抓取点；
- 搭建槽位视觉位姿和占用状态；
- 目标是否随夹爪移动、是否进入槽位、建筑是否视觉稳定等视觉证据；
- 向电控端发送带`valid/confidence/timestamp`的视觉结果和语义动作请求；
- 在视觉失效、相机冻结或结果超时时发送安全停车请求。

### 电控/控制组负责

- 电机、舵机、气动和夹爪的实际驱动；
- 底盘运动学、编码器、IMU和闭环PID；
- 把`GO_TO_MATERIAL`、`ALIGN_TO_BLOCK`、`CLOSE`等语义请求转换为实际运动；
- 速度、加速度、电流、行程和机械限位；
- 抓取接触、压力、电流、夹爪开合等非视觉传感器；
- 返回区域到达、运动完成、抓取确认和释放确认；
- 物理急停直接切断所有执行器；
- 视觉/树莓派失联后的独立硬件看门狗和安全停车；
- 电源、电池、通信电气接口和执行器安全。

### 双方共同约定，但不由视觉组单独完成

- 机器人坐标系和正方向；
- 串口消息字段、单位、频率和超时；
- “到达材料区/搭建区”的判定来源；
- 抓取、释放和异常复位的握手顺序；
- 视觉误差到运动控制量之间的限幅；
- 整车端到端延迟和失效测试。

视觉端提供的是测量和语义决策，不应直接输出未经电控限幅的PWM或电机电流。电控端也不应在
`valid=false`、数据超时或视觉流异常时继续使用旧坐标。

## 7. 当前明确缺失项

这些不是代码遗漏，而是必须等硬件或现场条件具备后完成：

- 两台相机的真实设备信息、内参和安装外参；
- 新版10 cm EVA方块的正式数据集和准确率；
- 5 cm胶带在斜坡、阴影和遮挡条件下的巡线门限；
- 6个标签的场地坐标；
- 搭建槽位三维位置；
- 夹爪中心、接近高度、抓取和释放容差；
- 树莓派双摄性能、散热和长时间稳定性；
- 电控端反馈字段和真实串口参数；
- 整车安全停车及异常复位测试。
