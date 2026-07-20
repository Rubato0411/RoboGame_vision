# 硬件到货前代码完善变更说明

## 回滚基线

- Git 基线：`a8a8a3b`。
- 开发分支：`codex-hardware-independent-completion`。
- 额外源码快照：`backups/pre_hardware_independent_completion_20260720.zip`（本地忽略，不上传）。

在本分支工作期间，可使用 `git diff a8a8a3b` 查看全部变更。需要完全回到修改前时，切换到
`main` 即可；若以后将变更合并到 `main`，应使用 `git revert <变更提交>` 生成可审计的反向提交，
不要使用 `git reset --hard`。

## 新增模块

- `competition_controller.py`：比赛阶段状态机、规则载荷保护、橙/紫策略、异常复位、6分钟结束和
  连续3秒稳定判定。
- `competition_runtime.py`：连接双摄视觉、状态机、机器人反馈和有线通信；抓取使用夹爪相机，巡线、
  标签定位及放置使用前视相机。
- `communication_runtime.py`：真实非阻塞串口适配、ACK、心跳、视觉与比赛指令发布。
- `hardware_config.py`：硬件参数完整性检查和正式启动门禁。
- `run_competition_system.py`：正式比赛入口；硬件未测时明确拒绝启动。
- `check_hardware_readiness.py`：列出所有未填写的实测项。

## 修改模块

- `vision_pipeline.py`：支持动态目标颜色、已占用搭建槽位，并接入10 cm方块三维抓取估计。
- `target_selector.py`：从固定橙块改为运行时请求橙块或紫块。
- `placement_tag_locator.py`：启用槽位前强制检查实测值。
- `communication_protocol.py` / `raspberry_pi_endpoint.py`：新增机器人反馈、启动信号和比赛指令。
- `black_line_detector.py`：拒绝落在图像外的拟合转向点，避免输出越界控制量。
- `block_pose_estimator.py`：默认方块尺寸从旧值0.15 m改为规则值0.10 m。
- `vision_output.py`：方块输出增加抓取点和图像角度。
- `run_vision_system.py`：增加可选真实串口、心跳和紫块选择参数，保留原离线使用方式。

## 配置变化

- `competition_rules.json`：集中保存规则确定值。
- `competition_strategy.json`：保存硬件无关的保守策略。
- `hardware_measurements.json`：唯一硬件实测参数整合文件，初始全部未测。
- `placement_slots.json`：替换旧15 cm示例，建立三座建筑的禁用测量槽位，禁止零值伪装成标定。

## 测试变化

新增规则计分、载荷限制、完整抓放循环、稳定计时、急停、比赛超时、紫块选择、硬件门禁及通信
运行测试。原有测试继续保留。
