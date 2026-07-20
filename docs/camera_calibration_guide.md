# 相机标定使用指南

## 棋盘格参数

本项目默认：

```text
内角点：9列 × 6行
方格边长：打印后实测填写，示例25 mm
```

9×6指内角点数量，对应10×7个黑白方格。程序中的`--square-mm`必须填写打印后用尺或游标卡尺测得的单个方格边长。

生成棋盘格：

```powershell
python tools\generate_chessboard.py --columns 9 --rows 6 --output data\calibration\chessboard_9x6.png
```

打印时：

- 使用100%或实际尺寸打印；
- 关闭适应纸张、缩放到页面；
- 贴在平整刚性板上；
- 使用哑光表面，避免反光；
- 打印后重新测量方格，不能只相信设计尺寸。

## 当前无真实相机时

可以用笔记本摄像头练习采集：

```powershell
python capture_calibration.py --source 0 `
  --output-dir data\calibration\laptop `
  --columns 9 --rows 6 --square-mm 25 `
  --backend dshow --fourcc MJPG --width 1280 --height 720 --fps 30
```

画面检测到角点后按空格保存，按Q退出。

这套标定结果只能用于当前笔记本摄像头和当前分辨率，不能用于以后购买的机器人相机。

## 正式相机采集要求

每台摄像头单独采集30～50张。图像必须覆盖：

- 中央；
- 左上、右上、左下、右下；
- 靠近四条边缘；
- 近距离和远距离；
- 水平旋转；
- 上下倾斜；
- 左右倾斜。

避免：

- 连续保存大量几乎相同的姿态；
- 运动模糊；
- 棋盘格出画；
- 严重反光；
- 棋盘纸弯曲；
- 标定过程中改变焦距、分辨率或镜头。

角点应尽量覆盖整个图像，否则边缘畸变估计会很差。

## 执行标定

```powershell
python calibrate_camera.py `
  --input data\calibration\front_camera `
  --output configs\camera_front_calibration.json `
  --columns 9 --rows 6 --square-mm 25 `
  --min-views 15 `
  --max-view-error 1.5 `
  --preview-dir data\calibration\front_previews
```

程序会输出：

- 有效和失败图片数量；
- RMS误差；
- 平均重投影误差；
- 每张图片误差；
- 被剔除的高误差图片；
- 相机矩阵和畸变参数JSON。

## 误差判断

经验目标：

```text
平均重投影误差 < 1.0 px：基本可用
平均重投影误差 < 0.5 px：较好
个别视图 > 1.5 px：检查模糊、反光、棋盘弯曲或角点错误
```

误差小不一定代表标定一定正确。如果所有图片都集中在画面中央，即使误差小，边缘畸变仍可能估计不准。

## 去畸变验证

```powershell
python undistort_image.py `
  --calibration configs\camera_front_calibration.json `
  --input data\calibration\verify.jpg `
  --output data\calibration\verify_undistorted.jpg
```

检查：

- 原图弯曲的直线是否变直；
- 图像边缘是否出现异常拉伸；
- 棋盘格角点是否稳定；
- 输出尺寸是否符合后续算法要求。

## 何时必须重新标定

以下任一变化后需要重新标定或至少重新验证：

- 更换摄像头；
- 更换镜头；
- 改变焦距；
- 改变分辨率；
- 改变驱动的裁切/缩放模式；
- 镜头被碰撞或松动；
- 相机模块重新装配导致镜头位置变化。

仅改变相机在机器人上的安装角度通常不改变内参，但需要重新做相机到机器人底盘的外参标定。

## 两台机器人相机的文件

建议分别保存：

```text
configs/camera_front_calibration.json
configs/camera_gripper_calibration.json
```

文件名中不要只写`camera0`，因为USB编号可能变化。使用相机角色、型号和序列号更可靠。
