# RGB-D + LIO 数据采集

本目录由 AGILEX 主仓库直接管理，无独立 Git 仓库。目标是生成一个服务器端可独立读取的 ROS1 bag：每组 RGB-D 都有对应图像时刻的相机光学坐标系位姿，同时保留原始 LIO 轨迹、运行时 TF 和标定信息。

验收目标为实际完整帧组 **超过 2 Hz**。默认采集上限 **6 Hz**，相机 **640×480、6 Hz** 工作，配对后整组降频。每帧相机位姿通过图像时刻前后两条 LIO 位姿插值，并乘以实际运行中的静态外参获得。相机关闭彩色自动曝光的低照度降帧选项（自动曝光仍开启），减少 RGB/深度配对失败；实际输出以报告中的 `actual_rate_hz` 为准。这里的“一一对应”是明确的数据关联，不表示硬件同步或真值精度。

## 1. 第一次实测

先结束原先的 LIO/相机 launch，避免设备和 TF 重复启动。

终端 A，启动双 MID360、HI226、D435；默认不开 RViz：

```bash
~/agilex_ws/rgbd_capture/start_sensors.sh
```

需要观察时：

```bash
~/agilex_ws/rgbd_capture/start_sensors.sh rviz:=true
```

等待 `IMU Initial Done`，保持静止几秒，再在终端 B 录制：

```bash
~/agilex_ws/rgbd_capture/run.sh record ~/datasets/scene01.bag
```

脚本会等待有效 RGB-D、CameraInfo、LIO 前后位姿和静态外参，之后显示 `frames=... rate=...`。先确认帧数持续增加，再开车。**不要把启动日志出现当成已经成功采集**。

建议第一段短实测加上原始双雷达/IMU，便于后续重新运行 LIO：

```bash
~/agilex_ws/rgbd_capture/run.sh record ~/datasets/debug02.bag --raw-sensors
```

在终端 B 按 **Ctrl-C** 停止录制。脚本先停止接纳新图像，继续等待最多 3 秒的 LIO 尾部数据，然后关闭并校验 bag。**看到 `Validated bag:` 后，再停止终端 A**。采集脚本不会发送底盘控制命令。

成功输出：

```text
scene01.bag                  # 自包含的数据文件，上传服务器用这个
scene01.bag.report.json      # 自动校验报告，便于快速检查
```

bag 内也保存会话元数据及录制报告，所以读取数据不依赖旁边的 JSON。所有输出拒绝覆盖，重录请换文件名。

也可以设置时长（从采集进程开始算，包含准备时间，停止后仍会等待位姿尾部）：

```bash
~/agilex_ws/rgbd_capture/run.sh record ~/datasets/scene02.bag --duration 60
```

## 2. Bag 内容和服务器读取约定

### 每个完整帧组恰有以下 7 条消息

| 话题 | 标准 ROS 消息 | 含义 |
|---|---|---|
| `/dataset/rgb/image_raw` | sensor_msgs/Image | 原始 RGB 像素，无损保存 |
| `/dataset/depth/image_raw` | sensor_msgs/Image | 与 RGB 像素模型对齐的深度 |
| `/dataset/rgb/camera_info` | sensor_msgs/CameraInfo | 本帧 RGB 内参、畸变模型、K/D/R/P |
| `/dataset/depth/camera_info` | sensor_msgs/CameraInfo | 本帧对齐深度的像素模型 |
| `/dataset/camera_pose` | geometry_msgs/PoseStamped | `T_lio_map_camera_color_optical_frame` |
| `/dataset/body_to_camera` | geometry_msgs/TransformStamped | 本帧计算使用的 `T_body_camera_optical` |
| `/dataset/frame` | std_msgs/String，内容为 JSON | 帧编号、原始时间戳、原始序号、同步与插值信息 |

上述 7 条消息的 **bag 记录时间相同，为 RGB 原始时间戳**。前 6 条消息的 `header.seq` 统一为从 0 连续递增的 `frame_index`；JSON 保存相同编号。RGB 与深度各自的原始 `header.stamp` 保持不变，不能假定深度 header 时间一定等于 RGB。

相机位姿的 header 时间为 RGB 时间，`header.frame_id = lio_map`。相机坐标系名称另存于帧 JSON 及 `body_to_camera.child_frame_id`。原始 RGB/深度序号保存在 `rgb_original_seq` / `depth_original_seq`，CameraInfo 原始时间也有记录。所有源传感器时间均使用消息 header，不用电脑收包时间替代。

位姿约定：

```text
p_world = T_world_camera_optical × p_camera_optical
相机 optical 坐标：x 向右，y 向下，z 向前
T_world_camera = T_world_body(t_rgb + offset) × T_body_camera
```

平移单位为米，四元数顺序为 xyzw。若算法需要 world-to-camera 的投影矩阵，应对保存的变换求逆。

### 另外保留的内容

- `/lio/odometry`：接收到的原频率里程计，包括原始 header、pose、协方差等；不是重新生成的相机位姿。
- `/tf`：接收到的完整动态 TF。
- `/tf_static`：收集并合并不同发布者的静态 TF 快照，保留 latch 属性。即使静态消息早于录制启动，订阅后仍可收到 latched 数据。每帧还单独保存实际使用的相机外参。
- 原 RGB、对齐深度 CameraInfo；若驱动发布，也保存原始深度 CameraInfo、彩色/深度 metadata 和 `depth_to_color` 外参话题。
- `/dataset/session`：schema 版本、采集参数、程序 Git 版本及源码哈希、`/lio`、`/lio_mounts`、`/camera` 参数快照、坐标/时间/深度单位约定。
- `/dataset/report`：录制统计、丢弃原因和是否正常收尾。停止后的完整校验结果另保存在 `.report.json`，也可随时从 bag 重新运行 `check`。
- `--raw-sensors` 开启时，保存两路 `/livox/lidar_192_168_1_*`、两路内置 IMU 和 `/imu/data_raw`。原始雷达保留逐点时间。高频原始数据由独立 C++ `rosbag record` 录制，停止后按记录时间合并到同一个最终 bag，并核对所有原始话题的消息数。两路雷达及当前 LIO 使用的 IMU 必须有数据，其余未发布的可选原始话题不会被虚构。

核心数据均为标准 ROS 消息，服务器读取 `/dataset/*` 不需要安装 Livox 消息包。Bag 自带可选原始话题的消息定义；自行读取全部话题的第三方工具应支持动态消息定义。

## 3. 同步与异常处理

- RGB/对齐深度按发布的 header 时间配对，默认最大差 **10 ms**；每张图只用一次。先配对，再按时间相位选取约 6 Hz 帧组，不能将各话题独立限频后按顺序拼接。
- 深度和 RGB 必须具有相同尺寸、光学 frame 和相同 CameraInfo 像素模型。**不会用 resize 假装修复未对齐深度**。当前不支持带 ROI、binning 或非单位 R 的输入。
- 图像时刻必须有两条不同时间的 LIO 位姿包围，默认间隔不得超过 **0.25 s**。平移线性插值，旋转四元数 SLERP；不外推、不复用上一帧 pose、不写 NaN 占位。
- 默认等待最多 **3 s**。缺少位姿、标定、有效深度或 TF 的帧整组跳过并计数。若启动后 20 秒仍无有效帧，或有效帧持续中断，会停止并报错。
- 输入时钟倒退、里程计 frame 改变、使用中的相机安装外参发生变化、写盘队列溢出等会中止这次录制。请排查后重新建 bag，而不是把重启前后的轨迹当成一条连续轨迹。
- `source_sequence_gaps` 是源 header 序号跳号诊断，不足以证明所有底层丢包都能被检测到。最终应同时检查实际输出频率、丢弃计数和深度有效比例。
- 默认保留至少 **2 GiB** 空间；临时队列有数量和内存上限，避免静默积压。默认 LZ4 无损压缩，可选 `--compression none`。

**时间同步的边界：** RealSense `enable_sync=true` 仅启用驱动帧组同步，深度对齐也不是硬件同时曝光。这个版本驱动可能给帧组里的图像使用统一 ROS header 时间；时间戳相等不证明曝光同步。HI226 仍使用主机收包时间，物理偏移尚未标定。保存设备 metadata 便于后续分析，但脚本不会凭 header 自动声称硬件同步已验证。

`--camera-time-offset SECONDS` 的定义为 `t_lio = t_rgb + offset`，默认 0。只有获得标定结果后才调整；不能直接拿 RViz 或消息显示延迟当作这个偏移。原始图像时间不改写，查询时刻记录在 `pose_query_stamp_ns` 中。

## 4. 深度和畸变

原 bag 不对图像缩放、去畸变、补洞、平滑或有损压缩，保留当前相机输出及标定数据。

- 本机 RealSense 驱动输出的 `16UC1` 深度按毫米处理，即默认 `depth_scale_m = 0.001`。如果换了发布者或单位，须显式指定 `--depth-scale`。
- `32FC1` 深度约定为米，比例固定为 1；不强制量化成 uint16。
- 小于等于 0 或非有限值为无效深度；每帧记录有效像素比例。全无效帧不写入正式帧组。
- “深度对齐到 RGB”不等于“RGB 已去畸变”。本帧 CameraInfo 中的 distortion_model、D、K、R、P 一起保存。

可选离线导出会生成服务器算法易用的目录：

```bash
# 保留原像素模型
~/agilex_ws/rgbd_capture/run.sh export ~/datasets/scene01.bag ~/datasets/scene01_export

# 同时对 RGB 和深度去畸变，输出针孔像素模型
~/agilex_ws/rgbd_capture/run.sh export ~/datasets/scene01.bag ~/datasets/scene01_rectified --rectify
```

输出含 `rgb/*.png`、`depth/*.png`（16UC1）或 `depth/*.npy`（32FC1）、
`poses_tum.txt`、`frames.jsonl`、`dataset.json`。每行 `frames.jsonl` 包含唯一编号、
路径、原始及导出内参、原始时间戳、深度比例和 4×4 相机位姿矩阵；不要依赖目录排序猜对应关系。

`--rectify` 支持 `plumb_bob` / `rational_polynomial` 和四参数 `equidistant`。
RGB 用线性重采样，深度用最近邻，避免在物体边缘混合深度。保持原 K、原输出尺寸和 R=I，导出的 D 置零、P 更新；相机位姿坐标系不变。边缘可能出现黑色/零深度，仍按无效值处理。未知模型拒绝处理，不能默认畸变为零。

导出前自动检查 bag；输出目录必须不存在。失败的导出目录带 `INCOMPLETE` 标记。

## 5. 校验与失败文件

```bash
~/agilex_ws/rgbd_capture/run.sh check ~/datasets/scene01.bag
```

校验会检查帧组完整性、连续编号、各时间戳、有效深度与内参、原始里程计连续性、
TF 连通所需的变换，并从原始 odometry 和外参重新计算每帧相机位姿进行比较。
校验通过表示结构与计算一致，**不是位姿精度或硬件时间同步认证**。

录制期间使用 `.bag.active`；干净关闭且校验通过才改名为 `.bag`。
正常处理到的错误保留 `.bag.partial` 和失败报告；断电/强杀可能只留下 `.active`。
这些文件供恢复排查，不当作完整实验数据，脚本也不会自动覆盖或删除。

`--raw-sensors` 录制过程中还会出现 `.bag.raw.bag.active` / `.bag.raw.bag` 和 `.bag.raw.log`；
停止后的合并使用 `.bag.merge.active`。合并及校验可能需要几十秒或更久，随数据量增加；请等待 `Validated bag:` 再关闭终端。
合并阶段需要额外约一份录制数据的磁盘空间；成功后自动清理本次临时分片，失败时保留用于排查。
报告中的 `queue_peak_events` / `queue_peak_bytes` 是主队列峰值，`raw_topic_counts` 是原始话题消息数。
`PASS` 表示数据完整性和位姿关联校验通过，实际帧率请单独看 `actual_rate_hz`，它不保证达到目标帧率。

只希望上传一个文件时上传最终 `.bag` 即可。服务器 `check/export` 不需要 roscore，
但需要 ROS1 Python 的 rosbag、标准消息、tf/tf2，以及 NumPy；去畸变/PNG 导出还需要 OpenCV。
可以先在 ROS Noetic 环境中导出，再让三维场景图算法使用普通 PNG/NPY/JSON/TUM 数据，无需改算法的 Python 环境。

## 6. 与旧采集脚本的关系及验证范围

参考了根目录 `collect_data.py` / `collect_data_v2.py` 的 NumPy 图像读取、
CameraInfo 保存和 TF 位姿提取思路，但不直接复用旧的雷达投影、独立写图/位姿以及
尺寸不匹配就缩放深度的流程。旧脚本保存过畸变参数，实际没有执行去畸变；这里新增
可选的同步重映射导出，并严格检查编码、大小端和行填充。

新代码测试入口：

```bash
bash -c 'source /opt/ros/noetic/setup.bash; /usr/bin/python3 -m unittest discover -s ~/agilex_ws/rgbd_capture/tests -v'
bash -c 'source /opt/ros/noetic/setup.bash; /usr/bin/python3 ~/agilex_ws/rgbd_capture/tests/smoke_live.py'
```

第二项会创建私有临时 ROS master，发布合成 640×480 RGB-D，执行真实采集 CLI、
Ctrl-C 收尾及 bag 校验；添加 `--raw-sensors` 可验证独立原始数据录制及合并（合成雷达负载不依赖 Livox 包）。
该合成测试不启动传感器和底盘。静态实机结果见 `verification/`，运动同步和融合质量仍需行驶实测确认。
