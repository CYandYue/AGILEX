# Scout 双 MID360 / FAST-LIO-MULTI 部署

已在本机 Ubuntu 20.04 / ARM64 / ROS Noetic 编译并完成双雷达 + 外置 HI226 静止实机测试。默认使用 FAST-LIO-MULTI 的 **bundle** 模式。此工作空间叠加厂家工作空间，复用已有 Livox driver2、HI226、RealSense 驱动。

`lio_ws` 及其中 FAST-LIO-MULTI / ikd-tree 的源码由 AGILEX 主仓库直接跟踪，不是嵌套仓库或 submodule。推送主仓库即可携带这次 LIO 适配；构建产物和原始 bag 不纳入版本管理。上游来源与版本见 `src/FAST_LIO_MULTI/UPSTREAM.md`。

注意：主仓库原有 `src/` 中部分厂家依赖已经以 gitlink 形式记录，但缺少顶层 `.gitmodules`，且本机存在历史未提交改动。这是已有仓库的可复现性限制，本次 LIO 提交不修复或打包这些历史内容；在另一台机器上构建仍需先准备完整的厂家工作空间及依赖。

## 启动

不要同时运行厂家原有雷达、IMU、gmapping 或底盘 TF launch，以免重复占用设备、发布冲突坐标变换。新 launch 不启动底盘控制。启动后先保持静止数秒，等待 IMU 初始化。

```bash
# 默认：双 MID360 + 外置 HI226，无相机、无 RViz
~/agilex_ws/lio_ws/run.sh

# 同时启动 D435 RGB-D（深度对齐到彩色）和显示
~/agilex_ws/lio_ws/run.sh camera:=true rviz:=true

# 改用 192.168.1.113 的 MID360 内置 IMU
~/agilex_ws/lio_ws/run.sh imu_source:=mid360
```

Ctrl-C 停止。默认不自动保存地图、不设开机自启。相机启动选项已配置，当前验证记录针对 LIO；尚未做 RGB-D 逐帧位姿导出和相机联合精度验证。

修改代码后编译：

```bash
bash ~/agilex_ws/lio_ws/build.sh
```

脚本使用系统 `/usr/bin/python3`，自动加载厂家 overlay，限制为两个编译任务。当前默认仅编译 bundle。上游 async/adaptive 尚未验证，本次的队列、时间戳修正仅作用于 bundle；不要将切换算法当成已验证功能。实验构建开关为 `-DFAST_LIO_MULTI_BUILD_EXPERIMENTAL=ON`，其中还会构建上游故意错误的对照实现，不能用于采集。

## 设备和数据

| 设备 | 连接 | 输入 | 本次实际读取 |
|---|---|---|---|
| MID360 A | eth0 / 192.168.1.113 | `/livox/lidar_192_168_1_113` | 约 10 Hz，约 20000 点/帧 |
| MID360 B | eth0 / 192.168.1.154 | `/livox/lidar_192_168_1_154` | 约 10 Hz，约 20000 点/帧 |
| 外置 HI226 | CP210x / `/dev/ttyUSB0`，115200 | `/imu/data_raw` | 本次约 194 Hz，静止加速度模长约 9.92 m/s² |
| MID360 内置 IMU | 同雷达网络 | `/livox/imu_192_168_1_113`、`...154` | 各约 200 Hz，加速度单位 g |

主机雷达网卡为 `192.168.1.4`。`config/mid360_dual_raw.json` 同时启用两个雷达，**所有驱动外参保持零**。点云使用 driver2 `CustomMsg`，保留逐点 offset_time；坐标变换由估计器执行，不能先拼接成 base_link 点云再输入。

C++ `sensor_relay` 仅重标 frame_id，原样保留坐标、单位和时间戳：

- `/sensors/mid360_a/lidar`、`/sensors/mid360_b/lidar`
- `/sensors/mid360_a/imu`、`/sensors/mid360_b/imu`

如果修改雷达 IP，同时修改 JSON 和 `launch/mapping.launch` 中 relay 的 `input_topics`，保持 A/B 顺序一致。外置 IMU 驱动当前固定使用 `/dev/ttyUSB0`；增加 USB 串口设备后需确认枚举。

## 坐标变换与仍需标定的部分

记 `T_A_B` 为将 B 坐标转换到 A 坐标的变换。里程计 `/lio/odometry` 输出 `T_lio_map_body`，动态 TF 同名。`body` 是本次选用的 IMU 参考系，不应无条件当作底盘中心。

```text
lio_map → body → mid360_a_lidar → mid360_b_lidar → mid360_b_imu
                            ├→ mid360_a_imu
                            └→ base_link → camera_link → 相机驱动内部 optical frames
                                        └→ imu_link
```

上图 camera_link、imu_link 同为 base_link 的子节点；D435 内部帧由 RealSense 驱动发布。

- `hi226.yaml`：默认 IMU。沿用厂家 `imu_link == base_link` 假设，`T_IMU_L1` 平移 `(0.175, -0.175, 0.06)` m，yaw `-0.780` rad。**厂家单位变换不构成外置 IMU 已标定的证据。**
- `mid360.yaml`：MID360 A 内置 IMU。使用名义 `T_IMU_Lidar` 平移 `(-0.011, -0.02329, 0.04412)` m、旋转单位阵。
- 双雷达相对变换由厂家双雷达 JSON 推出：`T_L1_L2` 平移 `(-0.4949747468, 0, 0)` m，旋转 `diag(-1,-1,1)`。厂家 JSON 两雷达 yaw 为 -45° / 135°，与 launch 的 -0.780 rad 有小差异；这是配置初值，非实测高精度标定。
- `mounts.yaml`：厂家 `base_link → camera_link` 平移 `(0.205, 0, 0.07)` m，无旋转；保存 base→L1 等安装值。修改 LIO 外参或安装关系时需保持这些配置一致。
- 未开启在线外参估计。双雷达外参仍需验证重叠区域重合程度；外置 IMU 的真实安装位置/方向、相机外参、时间偏移都需要后续标定。

HI226 厂家驱动给数据打的是 **主机收包时间**，不是传感器硬件采样时间。两雷达时间戳能用于当前联通测试，不代表与 HI226/D435 已完成硬件同步。点云到达时间减扫描起始时间约 0.1 s 含整帧采集过程，不能直接当作时钟偏移。

`common/bundle_max_pair_skew: 0.05` 按两路扫描起始时间配对，超限丢弃较早扫描，避免两台设备启动先后不同造成持续错误配对。该机制不是时钟校准；若持续出现 `Dropping unpaired lidar scan`，先检查时钟、点云频率和运行负载。

当前体素边长 `preprocess/filter_size_surf: 0.2` m，每路 `point_filter_num: 6`。较密的匹配输入在本机短测中出现逐渐积压，因此采用此配置并复测延迟。体素尺寸不是定位误差指标；原始输入仍可完整录制，后续可比较不同配置的轨迹与实时负载。

## 记录与后续相机位姿

主要输出：`/lio/odometry`、`/lio/path`、`/lio/cloud_registered`、`/tf`、`/tf_static`。位姿更新约 10 Hz。相机若为 30 Hz，需要按图像时间戳插值轨迹，不能直接用最近一条 odometry 作为每帧精确位姿。

后续每帧 optical pose 的计算为：

`T_map_camera_optical(t) = T_map_body(t) × T_body_camera_link × T_camera_link_camera_optical`。

后两项分别来自安装标定和 RealSense 驱动。当前没有闭环/全局优化，也没有 RGB-D 帧级轨迹导出器。允许离线优化时，应保留原始传感器数据，之后在优化轨迹上按图像时刻求位姿。

在新终端先进入 bash，加载环境后录制（请按实际时长安排存储空间）：

```bash
bash
source ~/agilex_ws/lio_ws/devel/setup.bash
rosbag record -O experiment.bag \
  /livox/lidar_192_168_1_113 /livox/lidar_192_168_1_154 \
  /livox/imu_192_168_1_113 /livox/imu_192_168_1_154 /imu/data_raw \
  /camera/color/image_raw /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info \
  /tf /tf_static /lio/odometry
```

离线重跑 LIO：先启动 `run.sh start_drivers:=false use_sim_time:=true`，再在加载 overlay 的终端运行：

```bash
rosbag play experiment.bag --clock --topics \
  /livox/lidar_192_168_1_113 /livox/lidar_192_168_1_154 \
  /livox/imu_192_168_1_113 /livox/imu_192_168_1_154 /imu/data_raw
```

此处只回放原始 LIO 输入，避免旧 TF/odometry 与重算结果冲突。循环或跳转回放前重启估计器；当前不是可自动重置的定位服务。

## 来源、修改与验证

通过用户 `proxy_on` / Clash 下载 `https://github.com/engcang/FAST_LIO_MULTI.git`，包括 ikd-tree 子模块。

- 上游 commit：`6c21072e2c12876d7f13697e9f3492fb460c107c`
- ikd-tree commit：`0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`
- 本地补丁：`patches/fast_lio_multi_driver2_bundle.patch`。

本地适配包括：切换 Livox driver2 消息类型；降低无用编译负担；bundle 每路每次消费一帧，保留等待 IMU 时新到的帧，避免积压多帧直接拼接但逐点时间未重定位；配对时丢弃过旧扫描；将两路逐点时间统一到较早扫描的起点后去畸变；输出时间戳使用两帧较晚结束时刻；发布前填入当前协方差；点云时间倒退时同步清理对应时间队列。

还修正了初始地图构建：本次 ikd-tree 子模块在空树上执行 `Add_Points(..., true)` 时，将输入点替换成体素中心。合成静止场景中，这会产生随体素尺寸改变的位置偏移。bundle 现直接用已经过 PCL 降采样的实际点坐标调用 `Build()`，避免初始地图几何被人为平移；未修改 ikd-tree 子模块本身。

验证摘要保存在 `verification/`。测试使用私有 ROS master，结束后关闭所启动的所有节点，不发送车辆运动命令。

- Release 编译通过。
- HI226 / MID360 两种 IMU 配置的合成双雷达测试通过；各输出 88 帧 odometry，较晚扫描结束时间与最终位姿时间戳误差约 0.078 ms（末点降采样造成）。两项均实际触发并验证了启动时半秒积压帧的丢弃。HI226 配置模拟 0.5 rad/s 原地旋转，两路扫描相差 20 ms，最终 yaw 误差约 0.00180 rad，平移误差约 0.456 mm；MID360 配置的理想静止场景末端平移误差约 0.0227 mm。两者均通过低于 2 cm 的平移误差检查，包含 relay 数据保真、轨迹有限性和相机 TF 平移组合检查。合成数据无真实传感器噪声，这不是实车精度评测。
- 最终配置的实机双 MID360 + HI226 测试启动持续 20 秒，雷达就绪后得到约 14 秒数据；两路分别 139 / 137 帧，里程计 133 帧、约 10 Hz，无时间倒退。位姿接收时间减位姿时间戳平均约 92 ms，最后一帧约 102 ms，通过平均/末帧低于 0.5 s 的延迟检查。末端相对初始位置模长约 0.0138 m。未行驶、未对照真值，不能据此宣称厘米级运动定位精度。
- 尚未完成：移动实验、长程漂移/闭环评估、实时相机联合负载测试、外参和时间标定。

测试入口为 `src/agilex_lio/test/smoke_test.py` 和 `hardware_probe.py --mapping`；前者不启动硬件，后者会启动真实雷达/IMU并短时录包，不能与正式采集同时运行。运行前加载本 overlay。
