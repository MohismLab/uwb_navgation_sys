# 接口约定：话题 / 参数 / QoS

供 GUI、导航等其他程序对接。`<robot>` 是机器人名字（`rm_0`、`dog_0` ...），同时也是它的 ROS 命名空间。

## 1. UWB 定位（控制站，`nlink_parser2`）

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/uwb/<robot>/pose` | `geometry_msgs/PoseStamped` | reliable, depth 10 | 原始 UWB 位置，约 50 Hz，`frame_id: world`。orientation 无意义（单位四元数）。z 噪声很大，只用 x/y |
| `/uwb_ekf/<robot>/pose` | `geometry_msgs/PoseStamped` | reliable, depth 10 | EKF 平滑后的位置，`frame_id: world`，z 固定为地面高度 `floor_z`。**`pose_valid` 为 false 时不发布** |
| `/uwb_ekf/<robot>/pose_valid` | `std_msgs/Bool` | reliable + **transient_local**, depth 1 | 原始位置超过 0.5 s 没更新时为 false，恢复后为 true；只在变化时发布 |
| `/uwb_ekf/<robot>/heading_valid` | `std_msgs/Bool` | reliable + **transient_local**, depth 1 | 目前恒为 false：`/uwb_ekf/<robot>/pose` 的 orientation 恒为单位四元数，**不是航向**。将来写入真实航向后会变为 true，届时 orientation 表示 UWB 坐标系中车体 +x 的方向 |
| `/uwb_ekf/<robot>/path` | `nav_msgs/Path` | reliable, depth 10 | 最近 30 s 轨迹，10 Hz；EKF 重置时清空 |
| `/uwb_ekf/<robot>/label` | `visualization_msgs/Marker` | reliable, depth 10 | 机器人名字文字，生命周期 1 s |
| `/uwb_ekf/<robot>/odometry/filtered` | `nav_msgs/Odometry` | — | robot_localization 原始输出，**不要使用**：标签掉线时会继续外推，yaw 会漂 |
| `/uwb/anchors` | `visualization_msgs/MarkerArray` | reliable, depth 10 | 基站 3D 模型 + 名字，1 Hz 重发，生命周期 3 s，`frame_id: world` |
| `/nlink_linktrack_anchorframe0` | `nlink_parser2/LinktrackAnchorframe0` | reliable, depth 200 | 基站原始帧（标签位置 + 到各基站的距离）|

EKF 重置：标签恢复（或第一帧）、或原始位置连续约 0.1 s 偏离 EKF 1 m 以上时，适配节点通过
`/uwb_ekf/<robot>/set_pose` 把 EKF 直接放到原始位置并清零速度，输出位置会**瞬间跳变**——下游画轨迹时应断开重画。

TF：`world → uwb_floor`（静态，z = `floor_z`）。不发布机器人的 TF。

### `linktrack_node` 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port_name` / `baud_rate` | `/dev/ttyACM0` / `1000000`（launch 中）| 串口 |
| `pose_topic_prefix` | `/uwb` | 位姿话题前缀 |
| `tag_name_prefix` | `rm_` | 标签 id → 机器人名：`<prefix><id>` |
| `pose_frame_id` | `world` | 位姿的 frame_id |
| `read_anchors` | `true` | 启动时读取基站坐标 |
| `anchor_group` | `0` | 读取哪组基站：0 = A0–A9，1 = A10–A19，2 = A20–A29 |

### `uwb_ekf_adapter.py` 参数（launch 自动传入）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--robot` | `rm_0` | 机器人名 |
| `--std` | `0.05` | UWB 位置标准差 [m] |
| `--floor-z` | `-1.75` | 地面高度 [m] |
| `--timeout` | `0.5` | 多久没有原始位置判为掉线 [s] |
| `--reset-distance` | `1.0` | 持续偏离多远重置 EKF [m] |
| `--history` | `30` | 轨迹长度 [s] |

## 2. 机器人端（不在本仓库，供对接参考）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/<robot>/cmd_vel` | `geometry_msgs/Twist` | 底盘速度，**车体坐标系**（x 前、y 左），全向底盘。底盘驱动订阅为 best effort |
| `/<robot>/odometry/filtered` | `nav_msgs/Odometry` | 机器人上 IMU 的地磁航向（EKF）：yaw 为 ENU 约定，**从磁东起逆时针**，未加磁偏角；只用 orientation |
| `/<robot>/imu/mag_state` | `std_msgs/String` | `LOCKED`（采用地磁）/ `HOLD`（运动中暂停）/ `REJECTED`（磁干扰）|
| `/<robot>/imu/flat_calib/start` | `std_msgs/Float32` | 开始原地转圈地磁校准，数据为圈数（≤0 用默认 5 圈）|
| `/<robot>/imu/flat_calib/status` | `std_msgs/String`（transient_local）| 校准状态：`待命` / `校准中：…` / `完成：…` / `失败：…` |
| `/uwb_nav/<robot>/cancel` | `std_msgs/Empty` | 停车：导航节点和地磁校准都会停止。**其他程序接管机器人前应先发这个** |
| `/uwb_nav/<robot>/goal_pose` | `geometry_msgs/PoseStamped` | 导航目标（UWB 坐标，只用 x/y）|

磁航向换算到 UWB 坐标系：`psi_uwb = offset_robot + s · yaw_imu`，UWB 坐标系为镜像，`s = -1`；
`offset_robot` 取决于 IMU 安装方向以及磁北与 UWB 轴的夹角，需要按机器人标定。

## 3. 注意事项

- 机器人（Jetson Orin）上 Fast DDS 的共享内存通道在进程重启后可能失效（能发现节点但收不到数据），
  机器人上本机订阅时建议用只走 UDP 的 profile（`FASTRTPS_DEFAULT_PROFILES_FILE`）。跨机器订阅不受影响。
- `ROS_DOMAIN_ID=0`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。
