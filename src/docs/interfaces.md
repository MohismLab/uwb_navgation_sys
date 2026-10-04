# 接口约定：话题 / 参数 / QoS

供 GUI、导航等其他程序对接。`<robot>` 是机器人名字（`rm_0`、`dog_0` ...），同时也是它的 ROS 命名空间。

## 1. UWB 定位（控制站，`nlink_parser2`）

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/uwb/<robot>/pose` | `geometry_msgs/PoseStamped` | reliable, depth 10 | 原始 UWB 位置，约 50 Hz，`frame_id: world`。orientation 无意义（单位四元数）。z 噪声很大，只用 x/y |
| `/uwb_ekf/<robot>/pose` | `geometry_msgs/PoseStamped` | reliable, depth 10 | 滤波后的位置，`frame_id: world`，z 固定为地面高度 `floor_z`。**`pose_valid` 为 false 时不发布**。orientation：`heading_valid` 为 true 时是绕 z 轴的航向（机体 +x，即 cmd_vel 坐标系，在 UWB 坐标系中的朝向），否则为单位四元数 |
| `/uwb_ekf/<robot>/pose_valid` | `std_msgs/Bool` | reliable + **transient_local**, depth 1 | 原始位置超过 0.5 s 没更新时为 false，恢复后为 true；只在变化时发布 |
| `/uwb_ekf/<robot>/heading_valid` | `std_msgs/Bool` | reliable + **transient_local**, depth 1 | true：`pose` / `odometry/filtered` 的 orientation 是航向 `heading_offset − yaw_imu`。条件：配置了 `heading_offset`、IMU 航向 1.5 s 内有更新、`/<robot>/imu/mag_state` 60 s 内出现过 `LOCKED`（短暂拒收仍可信：陀螺零偏已扣除；从未建立地磁参考或长时间拒收则为 false）。只在变化时发布 |
| `/uwb_ekf/<robot>/path` | `nav_msgs/Path` | reliable, depth 10 | 最近 30 s 轨迹，10 Hz；EKF 重置时清空 |
| `/uwb_ekf/<robot>/label` | `visualization_msgs/Marker` | reliable, depth 10 | 机器人名字文字，生命周期 1 s |
| `/uwb_ekf/<robot>/odometry/filtered` | `nav_msgs/Odometry` | reliable, depth 10 | 和 `pose` 同时发布（同样在 `pose_valid` 为 false 时不发布）：位置 + **UWB 坐标系下的速度**（`twist.linear.x/y`，m/s）及其协方差。orientation 与 `pose` 相同 |
| `/uwb_ekf/<robot>/heading_offset` | `std_msgs/Float32` | reliable + **transient_local**, depth 1 | 只有融合速度的机器人有：当前使用的航向偏移 [deg]，见下方“速度融合” |
| `/uwb/anchors` | `visualization_msgs/MarkerArray` | reliable, depth 10 | 基站 3D 模型 + 名字，1 Hz 重发，生命周期 3 s，`frame_id: world` |
| `/nlink_linktrack_anchorframe0` | `nlink_parser2/LinktrackAnchorframe0` | reliable, depth 200 | 基站原始帧（标签位置 + 到各基站的距离）|

滤波器重置：标签恢复（或第一帧）、或原始位置持续偏离滤波结果 1 m 以上（只用 UWB 时 0.1 s，融合速度时 1 s）时，
滤波器直接放到原始位置并清零速度，输出位置会**瞬间跳变**——下游画轨迹时应断开重画。

### 速度融合（`config/uwb_velocity.yaml`）

滤波器在 `uwb_ekf_adapter.py` 内部（常加速度模型的卡尔曼滤波，不再使用 robot_localization）。
配置文件中列出的机器人，会把它自己的机体速度一起融合：

| 输入 | 类型 | 说明 |
|---|---|---|
| `velocity_topic`，如 `/dog_4/odom/twist` | `geometry_msgs/TwistStamped` | 机体坐标系速度（x 前、y 左），约 50 Hz |
| `heading_topic`，如 `/dog_4/odometry/filtered` | `nav_msgs/Odometry` | 机器人外置 IMU 的地磁航向 |

换算：`v_uwb = Rot(heading_offset − yaw_imu) · diag(1, −1) · v_body`（UWB 坐标系是镜像的）。
`v_body` 是机体坐标系（cmd_vel 坐标系）速度；话题本身转了角度的用 `velocity_rotation` 说明（RM 的 `/rm_N/vel` 为 180°）。
`heading_offset` 按机器人不同（IMU 安装方向、磁北与 UWB 轴的夹角），写在配置文件里，运行中不修改。
**标定（不需要动捕）**：配置里不写 `heading_offset`，让机器人走十几段 0.5 m 以上的直线，适配节点用
UWB 位移对比里程计积分估出偏移并打印 `heading offset estimated from straight drives: …`，把这个值写进配置。
运行中这个估计一直在做，与配置相差超过 10° 时日志报警（IMU 被重装 / 重新标定）。
不在线自动修正的原因：地磁被拒收时 IMU 航向会漂，在线估计会跟着漂（2026-10-05 dog_4 漂到 87°，实际约 61°）。
机体 +x 在 UWB 坐标系中的朝向即 `heading_offset − yaw_imu`（与 GUI 的 `psi_uwb = theta + h·yaw`、h = −1 一致）。

融合速度时 UWB 标准差 0.3 m、马氏门限 3（遮挡时某个基站测距连续偏长 0.5–0.8 m 达数秒，原始位置被拖走，
里程计不会跟着走）；速度 1 s 没收到时退回只用 UWB。
**静止**：机体速度低于 0.02 m/s 持续 0.5 s 时，滤波器的速度、加速度被固定为 0，位置只对 UWB 求平均——
停着的机器人不再随 UWB 多径晃动（实测每 5 s 的标准差 40 mm → 2 mm）。判断静止不需要 IMU 航向和
`heading_offset`，所以没标定 offset 的机器人（目前三台 RM）静止时也已生效；行驶中的速度融合要等 offset 确定。
UWB 短暂断档时，只要速度在融合，滤波器靠里程计继续推算，恢复后不重置。
dog_4 实测（对照动捕）：均方根误差 0.142 → 0.109 m，遮挡时最大偏离 0.68 → 0.41 m。

TF：`world → uwb_floor`（静态，z = `floor_z`）。不发布机器人的 TF。

### `linktrack_node` 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port_name` / `baud_rate` | `/dev/ttyACM0` / `1000000`（launch 中）| 串口 |
| `pose_topic_prefix` | `/uwb` | 位姿话题前缀 |
| `tag_ids` / `tag_names` | 见 `config/uwb_tags.yaml` | 标签 id → 机器人名（两个等长数组）|
| `publish_unmapped` | `false`（映射为空时 `true`）| 没列出的标签是否发布 |
| `tag_name_prefix` | `rm_` | `publish_unmapped` 时没列出的标签命名为 `<prefix><id>` |
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
| `--reset-distance` | `1.0` | 持续偏离多远重置滤波器 [m] |
| `--reset-time` / `--reset-time-vel` | `0.1` / `1.0` | 偏离持续多久才重置 [s]（只用 UWB / 融合速度时）|
| `--vel-topic` / `--heading-topic` / `--heading-offset` | 来自 `config/uwb_velocity.yaml` | 速度融合，见上 |
| `--history` | `30` | 轨迹长度 [s] |

## 2. 机器人端（不在本仓库，供对接参考）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/<robot>/cmd_vel` | `geometry_msgs/Twist` | 底盘速度，**车体坐标系**（x 前、y 左），全向底盘。底盘驱动订阅为 best effort |
| `/<robot>/odometry/filtered` | `nav_msgs/Odometry` | 机器人上 IMU 的地磁航向（EKF）：yaw 为 ENU 约定，**从磁东起逆时针**，未加磁偏角；只用 orientation |
| `/<robot>/odom/twist` | `geometry_msgs/TwistStamped` | 机器人自身的机体速度（x 前、y 左），50 Hz。目前 dog_4 有（狗上 `go2_odom_twist.py` 从 `/sportmodestate` 转发）|
| `/<robot>/imu/mag_state` | `std_msgs/String` | `LOCKED`（采用地磁）/ `HOLD`（运动中暂停）/ `REJECTED`（磁干扰，此时航向只靠陀螺积分）|
| `/<robot>/imu/data_raw` | `sensor_msgs/Imu` | 角速度已扣除陀螺零偏（驱动在机器人静止时估计，存在机器人的 `~/uwb_imu_ws/gyro_bias_<robot>.yaml`）。dog_4 的 z 轴零偏 0.225°/s，扣除后地磁被拒收期间航向基本不漂（对照动捕：误差中位数 9° → 1.3°）|
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
