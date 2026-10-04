# UWB 多机器人定位与导航系统

基于 Nooploop LinkTrack UWB 的多机器人室内定位系统（ROS 2 Humble）：
UWB 基站解算每个标签（机器人）的位置，经 EKF 平滑后发布到 ROS，
在 RViz 中显示基站、机器人和轨迹，并通过 RViz 面板调节导航参数、对机器人做地磁校准。

当前用于 3 台 RoboMaster（`rm_0` ~ `rm_2`），后续还会加入机器狗（`dog_<id>`），
所以所有接口都按“机器人名字”区分，不限定机器人类型。

---

## 1. 系统组成

```
                         ┌──────────── 控制站 (mo, Ubuntu 22.04 + ROS 2 Humble) ────────────┐
 UWB 基站 ─USB(/dev/ttyACM0)→ │ linktrack_node ──/uwb/<robot>/pose──→ uwb_ekf_adapter ⇄ ekf_node  │
 (LinkTrack, 1 Mbps)          │       │                                  │                        │
                              │       └──/uwb/anchors (基站 3D 模型)       └→ /uwb_ekf/<robot>/pose  │
                              └──────────────────────────────────────────────────────────────────┘
                                                      │  ROS 2 网络 (Fast DDS, domain 0)
                    ┌─────────────────────────────────┴─────────────────────────────┐
                    │                                                               │
            RViz (任意电脑)                                                   机器人 (Jetson Orin)
     基站/机器人/轨迹显示                                           底盘驱动 /<robot>/cmd_vel
     uwb_rviz_plugins 面板                                          IMU 地磁航向 /<robot>/odometry/filtered
```

| 部分 | 位置 | 作用 |
|---|---|---|
| `nlink_parser2` | 控制站 | LinkTrack 串口驱动（在 Nooploop 官方包基础上修改）、按标签发布位姿、读取基站坐标、多机器人 EKF |
| `uwb_rviz_plugins` | 运行 RViz 的电脑 | RViz 面板“UWB 导航”：选择机器人、导航参数、停车、地磁校准 |
| `docs` | — | 话题 / 参数 / QoS 约定，供 GUI 等其他程序对接 |

---

## 2. 目录结构

```
.
├── README.md                       本文件
├── .gitmodules                     第三方子模块 (asio, nlink_unpack, protocol_extracter)
└── src/
    ├── docs/
    │   └── interfaces.md           话题、参数、QoS 约定（对接用）
    ├── nlink_parser2/              ROS 2 包 (ament_cmake)
    │   ├── launch/
    │   │   ├── linktrack.launch.py         只启动 UWB 驱动
    │   │   ├── linktrack.ekf.launch.py     UWB 驱动 + 每个机器人一套 EKF（日常使用）
    │   │   └── linktrack_aoa / tofsense*   官方的其他产品（未改动）
    │   ├── config/uwb_tags.yaml            标签 id → 机器人名字（加机器人改这里）
    │   ├── config/uwb_velocity.yaml        哪些机器人把自身速度融合进滤波（目前 dog_4）
    │   ├── scripts/uwb_ekf_adapter.py      每个机器人的卡尔曼滤波（速度融合、掉线检测、重置、轨迹）
    │   ├── meshes/
    │   │   ├── uwb_anchor.dae              RViz 中的基站 3D 模型（三脚架 + 模块 + 天线）
    │   │   └── make_anchor_mesh.py         生成上面模型的脚本（改尺寸/颜色后重新运行）
    │   ├── src/linktrack_node/             驱动节点：解析帧、按标签发布位姿、读取基站坐标
    │   ├── src/nlink_utils/                协议解析库（nlink_unpack、protocol_extracter 为子模块）
    │   ├── extern/asio, extern/serial      串口库（asio 为子模块）
    │   └── msg/                            LinkTrack / TOFSense 消息定义
    └── uwb_rviz_plugins/           ROS 2 包 (ament_cmake, Qt5)
        ├── src/uwb_nav_panel.{hpp,cpp}     RViz 面板 “UWB 导航”
        └── plugins_description.xml
```

---

## 3. 安装与编译

依赖：Ubuntu 22.04、ROS 2 Humble（滤波在 `uwb_ekf_adapter.py` 内部，只需要 numpy，不再需要 robot_localization）。

```bash
sudo apt install python3-numpy python3-colcon-common-extensions

mkdir -p ~/uwb_ws && cd ~/uwb_ws
git clone --recursive git@github.com:MohismLab/uwb_navgation_sys.git .
# 已经 clone 但没带子模块时：
git submodule update --init --recursive

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

- 控制站（接 UWB 基站的电脑）需要 `nlink_parser2`。
- 运行 RViz 的电脑需要 `nlink_parser2`（基站模型 `package://nlink_parser2/meshes/...`）和 `uwb_rviz_plugins`。

串口权限（只需一次，之后重新登录）：

```bash
sudo usermod -aG dialout $USER
```

---

## 4. 使用方法

### 4.1 启动 UWB 定位（控制站）

UWB 基站模块通过 USB 接在控制站上（CH9102/CDC 串口，波特率 **1000000**）。

```bash
source ~/uwb_ws/install/setup.bash
ros2 launch nlink_parser2 linktrack.ekf.launch.py
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port_name` | `/dev/ttyACM0` | 基站串口 |
| `baud_rate` | `1000000` | 基站波特率（在 NAssistant 中设置） |
| `tags_file` | `config/uwb_tags.yaml` | 标签 id → 机器人名字 的映射（见下）|
| `robots` | 空 = 映射文件里的全部机器人 | 需要 EKF 的机器人，逗号分隔 |
| `uwb_std` | `0.05` | 只用 UWB 时的位置标准差 [m]，越大越平滑、跟随越慢 |
| `velocity_file` | `config/uwb_velocity.yaml` | 融合机器人自身速度的配置（见 `src/docs/interfaces.md`“速度融合”），`''` 为全部只用 UWB |
| `floor_z` | `-1.75` | 地面在 UWB 坐标系中的高度 [m]（基站在 z=0）|

例：只给 rm_0 和 dog_4 开 EKF

```bash
ros2 launch nlink_parser2 linktrack.ekf.launch.py robots:=rm_0,dog_4
```

#### 标签 → 机器人名字：`src/nlink_parser2/config/uwb_tags.yaml`

```yaml
/**:
  ros__parameters:
    tag_ids:   [0,      1,      2,      4]
    tag_names: ["rm_0", "rm_1", "rm_2", "dog_4"]
    publish_unmapped: false
```

- 标签 `tag_ids[i]` 发布为 `/uwb/<tag_names[i]>/pose`；EKF 默认给这里列出的每个机器人各开一套。
- **加机器人只改这一个文件**（改完重新 `colcon build` 或用 `tags_file:=` 指向自己的文件，然后重启 launch）。
- 没列出的标签**不发布**，日志对每个 id 警告一次——这也过滤掉了基站帧偶尔解析出的假标签（如 122、250–253）。
  `publish_unmapped: true` 时没列出的标签按 `rm_<id>` 发布。

只要原始 UWB、不要 EKF：`ros2 launch nlink_parser2 linktrack.launch.py`。

启动后：

- 映射文件里的每个标签发布 `/uwb/<机器人名>/pose`（如标签 4 → `/uwb/dog_4/pose`）。
- 启动 2 秒内向基站读取基站坐标（Setting_Frame0），日志打印 `anchor A0: (x, y, z)`，
  并在 `/uwb/anchors` 发布基站 3D 模型。未配置的基站槽位（-8388 m）会被过滤。
- 每个机器人的 EKF 输出 `/uwb_ekf/<robot>/pose`、轨迹 `/uwb_ekf/<robot>/path`、名字标签 `/uwb_ekf/<robot>/label`。
- 标签掉电 / 超出范围 0.5 s 后 `/uwb_ekf/<robot>/pose_valid` 变为 false，并停止发布该机器人的位姿；
  标签恢复时 EKF 直接重置到新位置（不会从旧位置“滑”过去）。

> **注意**：`linktrack_node` 需要按两次 Ctrl+C 才会退出。

### 4.2 坐标系

| 坐标系 | 说明 |
|---|---|
| `world` | UWB 坐标系，原点和轴向由 NAssistant 中的基站坐标决定，**基站在 z=0** |
| `uwb_floor` | `world` 沿 z 平移 `floor_z`，即地面。RViz 以它为固定坐标系，网格画在地面，2D Goal Pose 点在地面上 |

UWB 坐标系相对真实世界（右手系）是**镜像**的：在 UWB 坐标中看，机器人的左侧（车体 +y）
在车头方向（车体 +x）**顺时针** 90° 处，而不是通常的逆时针 90°。用 UWB 坐标换算车体速度时要注意。

### 4.3 RViz 面板 “UWB 导航”

在 RViz 中：`Panels → Add New Panel → uwb_rviz_plugins/UwbNavPanel`。

| 控件 | 作用 |
|---|---|
| 选中（复选框） | 选择要操作的机器人，和 3D 视图中点击机器人同步 |
| 最大速度 / 最大加速度 / 比例增益 / 到达容差 | 点“应用”写入选中机器人的导航节点 `/uwb_goal_nav_<robot>`；“读取”读回 |
| 停车 / 全部停车 | 发布 `/uwb_nav/<robot>/cancel`，导航和地磁校准都会立即停止 |
| 地磁校准：圈数 + “转圈校准选中的机器人” | 发布 `/<robot>/imu/flat_calib/start`，机器人原地转圈校准地磁；下方实时显示每台的状态（待命 / 校准中 / 完成 / 失败）|

面板列出的机器人在 `.rviz` 配置里设置：

```yaml
- Class: uwb_rviz_plugins/UwbNavPanel
  Name: UWB 导航
  robots: rm_0,rm_1,rm_2
```

> 面板只是“遥控器”：导航参数需要机器人对应的导航节点在运行，地磁校准需要机器人上的 IMU
> 校准节点在运行（这两部分不在本仓库中，见 `src/docs/interfaces.md`）。没有运行时面板会提示
> “导航节点未运行” / “IMU/校准程序没在运行”。

### 4.4 在 RViz 中显示

| 显示类型 | 话题 |
|---|---|
| MarkerArray | `/uwb/anchors`（基站 3D 模型 + 名字）|
| Pose (Axes) | `/uwb_ekf/<robot>/pose` |
| Path | `/uwb_ekf/<robot>/path` |
| Marker | `/uwb_ekf/<robot>/label` |

Fixed Frame 设为 `uwb_floor`。

---

## 5. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 话题全是乱码 / 没有 `/uwb/...` | 波特率不对（应为 1000000），或串口无权限（`dialout` 组）|
| 基站没显示 | 查看日志是否有 `anchor A0: ...`；读取请求最多重试 10 次（`no Setting_Frame0 answer`），确认串口没有被其他程序占用 |
| 位置偶尔跳动 | UWB 多径 / 遮挡，滤波的马氏距离门限会丢弃离群点；持续 1 m 以上的偏离会触发重置 |
| 被遮挡时位置飘远又回来 | 某个基站被挡，测距连续偏长数秒，单靠 UWB 分不出来。机器人能提供自身速度时加入 `config/uwb_velocity.yaml`（dog_4 遮挡偏离 0.68 → 0.41 m）|
| `/uwb_ekf/<robot>/pose` 不发布 | 该机器人的标签没有数据（`pose_valid` 为 false），检查标签供电和 id |
| 本机 `ros2 topic list` 看不到别的机器 | `ros2 daemon stop && ros2 daemon start` |
| 其他程序读不到航向 | `/uwb_ekf/<robot>/pose` 的 orientation 恒为单位四元数，`heading_valid` 为 false；航向见 `src/docs/interfaces.md` |

---

## 6. 与上游的差异

`src/nlink_parser2` 来自 [nooploop-dev/nlink_parser2](https://github.com/nooploop-dev/nlink_parser2)（V3.4），主要修改：

- 按标签 id 发布 `geometry_msgs/PoseStamped`（`/uwb/<prefix><id>/pose`）
- 通过 Setting_Frame0 读取基站坐标并以 3D 模型发布到 `/uwb/anchors`（只在发出读请求后接收，防止数据流中的误判帧）
- `linktrack.ekf.launch.py` + `uwb_ekf_adapter.py`：多机器人卡尔曼滤波（可融合机器人自身速度）、掉线检测、重置、轨迹
- 默认串口 `/dev/ttyACM0`、波特率 1000000
