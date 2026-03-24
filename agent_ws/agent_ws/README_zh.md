# smart_agent_gazebo 中文说明

本工作空间提供一个 ROS2 包：`smart_agent_gazebo`。

该包在 Gazebo 中创建一个智能体，包含以下传感器：
- 二维激光雷达（LiDAR）
- 前向 RGB 相机
- 后向 RGB 相机
- 深度相机

同时提供一个监控节点，用于检查传感器话题是否持续发布数据。
当前版本适配 `gz sim`（Gazebo Harmonic / Gazebo Sim 8.x）+ `ros_gz`。
启动文件已内置时钟桥接：`/world/toposlam_validation/clock -> /clock`。

## 目录结构

- `src/smart_agent_gazebo/urdf/smart_agent.urdf.xacro`：机器人模型与 Gazebo 传感器配置
- `src/smart_agent_gazebo/models/smart_agent/model.sdf`：Gazebo Sim 使用的智能体模型
- `src/smart_agent_gazebo/launch/spawn_agent.launch.py`：Gazebo 启动与模型加载
- `src/smart_agent_gazebo/smart_agent_gazebo/sensor_monitor.py`：传感器话题监控节点

## 依赖安装（Humble + Gazebo Sim）

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

## 主要话题

- `/agent/lidar/scan`
- `/agent/lidar/scan/points`（兼容点云输出）
- `/agent/front_camera/image_raw`
- `/agent/rear_camera/image_raw`
- `/agent/depth_camera/image_raw`

## 数据落盘（图片 + 雷达bin）

启动 `spawn_agent_with_record.launch.py` 后会自动运行 `data_recorder` 节点。
默认每 2 秒保存一次“时间对齐后的多传感器快照”，目录结构如下：

- `~/agent_records/step1/front_rgb/image.png`
- `~/agent_records/step1/rear_rgb/image.png`
- `~/agent_records/step1/depth/image.png`（16位深度图）
- `~/agent_records/step1/lidar/scan.bin`
- `~/agent_records/step1/meta.json`（对齐时间差与文件说明）
  - 雷达优先使用 `/agent/lidar/scan`；若无则自动回退到 `/agent/lidar/scan/points`

后续按 `step2`、`step3` 递增保存，重启后会自动续号，避免覆盖旧数据。

其中雷达 `.bin` 采用 `float32` 连续存储，每个点 3 个值：

- `angle`
- `range`
- `intensity`
若回退到点云模式，则 `scan.bin` 保存 PointCloud2 原始字节流，具体字段写在 `meta.json`。

可用参数：

```bash
ros2 launch smart_agent_gazebo spawn_agent.launch.py recording_dir:=/your/path
```

## 编译

```bash
cd g:/vm/agent_ws
colcon build
```

## 运行

```bash
cd g:/vm/agent_ws
source install/setup.bash
ros2 launch smart_agent_gazebo spawn_agent.launch.py
```

可选启动文件：

```bash
# 不保存数据
ros2 launch smart_agent_gazebo spawn_agent_no_record.launch.py

# 保存 RGB/Depth 图片和 LiDAR bin
ros2 launch smart_agent_gazebo spawn_agent_with_record.launch.py
```

## 键盘控制（WASD + QE）

先启动仿真（任意一个 launch），再在新终端运行：

```bash
ros2 run smart_agent_gazebo keyboard_teleop
```

按键映射：

- `W/S`：前进 / 后退
- `A/D`：左移 / 右移
- `Q/E`：左转 / 右转（yaw）
- `Space` 或 `X`：急停

可调速度参数：

```bash
ros2 run smart_agent_gazebo keyboard_teleop --ros-args -p linear_speed:=1.0 -p angular_speed:=1.2
```

默认会加载 TopoSLAM 验证世界：

- `src/smart_agent_gazebo/worlds/toposlam_validation.world`

如需切换世界文件：

```bash
ros2 launch smart_agent_gazebo spawn_agent.launch.py world:=/绝对路径/your_world.world
```

## 快速检查

```bash
ros2 topic list | grep agent
ros2 topic hz /agent/lidar/scan
ros2 topic hz /agent/front_camera/image_raw
ros2 topic hz /agent/rear_camera/image_raw
ros2 topic hz /agent/depth_camera/image_raw
```

## 补充说明

- 若你使用的是 ROS2 Humble / Iron，以上结构可直接使用。
- 如果 `gz sim --versions` 输出为 `8.x`，请使用本工程当前 `ros_gz` 版本启动方式（不是 `gazebo_ros`）。
- 如需加入运动控制，可后续增加 `cmd_vel` 驱动、差速/全向底盘插件。
- 如需用于算法训练，可继续添加里程计、IMU、碰撞传感器与随机场景脚本。
- 新增的 TopoSLAM 验证世界包含回环走廊、房间开口和颜色地标，适合测试拓扑建图与回环识别可行性。
- export GZ_SIM_RESOURCE_PATH=$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models:$GZ_SIM_RESOURCE_PATH用来加载 TopoSLAM 验证世界中的模型
