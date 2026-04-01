# IMU 数据说明（smart_agent_gazebo / small_house）

本文档基于当前工程配置，说明 IMU 的数据来源、坐标系定义、惯导数据采用的坐标系、以及与 RGB 对齐保存的规则。

## 1. 当前工程中的 IMU 数据链路

### 1.1 传感器定义（模型侧）

在 `models/smart_agent/model.sdf` 中：

- 传感器类型：`imu`
- 传感器名：`imu_sensor`
- 输出 topic：`/agent/imu`
- 更新频率：`100 Hz`
- `gz_frame_id`：`base_link`
- 传感器未单独设置 `<pose>`，因此默认与 `base_link` 坐标系重合

结论：当前 IMU 的“体坐标系”就是机器人 `base_link` 坐标系。

### 1.2 桥接（Gazebo -> ROS 2）

在 `launch/spawn_agent_headless_web.launch.py` 中，IMU 相关桥接包括：

- `/agent/imu@sensor_msgs/msg/Imu@gz.msgs.IMU`
- `/world/small_house/model/smart_agent/link/base_link/sensor/imu_sensor/imu@...`
- `/model/smart_agent/link/base_link/sensor/imu_sensor/imu@...`

并 remap 为：

- `/agent/imu_scoped_world`
- `/agent/imu_scoped_model`

因此实际可用 IMU ROS 话题通常有：

- `/agent/imu`
- `/agent/imu_scoped_world`
- `/agent/imu_scoped_model`

### 1.3 录制器订阅策略

`data_recorder` 会订阅多个 IMU topic，并且每个 topic 同时建立：

- `RELIABLE` 订阅
- `BEST_EFFORT` 订阅

这是为了最大化兼容不同 QoS 组合，避免“topic 可见但消息收不到”的情况。

---

## 2. 坐标系定义

## 2.1 世界坐标系（World / Inertial Frame, 记作 `W`）

Gazebo world 是右手系、`Z` 轴向上。工程中重力为：

```xml
<gravity>0 0 -9.8</gravity>
```

因此可认为：

- `+Z`：向上
- 重力方向：沿 `-Z`

### 2.2 机器人体坐标系（Body Frame, 记作 `B`）

当前 `B` 与 `base_link` 重合。按 ROS 常用机器人约定：

- `+X`：前方
- `+Y`：左侧
- `+Z`：上方

由于 IMU 传感器与 `base_link` 重合，IMU 原始角速度/线加速度的分量方向即按该体坐标系解释。

### 2.3 IMU 消息中的坐标参考

`sensor_msgs/msg/Imu` 中：

- `angular_velocity`：通常在 IMU 体坐标系 `B` 下表示（单位 `rad/s`）
- `linear_acceleration`：通常在 IMU 体坐标系 `B` 下表示（单位 `m/s^2`）
- `orientation`（四元数）：表示机体姿态相对世界系的旋转（实际使用时建议通过静态测试确认方向约定）

---

## 3. 惯导数据采用的坐标系（你关心的重点）

如果做惯导积分（strapdown），推荐采用：

- **原始测量**：在体坐标系 `B`（IMU / base_link）
- **积分状态（速度/位置）**：在世界坐标系 `W`

典型流程：

1. 读取 `gyro_B`、`acc_B`
2. 用姿态四元数得到旋转矩阵 `R_WB`
3. 将加速度从 `B` 变换到 `W`：`acc_W = R_WB * acc_B`
4. 按传感器语义处理重力项，再进行速度/位置积分

注意：`linear_acceleration` 是否已去重力，取决于上游实现。  
建议做静态测试（机器人静止、水平放置）：

- 若 `acc_z` 约为 `+9.8` 或 `-9.8`，说明重力未被去除（需在世界系补偿）
- 若三轴接近 `0`，说明已近似去重力

---

## 4. 当前保存格式与 IMU 字段对应

## 4.1 全局 IMU 流：`imu_stream.csv`

每条记录字段：

```text
index,time_ns,
ang_vel_x,ang_vel_y,ang_vel_z,
lin_acc_x,lin_acc_y,lin_acc_z,
ori_x,ori_y,ori_z,ori_w
```

含义：

- `time_ns`：IMU 时间戳（纳秒）
- `ang_vel_*`：角速度（体坐标系）
- `lin_acc_*`：线加速度（体坐标系）
- `ori_*`：姿态四元数

## 4.2 每个 step 的对齐信息：`steps/stepXXXXXX/meta.json`

与 IMU 相关核心字段：

- `imu_anchor`
  - `stream_index`：与本次 RGB 对齐的 IMU 样本索引
  - `time_ns`：该 IMU 样本时间
  - `delta_ms`：IMU 与 RGB 锚点时间差
- `alignment_valid.imu`
  - 判断条件：`abs(delta_ms) <= imu_align_tolerance_sec`

## 4.3 每个 step 的 IMU 窗口：`steps/stepXXXXXX/imu/window.npy`

形状：`[N, 13]`，列定义：

1. `rel_ms`（相对 RGB 时间，毫秒）
2. `ang_vel_x`
3. `ang_vel_y`
4. `ang_vel_z`
5. `lin_acc_x`
6. `lin_acc_y`
7. `lin_acc_z`
8. `ori_x`
9. `ori_y`
10. `ori_z`
11. `ori_w`
12. `stream_index`
13. `sample_delta_ms`（该行样本与 RGB 锚点时间差）

---

## 5. RGB 与 IMU 的对齐规则（当前实现）

当前实现是“**以前视 RGB 时间为锚点**”：

1. 先取 `front_rgb` 的时间戳 `rgb_time_ns`
2. 在 IMU 流里找最近样本，得到 `imu_anchor`
3. 再按 `imu_save_hz` 和 `imu_window_sec` 在锚点前后构建固定时间窗口

默认参数（当前工程）：

- `imu_save_hz = 100`
- `imu_window_sec = 0.1`
- `imu_align_tolerance_sec = 0.01`

这意味着 IMU 对齐是围绕 RGB 进行的，便于“图像主导”的训练/评估流水线。

---

## 6. 快速自检命令

```bash
# 1) 看 IMU topic 是否存在
ros2 topic list | grep -i imu

# 2) 看 IMU 是否有频率
ros2 topic hz /agent/imu
ros2 topic hz /agent/imu_scoped_world
ros2 topic hz /agent/imu_scoped_model

# 3) 看 IMU 消息内容（含 frame_id）
ros2 topic echo /agent/imu --once

# 4) 查看最新录制的 IMU CSV 是否有数据
RUN_DIR=$(ls -dt ~/agent_records/run_* | head -1)
wc -l "$RUN_DIR/imu_stream.csv"
head -5 "$RUN_DIR/imu_stream.csv"
```

如果 IMU topic 存在但始终无数据，请确认 world 中已启用 IMU 系统插件：

```xml
<plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>
```

---

## 7. 一句话总结

- **IMU 体坐标系**：`base_link`（X 前、Y 左、Z 上）  
- **惯导积分建议坐标系**：原始测量在 `B`，状态积分在 `W`  
- **当前保存策略**：以 RGB 时间为锚点，保存 IMU 锚点与窗口，并在 `meta.json` 记录对齐关系
