# smart_agent_gazebo workspace

This workspace provides one ROS2 package: `smart_agent_gazebo`.

It includes:
- A Gazebo robot model with:
  - 2D LiDAR (`/agent/lidar/scan`)
  - Front RGB camera (`/agent/front_camera/image_raw`)
  - Rear RGB camera (`/agent/rear_camera/image_raw`)
  - Depth camera image (`/agent/depth_camera/image_raw`)
- A monitor node: `sensor_monitor` (checks if all sensor topics are active)
- A launch file: `spawn_agent.launch.py`

## Build

```bash
cd g:/vm/agent_ws
colcon build
```

## Run

```bash
cd g:/vm/agent_ws
source install/setup.bash
ros2 launch smart_agent_gazebo spawn_agent.launch.py
```

## Quick topic checks

```bash
ros2 topic list | grep agent
ros2 topic hz /agent/lidar/scan
ros2 topic hz /agent/front_camera/image_raw
ros2 topic hz /agent/rear_camera/image_raw
ros2 topic hz /agent/depth_camera/image_raw
```
