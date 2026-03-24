from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("smart_agent_gazebo")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")
    default_world = PathJoinSubstitution([pkg_share, "worlds", "toposlam_validation.world"])
    model_path = PathJoinSubstitution([pkg_share, "models"])

    world = LaunchConfiguration("world")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    cmd_max_hz = LaunchConfiguration("cmd_max_hz")
    recording_dir = LaunchConfiguration("recording_dir")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Absolute path to world file",
    )
    host_arg = DeclareLaunchArgument(
        "web_host",
        default_value="0.0.0.0",
        description="FastAPI listen host",
    )
    port_arg = DeclareLaunchArgument(
        "web_port",
        default_value="8000",
        description="FastAPI listen port",
    )
    cmd_max_hz_arg = DeclareLaunchArgument(
        "cmd_max_hz",
        default_value="6.0",
        description="Max accepted cmd_vel update rate (Hz)",
    )
    recording_dir_arg = DeclareLaunchArgument(
        "recording_dir",
        default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "agent_records"]),
        description="Directory used by recorder when started from web button",
    )

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[model_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[model_path, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
    )
    ign_render_engine = SetEnvironmentVariable(
        name="IGN_RENDER_ENGINE",
        value="ogre2",
    )
    gz_render_engine = SetEnvironmentVariable(
        name="GZ_RENDER_ENGINE",
        value="ogre2",
    )
    qt_offscreen = SetEnvironmentVariable(
        name="QT_QPA_PLATFORM",
        value="offscreen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": ["-r -s --headless-rendering ", world]}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/world/toposlam_validation/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/agent/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/agent/lidar/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/world/toposlam_validation/model/smart_agent/link/lidar_link/sensor/lidar_sensor/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/world/toposlam_validation/model/smart_agent/link/lidar_link/sensor/lidar_sensor/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/model/smart_agent/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/agent/front_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/agent/rear_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/agent/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        remappings=[
            ("/world/toposlam_validation/clock", "/clock"),
            (
                "/world/toposlam_validation/model/smart_agent/link/lidar_link/sensor/lidar_sensor/scan",
                "/agent/lidar/scan",
            ),
            (
                "/world/toposlam_validation/model/smart_agent/link/lidar_link/sensor/lidar_sensor/scan/points",
                "/agent/lidar/scan/points",
            ),
            ("/agent/front_camera/image", "/agent/front_camera/image_raw"),
            ("/agent/rear_camera/image", "/agent/rear_camera/image_raw"),
            ("/agent/depth_camera/image", "/agent/depth_camera/image_raw"),
        ],
    )

    monitor = Node(
        package="smart_agent_gazebo",
        executable="sensor_monitor",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    lidar_fallback = Node(
        package="smart_agent_gazebo",
        executable="depth_lidar_fallback",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"depth_topic": "/agent/depth_camera/image_raw"},
            {"scan_topic": "/agent/lidar/scan"},
            {"horizontal_fov_rad": 1.396},
            {"sample_count": 360},
            {"publish_rate_hz": 10.0},
            {"range_min": 0.12},
            {"range_max": 15.0},
        ],
    )

    web_gateway = Node(
        package="smart_agent_gazebo",
        executable="web_gateway",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"host": web_host},
            {"port": web_port},
            {"jpeg_quality": 80},
            {"depth_max_m": 12.0},
            {"cmd_timeout_sec": 0.5},
            {"cmd_max_hz": ParameterValue(cmd_max_hz, value_type=float)},
            {"record_output_dir": recording_dir},
            {"record_save_interval_sec": 2.0},
            {"record_sync_tolerance_sec": 0.6},
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            host_arg,
            port_arg,
            cmd_max_hz_arg,
            recording_dir_arg,
            gz_resource_path,
            ign_resource_path,
            ign_render_engine,
            gz_render_engine,
            qt_offscreen,
            gz_sim,
            bridge,
            monitor,
            lidar_fallback,
            web_gateway,
        ]
    )
