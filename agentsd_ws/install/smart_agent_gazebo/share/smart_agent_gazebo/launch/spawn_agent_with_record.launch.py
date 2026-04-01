from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('smart_agent_gazebo')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    default_world = PathJoinSubstitution([pkg_share, 'worlds', 'small_house.world'])
    model_path = PathJoinSubstitution([pkg_share, 'models'])
    world = LaunchConfiguration('world')
    recording_dir = LaunchConfiguration('recording_dir')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to world file',
    )
    recording_dir_arg = DeclareLaunchArgument(
        'recording_dir',
        default_value='/home/test',
        description='Directory used to save rgb/depth images and lidar bin files',
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[model_path, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[model_path, ':', EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value='')],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/world/small_house/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/world/small_house/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/agent/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/agent/lidar/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/agent/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/world/small_house/model/smart_agent/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/model/smart_agent/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/model/smart_agent/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/agent/front_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/agent/rear_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/agent/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            ('/world/small_house/clock', '/clock'),
            ('/agent/front_camera/image', '/agent/front_camera/image_raw'),
            ('/agent/rear_camera/image', '/agent/rear_camera/image_raw'),
            ('/agent/depth_camera/image', '/agent/depth_camera/image_raw'),
            ('/world/small_house/model/smart_agent/link/base_link/sensor/imu_sensor/imu', '/agent/imu_scoped_world'),
            ('/model/smart_agent/link/base_link/sensor/imu_sensor/imu', '/agent/imu_scoped_model'),
        ],
    )

    monitor = Node(
        package='smart_agent_gazebo',
        executable='sensor_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    recorder = Node(
        package='smart_agent_gazebo',
        executable='data_recorder',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_dir': recording_dir},
            {'save_interval_sec': 1.0},
            {'sync_tolerance_sec': 0.6},
            {'require_lidar': True},
            {'imu_save_hz': 20.0},
            {'imu_window_sec': 0.1},
            {'pose_topic': '/world/small_house/dynamic_pose/info'},
            {'run_prefix': 'run'},
        ],
    )

    return LaunchDescription([
        world_arg,
        recording_dir_arg,
        gz_resource_path,
        ign_resource_path,
        gz_sim,
        bridge,
        monitor,
        recorder,
    ])
