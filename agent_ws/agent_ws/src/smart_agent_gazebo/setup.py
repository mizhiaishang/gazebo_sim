import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_agent_gazebo'


def collect_model_data_files():
    data_files = []
    model_root = 'models'
    for root, _, files in os.walk(model_root):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, root)
        src_files = [os.path.join(root, file_name) for file_name in files]
        data_files.append((install_dir, src_files))
    return data_files


model_data_files = collect_model_data_files()

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Some imported AWS DAE files reference textures using '../../../../photos/...'
        # from within .../share/<pkg>/models/<model>/meshes, which resolves to .../share/photos.
        (os.path.join('share', 'photos'), glob('photos/*')),
        # Keep a package-local copy for convenience / debugging.
        (os.path.join('share', package_name, 'photos'), glob('photos/*')),
    ] + model_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 Gazebo smart agent with lidar, RGB cameras, and depth camera.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_monitor = smart_agent_gazebo.sensor_monitor:main',
            'data_recorder = smart_agent_gazebo.data_recorder:main',
            'keyboard_teleop = smart_agent_gazebo.keyboard_teleop:main',
            'web_gateway = smart_agent_gazebo.web_gateway:main',
            'depth_lidar_fallback = smart_agent_gazebo.depth_lidar_fallback:main',
        ],
    },
)
