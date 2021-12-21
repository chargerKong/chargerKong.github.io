---
title: ros2 launch 使用记录说明
date: 2021-10-26 09:59:22
tags: ros2-launch
---

# IncludeLaunchDescription

##  condition、launch_arguments

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server'))，
            launch_arguments={'model': model}.items(),
            
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
    ])
```

# ExecuteProcess

```
    gzserver = launch.actions.ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )
```

# xacro

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    world = os.path.join(get_package_share_directory('robot_simulation'), 'worlds', 'turtlebot3_world.world')
    # world = os.path.join(get_package_share_directory('robot_simulation'), 'worlds', 'room1.world')
    # urdf = os.path.join(get_package_share_directory('hoverboard_mvp'), 'urdf', 'hoverboard.urdf')
    urdf_file = os.path.join(get_package_share_directory('mobile_robot_models'), 'urdf/quimera_robot.urdf.xacro')
    # doc = xacro.parse(open(urdf_file))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    robot = xacro.process(urdf_file)
    params = {'robot_description': robot}

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                        '-entity', 'cartpole'],
            output='screen')

    ])
```

# Rviz on_exit

```
launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                (
                    "carla/ego_vehicle/spectator_pose",
                    "/carla/ego_vehicle/rgb_view/control/set_transform"
                )
            ],
            arguments=[
                '-d', os.path.join(get_package_share_directory(package_name), rviz2_config_path)],
            on_exit=launch.actions.Shutdown()
        )
```

rclcpp::QoS(1).transient_local());

#

