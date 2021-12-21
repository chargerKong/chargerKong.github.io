---
title: tianbot ros2
date: 2021-09-24 15:59:23
tags: ros2
---

# 驱动

## 相机驱动

ROS1: [usb_cam](http://wiki.ros.org/action/fullsearch/usb_cam?action=fullsearch&context=180&value=linkto%3A"usb_cam")，V4L（video for linux）的相机驱动

ROS2: [ros2_v4l2_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera)，

## 单线雷达驱动

ROS1: [rplidar_ros](https://github.com/Slamtec/rplidar_ros)， 思岚科技

ROS2: [rplidar_ros](https://github.com/Slamtec/rplidar_ros)，思岚科技 



# 遗留问题

## STL可视化，rviz2和gazebo如何统一

rviz2 添加stl  package::

gazebo   model::



## joint_name_tianracer_description 没有用



```
def __init__(
        self, *,
        package: SomeSubstitutionsType,
        node_executable: SomeSubstitutionsType,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ) -> None:
```

## 启动 tianracer_tf.launch.py

虽然tf 和tf_static都可以得到数据，但是

tf_tools view_frame.py 却无法get到tf_static的数据

foxy是没有问题的，初步猜测，dashing的数据，foxy无法完全兼容



## serial_port 冲突

tianracer_core和rplidar启动的端口 名字冲突

