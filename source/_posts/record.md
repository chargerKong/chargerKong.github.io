---
title: 错误记录
date: 2021-04-12 09:59:22
tags:
---

 

可以通过robot_state_publisher来带一个world file

有一个很好的项目，[ROBOTIS-GIT](https://github.com/ROBOTIS-GIT)/**[turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)**



## Invalid frame ID "front_caster_link" passed to canTransform argument source_frame

利用可视化model的时候, 发现没有从base_link到前后万向轮的转换，同时通过`tf2`的工具

```
os2 run tf2_tools view_frames 
```

生成的PDF文件为

![](wrong_tf2.png)

的确是缺少了`base_link`和前后万向轮的一个连接。

"robot_description" 参数定义了urdf文件的路径，它被 robot_state_publisher节点使用。该节点解析urdf文件后将各个frame的状态发布给tf. 因此在rviz里面就看到各个frame(link)之间的tf转换显示OK.否则会显示warning.

需要添加`joint_state_piblisher`

```
git clone 
```

然后在source	

再启动	



## TF_OLD_DATA  odom 没有

调整插件`libgazebo_ros_diff_drive.so`

```
<publish_wheel_tf>true</publish_wheel_tf> 
```





## TF_OLD_DATA ignoring data from the past for frame right_wheel_link at time 25.479000 according to authority Authority undetectable

经检查发现是tf变换冲突，可能是launch文件里面的joint_state_publisher发布的tf变换和插件发布的有冲突



## 接收不到雷达信息

之前我们只输入了一个队列长度作为参数就会导致QoS配置出问题最终导致无法接收消息，现在我们采用专门为传感器设计的QoS配置文件或者系统默认的配置文件就可以正常工作了。

错误

```
    sub = node.create_subscription(
        LaserScan, '/scan', lambda msg: node.get_logger().info('I get a msg'), 1)
```

修改为

```
    sub = node.create_subscription(
        LaserScan, '/scan', lambda msg: node.get_logger().info('I get a msg'), rclpy.qos.qos_profile_sensor_data)
```

## 编译foxy错误  Could not find RTI Connext - skipping 'rmw_connext_cpp'

6 packages had stderr output: qt_gui_cpp rmw_connext_cpp rmw_connext_shared_cpp ros1_bridge rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp



Nav案例問題

## [Err] [Joint.cc:297] EXCEPTION: Couldn't Find Child Link[camera_rgb_frame]

打開model文件，直接修改相關link，發現的確沒有關於camera_rgb_frame的link。先把關於camera_rgb_frame的joint刪除

## [Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://turtlebot3_waffle/meshes/left_tire.dae]

去[这里](https://github.com/ros-planning/navigation2/tree/main/nav2_system_tests/models/turtlebot3_burger/meshes)下载`left_tire.dae`和`right_tire.dae`到`/opt/ros/foxy/share/turtlebot3_gazebo/models/turtlebot3_waffle/meshes`下面



## [rviz2]: PluginlibFactory: The plugin for class 'rviz_common/Time' failed to load.



(zai roszhong 是很重要的机器人描述格式，是XML来描述，里面有一系列的标签和属性，利用这些标签就可以完成建模的工作， 可以模拟手臂的形状，小臂和大臂就相当于是连杆，刚体。要让刚体运行必须要一个关节，连接大臂和小臂，需要关节带动刚体，所以有两个部分，一个是lin，刚体部分，另外一个是关节joint ，机器人也是这样，需要joint来带动。所以在任意一个urdf文件中，都会是一系列的link和joint描述

除了机器人，其实对外界的物体也是建模的，比如这个桌子，比如有一个桌面，四个桌腿，也是可以通过五个link进行建模



joint_state_publisher:

监控各个关节的角度，实时位置，打包为自己的话题发布出去

robot_state_publisher

订阅joint_state_publisher的位置，封装，发布tf



ICP方法是用于找到一个R和T来找到两个集合的位姿变换，怎么又可以用于运动畸变去除？



## 链接libnabo库  undefined reference to `Nabo::NearestNeighbourSearch...

解决

```
target_link_libraries(imlsMatcher_node
   ${catkin_LIBRARIES}
   ${EXTERNAL_LIBS}
   /opt/ros/noetic/lib/libcsm.so
   ${libnabo_LIBRARIES} libnabo::nabo           -------------------这里没有添加
)
```



## Python 显示ply文件

```python
import open3d as o3d
# visualization of point clouds.
pcd = o3d.io.read_point_cloud('test.ply')
o3d.visualization.draw_geometries([pcd])
```



