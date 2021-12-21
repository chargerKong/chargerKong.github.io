---
title: 配置sensors-navigation2
date: 2021-09-18 15:59:23
tags: nav2
---

# 配置传感器

在本指南中，我们将讨论传感器在安全导航机器人方面的重要性以及如何使用 Nav2 设置传感器。在本教程的前半部分，我们将简要介绍 Nav2 中常用的传感器和常见的传感器消息。接下来，我们将在我们之前构建的模拟机器人 sam_bot 上添加一个基本的传感器设置。最后，我们将通过在 RViz 中可视化它们来验证 sam_bot 的模拟传感器消息。

一旦在机器人上安装了传感器，它们读取的内容就可以用于地图绘制、定位和感知任务。在本指南的后半部分，我们将首先讨论建图和定位是如何使用传感器数据的。然后，我们还将看看 Nav2 的其中一个包，`nav2_costmap_2d`，它生成最终将用于 Nav2 路径规划的成本地图。我们将为这个包设置基本配置参数，以便它从 sam_bot 正确获取传感器信息。最后，我们在 RViz 中可视化生成的costmaps来验证其接收到的数据。

## Sensor Introduction[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#sensor-introduction)

移动机器人配备了大量传感器，使他们能够看到和感知周围的环境。 这些传感器获取的信息可用于构建和维护环境地图、在地图上定位机器人以及查看环境中的障碍物。 这些任务对于在动态环境中安全有效地导航机器人至关重要。

常用传感器的示例有激光雷达、雷达、RGB 摄像头、深度摄像头、IMU 和 GPS。 为了标准化这些传感器的消息格式并允许供应商之间更容易的互操作，ROS 提供了 sensor_msgs 包来定义通用传感器接口。 这也允许用户使用任何传感器供应商，只要它遵循 sensor_msgs 中的标准格式。 在下一小节中，我们将介绍导航中一些常用的消息，即 sensor_msgs/LaserScan、sensor_msgs/PointCloud2、sensor_msgs/Range 和 sensor_msgs/Image。

除了 sensor_msgs 包，还有你应该知道的radar_msgs 和vision_msgs 标准接口。 radar_msgs 定义了特定于雷达的传感器的消息，而 vision_msgs 包定义了计算机视觉中使用的消息，例如对象检测、分割和其他机器学习模型。 此包支持的消息是 vision_msgs/Classification2D、vision_msgs/Classification3D、vision_msgs/Detection2D 和 vision_msgs/Detection3D，仅举几例。

> For more information, see the API documentation of [sensor_msgs](http://wiki.ros.org/sensor_msgs), [radar_msgs](http://wiki.ros.org/radar_msgs), and [vision_msgs](http://wiki.ros.org/vision_msgs).

您的物理机器人的传感器可能已经为它们编写了 ROS 驱动程序（例如，一个 ROS 节点连接到传感器，将数据填充到消息中，并将它们发布给您的机器人使用），这些驱动程序遵循 sensor_msgs 包中的标准接口。 sensor_msgs 包使您可以轻松使用来自不同制造商的许多不同传感器。 然后像 Nav2 这样的通用软件包可以读取这些标准化消息并执行独立于传感器硬件的任务。 在 sam_bot 等模拟机器人上，Gazebo 具有传感器插件，这些插件也会在 sensor_msgs 包之后发布它们的信息。

### Common Sensor Messages

在这个小节中，我们讨论一些你在配置Nav2的过程中可能遇到的常见的类型。我们会对每一个传感器都一些简单的介绍，在 Gazebo 中模拟的图像以及 RViz 中传感器读数的相应可视化。

> There are other types of `sensor_msgs` aside from the ones listed below. The complete list of messages and their definitions can be found in the [sensor_msgs documentation](http://wiki.ros.org/sensor_msgs).

#### sensor_msgs/LaserScan[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#sensor-msgs-laserscan)

此消息表示来自平面激光测距仪的单次扫描。 此消息在 slam_toolbox 和 nav2_amcl 中用于定位和建图，或在 nav2_costmap_2d 中用于感知物体。

![../../_images/sensor_laserscan.png](https://navigation.ros.org/_images/sensor_laserscan.png)

#### sensor_msgs/PointCloud2[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#sensor-msgs-pointcloud2)

此消息包含 3D 点的集合，以及有关每个点的可选附加信息。 这可以来自 3D 激光雷达、2D 激光雷达、深度相机或更多。

![../../_images/sensor_pointcloud2.png](https://navigation.ros.org/_images/sensor_pointcloud2.png)

#### sensor_msgs/Range[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#sensor-msgs-range)

这是一个来自主动测距仪的单距离读数，主动测距仪发射能量，并报告沿测量距离的弧线有效的一个距离读数。声纳、红外传感器或一维测距仪都是使用这种信息的传感器。

![../../_images/sensor_range.png](https://navigation.ros.org/_images/sensor_range.png)

#### sensor_msgs/Image[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#sensor-msgs-image)

这表示传感器读数从RGB或深度相机，对应于RGB或范围值。

![../../_images/sensor_image.png](https://navigation.ros.org/_images/sensor_image.png)

## Simulating Sensors using Gazebo[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#simulating-sensors-using-gazebo)

和之前的教程类似，我们现在给`sam_bot`添加一个雷达传感器和深度相机传感器，

### Adding Gazebo Plugins to a URDF[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#adding-gazebo-plugins-to-a-urdf)

首先为`sam_bot`添加雷达传感器，打开URDF文件，在`<robot>`标签之前添加以下内容

```xml
<link name="lidar_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.125"/>
    <inertia ixx="0.001"  ixy="0"  ixz="0" iyy="0.001" iyz="0" izz="0.001" />
  </inertial>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
       <cylinder radius="0.0508" length="0.055"/>
    </geometry>
  </collision>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
       <cylinder radius="0.0508" length="0.055"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.12" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>5</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.000000</resolution>
          <min_angle>0.000000</min_angle>
          <max_angle>6.280000</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.120000</min>
        <max>3.5</max>
        <resolution>0.015000</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="scan" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

在上面的代码片段中，我们创建了一个`lidar_link`，这个link被referenced到gazebo_ros_ray_sensor插件。我们设置了模拟雷达的一些参数，并且设置了topic为scan，输出的消息类型是`sensor_msgs/LaserScan`

接下来，为`sam_bot`添加深度摄像机

```xml
<link name="camera_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.015 0.130 0.022"/>
    </geometry>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.015 0.130 0.022"/>
    </geometry>
  </collision>

  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.035"/>
    <inertia ixx="0.001"  ixy="0"  ixz="0" iyy="0.001" iyz="0" izz="0.001" />
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.215 0 0.05" rpy="0 0 0"/>
</joint>

<link name="camera_depth_frame"/>

<joint name="camera_depth_joint" type="fixed">
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi}"/>
  <parent link="camera_link"/>
  <child link="camera_depth_frame"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="camera">
      <horizontal_fov>1.047198</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <baseline>0.2</baseline>
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <frameName>camera_depth_frame</frameName>
      <pointCloudCutoff>0.5</pointCloudCutoff>
      <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
      <distortionK1>0</distortionK1>
      <distortionK2>0</distortionK2>
      <distortionK3>0</distortionK3>
      <distortionT1>0</distortionT1>
      <distortionT2>0</distortionT2>
      <CxPrime>0</CxPrime>
      <Cx>0</Cx>
      <Cy>0</Cy>
      <focalLength>0</focalLength>
      <hackBaseline>0</hackBaseline>
    </plugin>
  </sensor>
</gazebo>
```

和雷达传感器类似，我们创建了一个`camera_link`链接到 `gazebo_ros_camera` 插件上，我们还创建了一个camera_depth_frame，它被附加到camera_link，并将被设置为深度相机插件的`<frameName>`坐标系名字。

我们还配置插件，使其将sensor_msgs/Image和sensor_msgs/PointCloud2消息分别发布到/depth_camera/image_raw和/depth_camera/points主题。最后，我们还设置了深度相机的其他基本配置属性。

### Launch and Build Files

为了验证传感器的设置是否正确，以及它们是否能够看到我们环境中的物体，让我们在带有物体的Gazebo世界中启动`sam_bot`。让我们用`sam_bot`的传感器范围内的一个立方体和一个球体创建一个Gazebo世界，这样我们就可以验证它是否能正确地看到物体。

首先在项目的根目录创建一个world文件夹，并且创建一个名字为`my_world.sdf`的文件在文件夹里，复制这些内容到文件里面

```xml
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='unit_box'>
      <pose>1.51271 -0.181418 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <model name='unit_sphere'>
      <pose>-1.89496 2.36764 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <state world_name='default'>
      <sim_time>0 0</sim_time>
      <real_time>0 0</real_time>
      <wall_time>1626668720 808592627</wall_time>
      <iterations>0</iterations>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box'>
        <pose>1.51272 -0.181418 0.499995 0 1e-05 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>1.51272 -0.181418 0.499995 0 1e-05 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0.010615 -0.006191 -9.78231 0.012424 0.021225 1.8e-05</acceleration>
          <wrench>0.010615 -0.006191 -9.78231 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_sphere'>
        <pose>-0.725833 1.36206 0.5 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-0.944955 1.09802 0.5 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>3.17226 -5.10401 6.58845 0 0.739643 2.19219</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
```

我们接下来修改一下launch文件，打开launch文件，在`generate_launch_description()`里面添加world文件的路径

```python
world_path=os.path.join(pkg_share, 'world/my_world.sdf'),
```

然后在一行 `launch.actions.ExecuteProcess(cmd=['gazebo',...` 添加world的参数

```python
launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
```

我们也要添加world文件夹到CMakeLists.txt中，像下面这样子

```
install(
  DIRECTORY src launch rviz config world
  DESTINATION share/${PROJECT_NAME}
)
```

### Build, Run and Verification[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#build-run-and-verification)

到根目录下，输入一下内容

```
colcon build
. install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

RViz和Gazebo将在sam_bot出现的情况下启动。在Gazebo窗口中，我们创建的世界应该被启动，sam_bot应该在那个世界中生成。你现在可以用360度激光雷达和深度相机观察sam_bot了，如下图所示。

![../../_images/gazebo_sensors.png](https://navigation.ros.org/_images/gazebo_sensors.png)

在RViz窗口，我们可以验证我们是否正确地建模了我们的传感器，以及我们新添加的传感器的转换是否正确:

![../../_images/rviz_sensors.png](https://navigation.ros.org/_images/rviz_sensors.png)

为了可以在Rviz中显示出 `sensor_msgs/LaserScan` 消息，点击Rviz左下角的add按钮，选择by topic.

![../../_images/add_topic_laserscan.png](https://navigation.ros.org/_images/add_topic_laserscan.png)

设置`Reliability Policy` 为`Best Effort`，并且设置`size`为0.1可以使点看的更加清晰，

![../../_images/demo_laserscan_rviz.png](https://navigation.ros.org/_images/demo_laserscan_rviz.png)

为了可视化 `sensor_msgs/Image` 和 `sensor_msgs/PointCloud2`, 和上面做类似的操作即可

![../../_images/add_topic_image_pointcloud2.png](https://navigation.ros.org/_images/add_topic_image_pointcloud2.png)	

在添加了`depth_camera/image_raw`话题之后，设置`Reliability Policy` 为`Best Effort`，就可以应该看见图像窗口在Rviz中出现

![../../_images/demo_image_rviz.png](https://navigation.ros.org/_images/demo_image_rviz.png)

你也可以看见如下形状的点云

![../../_images/demo_pointcloud2_rviz.png](https://navigation.ros.org/_images/demo_pointcloud2_rviz.png)

## Mapping and Localization

现在我们已经有了一个多个传感器配置好的机器人，我们可以利用已经获得的传感器信息来建立此环境的地图，并且在地图中定位。`slam_toolbox`是一个可以在2D中做 SLAM 的强大功能包，他也是Nav2官方所支持的一种SLAM库。除了`slam_toolbox`之外，还有`nav2_amcl`也可以做到定位功能， 该功能包实现了自适应蒙特卡罗定位(AMCL)，估计机器人在地图上的位置和方向。其他技术请查看Nav2的文档获取更多信息。

 `slam_toolbox` 和 `nav2_amcl` 都可以使用雷达扫描传感器来感受机器人的环境信息，因此，我们必须保证他们订阅了发布 `sensor_msgs/LaserScan` 消息类型的话题名字。这个是可以通过设置`scan_topic`参数来更新，此参数的默认值即为`/scan`。我们的机器人`sam_bot`之前设置的就是`/scan`

对参数更深度的讲解将会比较复杂，建议去查看官方文档

> For the complete list of configuration parameters of `slam_toolbox`, see the [Github repository of slam_toolbox](https://github.com/SteveMacenski/slam_toolbox#readme).For the complete list of configuration parameters and example configuration of `nav2_amcl`, see the [AMCL Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-amcl.html).

您还可以参考(SLAM) [navigation While Mapping](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)指南，了解如何在SLAM中使用Nav2。通过在RViz中可视化地图和机器人的姿态来验证slam_toolbox和nav2_amcl是否已经正确设置，这与上一节中显示的内容类似。

## Costmap 2D

costmap 2D 功能包可以根据传感器的信息，**以栅格地图的形式**来表达出机器人周围的环境情况，在栅格地图中的cell值是处于0-254之间的，他表示穿过此区域的一个代价值。如果一个代价值=0，这意味着穿过此区域是很自由的 ，如果代价值是254，这就是表示此区域是一定被占据的。这些值的信息会被用于导航算法中，可以实现避开作为势场的一个障碍物。代价地图（costmap）的实现在Nav2中的`nav2_costmap_2d`功能包中

此代价地图是由多层组成的，每一层都有一些特定的功能来计算此区域的一个代价值。

这个包由以下几层组成，但是是基于插件的，以允许自定义和使用新的层：

- static layer: 表示地图部分的代价地图，从发布到/map topic的消息(如由SLAM生成的消息)中获得
- range layer: 处理包含声纳和红外传感器提供的信息。
- obstacle layer: 包括一些对象，这些对象是由发布LaserScan和PointCloud2消息的传感器检测到的
- voxel layer: 和障碍层十分像，他也可以使用LaserScan或者PointCloud2的传感器信息，但是处理的是3D数据
- inflation layer:  膨胀层表示围绕致命障碍物的附加成本值，这样我们的机器人就可以避免由于机器人的几何形状而进入障碍物

下一节中，我们会讨论在 `nav2_costmap_2d`中各个层的参数配置

这一些层已经通过插件接口整合到了costmap中，然后使用用户指定的膨胀半径进行膨胀(如果启用了膨胀层)。对于更深层次的关于costmap的概念，可以查看 [ROS1 costmap_2D documentation](http://wiki.ros.org/costmap_2d). 注意，`nav2_costmap_2d`包主要是ROS1导航堆栈版本的一个简单的ROS2接口，需要对ROS2的支持做一些小的修改，并添加一些新的层插件。

### Configuring nav2_costmap_2d[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#configuring-nav2-costmap-2d)

在此小节，我们给了一个`nav2_costmap_2d`的配置案例，这里包含这以上所述的一些层的配置。我们同时设置obstacle 和 voxel层是使用LaserScan的消息格式来发布/scan，同时我们也设置了基本的参数来定义如何去检测障碍物，并且将其可视化在costmap中。注意这个配置要包含在Nav2的配置文件中

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: false
      rolling_window: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
     
```

在上面的设置中，我们定义了两个不同的costmap参数：global_costmap和local_costmap，global_costmap主要是用于长时间的全局的规划，而local_costmap是用于局部的，避免碰撞

我们使用什么层将会设置在global_costmap或者local_costmap里面的plugins中，这些值被设置为地图层名称的列表，也作为plugin后面参数的的名称空间

注意，这个列表中的每一层/命名空间都必须有一个plugin参数(如第15、18、32、52和68行所示)，该参数定义了要为该特定层加载的插件的类型。

static layer 参数

- map_subscribe_transient_local：设置map topic的 QoS 设置
- map_topic： 要订阅的地图topic，默认为/map

对于障碍物层(第17-30行)，我们在observation_sources参数(第20行)下定义其传感器源为scan，其参数在第22-30行设置。我们将它的topic参数设置为发布定义的传感器源的topic，并根据它将使用的传感器源设置data_type。在我们的配置中，障碍物层将使用激光雷达传感器发布的LaserScan消息类型的/scan话题消息。

注意，obstacle layer 和 voxel layer can 都可以使用LaserScan消息类型或者PointCloud2消息类型作为他们的data_type，但是他们的默认值是LaserScan。下面的代码片段展示了同时使用LaserScan和PointCloud2作为传感器来源的一个例子。这可能对你的实体机器人特别有用处。

```
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan pointcloud
  scan:
    topic: /scan
    data_type: "LaserScan"
  pointcloud:
    topic: /depth_camera/points
    data_type: "PointCloud2"
```

obstacle layer的其他参数， 

- max_obstacle_height: 传感器能感知的最高高度
- min_obstacle_height: 传感器能感知的最小高度，默认为0。这里没有设置
- clearing: 判定障碍物是否从costmap里面移除掉
- marking: 参数用于设置插入的障碍物是否被标记到代价地图中。
- obstacle_max_range: 标记代价地图中障碍物的最大距离
- obstacle_min_range: 标记代价地图中障碍物的最小距离

inflation参数：

- cost_scaling_factor：膨胀半径上的指数衰减因子。
- inflation_radius: 致命障碍物周围膨胀的半径值。

voxel layer 参数

- publish_voxel_map: 是否发布3D voxel网格
- z_resolution: 体素在高度上的分辨率
- z_voxels: 每列中体素的数量
- mark_threshold: 在栅格占用网格中标记为已占用的最小体素数。
- observation_sources: 观察环境的传感器名字，我们设置为scan
- scan: 设置topic的名字，data_type

注意，我们没有在配置中使用range layer，但它可能对你自己的机器人设置有用。

range layer参数：它的基本参数是' topics '， ' input_sensor_type '和' clear_on_max_reading '参数。要订阅的范围主题在' topics '参数中定义。' input_sensor_type '被设置为' ALL '， ' VARIABLE '或' FIXED '。' clear_on_max_reading '是一个布尔参数，用于设置是否清除最大范围内的传感器读数。如果您需要设置它，请查看下面链接中的配置指南。

> For more information on `nav2_costmap_2d` and the complete list of layer plugin parameters, see the [Costmap 2D Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-costmaps.html).

### Build, Run and Verification[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#id1)

我们首先要启动launch文件，此文件可以发布从base_link到其他传感器的transform。此文件也会启动一个Gazebo来模拟我们的实体机器人，并且从差分驱动插件提供了一个odom到base_link的变换。此文件还会启动一个Rviz进行可视化

然后我们会启动slam_toolbox去发布map话题，并且发布map到odom的变换。map => odom变换是Nav2系统的主要需求之一。然后，发布在/map主题上的消息将由global_costmap的静态层使用。

在我们正确设置机器人URDF文件、里程传感器和必要的transform之后，我们最终将启动Nav2系统。目前，我们将只探索Nav2的成本地图生成系统。启动Nav2后，我们将在RViz中可视化成本图，以确认我们的输出。

#### Launching Description Nodes, RViz and Gazebo[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#launching-description-nodes-rviz-and-gazebo)

首先启动launch文件

```
colcon build
. install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

RViz和Gazebo现在应该在sam_bot出现的情况下启动。回想一下，base_link => sensors transform现在由robot_state_publisher发布，而odom => base_link transform则由我们的Gazebo插件发布。这两个转换现在应该显示在RViz没有错误。

#### Launching slam_toolbox[¶](https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#launching-slam-toolbox)

先安装slam_toolbox

```
sudo apt install ros-<ros2-distro>-slam-toolbox
```

我们将使用包的内置启动文件启动slam_toolbox的async_slam_toolbox_node。打开一个新的终端，然后执行以下行:

```
ros2 launch slam_toolbox online_async_launch.py
```

slam_toolbox 现在应该发布/map topic并提供 map => odom 转换。

我们可以在RViz中验证/map主题正在被发布。在RViz窗口中，单击左下角的添加按钮，然后转到By topic选项卡，然后选择/ Map主题下的Map。你应该能够接受到信息并且正确的可视化，如下图所示。

![../../_images/map.png](https://navigation.ros.org/_images/map.png)

我们还可以通过在一个新的终端中执行以下行来检查转换是否正确:

```
ros2 run tf2_tools view_frames.py
```

上面的命令将创建一个frames.pdf文件，显示当前的转换树。你的转换树应该如下所示:

![../../_images/view_frames.png](https://navigation.ros.org/_images/view_frames.png)