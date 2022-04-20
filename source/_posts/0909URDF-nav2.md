---
title: 配置URDF-navigation2
date: 2021-09-09 15:59:22
tags: navigation2
---

翻译自https://navigation.ros.org/setup_guides/urdf/setup_urdf.html#urdf-and-the-robot-state-publisher

在此教程中，我们会带着你用URDF手写一个双轮差速机器人，并且配置robot_state_publisher，最后将其显示在rviz里面。为了准备做仿真，我们还会为其添加动力学的一点性质，这些步骤对于表示用于导航的机器人的所有传感器、硬件和机器人转换是必要的。

# URDF and the Robot State Publisher

对于Navigation2来说，一个必不可少的变换是从`base_link`到其他各种frame的变换。这个转换树可以是一个简单的树，只有一个从base_link到laser_link的连接，或者由位于不同位置的多个传感器组成的树，每个传感器都有自己的坐标系。建立多个publisher来处理所有的这些变换是非常麻烦的，所以我们使用一个功能包`Robot State Publisher`来对这些变换进行发布。

Robot State Publisher是一个ROS 2包，它与tf2包交互，发布所有可以直接从机器人的几何和结构推断的必要转换。我们需要为它提供正确的通用机器人描述符文件(Universal Robot Descriptor File, URDF)，它将自动处理转换的发布。这对于复杂的转换非常有用，对于较简单的转换树仍然推荐使用。

通用机器人描述符文件(Universal Robot Descriptor File, URDF)是一个表示机器人模型的XML文件。在本教程中，它将主要用于构建与机器人几何相关的转换树，但它也有其他用途。一个例子是它如何在RVIZ (ROS的3D可视化工具)中可视化你的机器人模型，通过定义可视化组件，如materials 和 meshes。另一个例子是如何使用URDF来定义机器人的物理属性。然后在物理模拟器(如Gazebo)中使用这些属性来模拟机器人在环境中如何交互。

URDF的另一个主要特性是，它还支持Xacro (XML宏)，以帮助您创建更短、可读的XML，以帮助定义复杂的机器人。我们可以使用这些宏来消除URDF中重复XML块的需要。Xacro在定义可以在整个URDF中重用的配置常量方面也很有用。

> If you want to learn more about the URDF and the Robot State Publisher, we encourage you to have a look at the official [URDF Documentation](http://wiki.ros.org/urdf) and [Robot State Publisher Documentation](http://wiki.ros.org/robot_state_publisher)

# Setting Up the Environment

在本指南中，我们假设您已经熟悉ROS 2以及如何设置开发环境，所以我们将轻松地完成本节中的步骤。

我们需要先额外安装一些ROS2的功能包

```
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
```

接下来，我们创建一个自己的ROS包

```
ros2 pkg create --build-type ament_cmake sam_bot_description
```

# Writing the URDF

现在我们已经设置好了项目工作区，让我们直接开始编写URDF。下面是我们将尝试建造的机器人的图像。

![../../_images/base-bot_1.png](https://navigation.ros.org/_images/base-bot_1.png)

首先，创建一个名为sam_bot_description.urdf的文件。Urdf下的src/description，并输入以下内容作为文件的初始内容。

```
<?xml version="1.0"?>
<robot name="sam_bot" xmlns:xacro="http://ros.org/wiki/xacro">



</robot>
```

> 下面的代码片段应该放在标记中。我们建议按照本教程中介绍的顺序添加它们。我们还包含了一些行号，以便您大致了解在哪里输入代码。这可能与您正在编写的实际文件不同，这取决于空白的使用情况。还请注意，行号假定您正在输入本指南中显示的代码。

下面我们利用XAcro来定义一些在URDF中会重复使用的常量
```xml
  <!-- Define robot constants -->
  <xacro:property name="base_width" value="0.31"/>
  <xacro:property name="base_length" value="0.42"/>
  <xacro:property name="base_height" value="0.18"/>

  <xacro:property name="wheel_radius" value="0.10"/>
  <xacro:property name="wheel_width" value="0.04"/>
  <xacro:property name="wheel_ygap" value="0.025"/>
  <xacro:property name="wheel_zoff" value="0.05"/>
  <xacro:property name="wheel_xoff" value="0.12"/>

  <xacro:property name="caster_xoff" value="0.14"/>
```

这里是一个简短的讨论，上面这些属性将用于表示出我们的urdf。`base_*`属性定义了机器人主底盘的大小。wheel_radius和wheel_width定义机器人的两个后轮的形状。wheel_ygap沿着y轴调整车轮和底盘之间的间隙，而wheel_zoff和wheel_xoff沿着z轴和x轴适当地定位后轮。最后，caster_xoff沿着x轴定位前脚轮。

然后让我们定义我们的base_link——这个link将是一个大盒子，并将作为我们的机器人的主要底盘。在URDF中，一个`link`标签描述了机器人的刚性部分或组件。然后，机器人状态发布者利用这些定义来确定每个`link`的坐标框架，并发布它们之间的转换。

我们还将定义一些链接的视觉属性，这些属性可以被Gazebo和Rviz等工具使用，向我们展示机器人的3D模型。在这些属性中，描述了link的形状，描述了link的颜色。

对于下面的代码块，我们使用${property}语法访问之前定义的机器人常量基本属性。另外，我们还将主底盘的材质颜色设置为青色。注意，我们在标签下设置了这些参数，所以它们只会作为视觉参数应用，不会影响任何碰撞或物理属性。

```xml
  <!-- Robot Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1.0 1.0 1.0"/>
      </material>
    </visual>
  </link>
```

接下来，让我们定义一个base_footprint。base_footprint是一个虚拟(非物理)链接，它没有维度或碰撞区域。它的主要目的是使各种功能包确定机器人投影到地面的中心。例如，Navigation2使用这个link来确定其避障算法中使用的圆形足迹的中心。同样，我们设置了这个没有尺寸的link，以及当机器人的中心投影到地平面时的位置。

定义了base_link之后，再添加一个joint，将其连接到base_link。在URDF中，一个`joint`描述了坐标系之间的运动学和动力学特性。对于本例，我们将定义一个带有适当偏移量的固定关节`fixed`，以便根据上面的描述将base_footprint链接放置在适当的位置。记住，当从主底盘的中心投影时，我们希望将base_footprint设置为地平面，因此我们得到了wheel_radius和wheel_zoff的总和，以获得沿着z轴的适当位置。

```xml
  <!-- Robot Footprint -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0.0 0.0 ${-(wheel_radius+wheel_zoff)}" rpy="0 0 0"/>
  </joint>
```

现在，我们要给机器人增加两个大的驱动轮。为了使代码更简洁并避免重复，我们将使用宏来定义一个代码块，该代码块将使用不同的参数重复。我们的宏将有3个参数:prefix，它只是为我们的joint和link名称添加一个前缀，x_reflect和y_reflect允许我们分别在x轴和y轴上翻转轮子的位置。在这个宏中，我们还可以定义单个轮子的视觉属性。最后，我们还将定义一个连续的关节，以允许我们的车轮围绕一个轴自由旋转。这个关节还将轮子连接到适当位置的base_link。

在这个代码块的末尾，我们将使用刚刚通过xacro:wheel标记创建的宏实例化两个轮子。请注意，我们还定义了参数，使机器人后部的两侧都有一个轮子。

```xml
  <!-- Wheels -->
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_link">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
            <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="Gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>
    </link>

    <joint name="${prefix}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_link"/>
      <origin xyz="${x_reflect*wheel_xoff} ${y_reflect*(base_width/2+wheel_ygap)} ${-wheel_zoff}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <xacro:wheel prefix="drivewhl_l" x_reflect="-1" y_reflect="1" />
  <xacro:wheel prefix="drivewhl_r" x_reflect="-1" y_reflect="-1" />
```

接下来，我们将在机器人的前面添加一个脚轮。我们将建模这个轮子作为一个球体。同样，我们定义了车轮的geometry，material和joint，以便在适当的位置将其连接到base_link。

```xml
  <!-- Caster Wheel -->
  <link name="front_caster">
    <visual>
      <geometry>
        <sphere radius="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1.0 1.0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="${caster_xoff} 0.0 ${-(base_height/2)}" rpy="0 0 0"/>
  </joint>
```

我们已经为一个简单的差动驱动机器人建造了一个URDF。在下一节中，我们将重点讨论构建包含URDF的ROS包，启动机器人状态发布者，并在RVIz中可视化机器人。

# build and launch

让我们通过添加一些在构建这个项目之后需要的依赖项来开始本节。打开项目目录的根目录，并将以下行添加到package.xml中(最好在 `<buildtool_depend>` 标记之后)

接下来，让我们创建launch文件。ROS 2使用launch文件为我们的包启动必要的节点。从项目的根目录创建一个名为launch的目录，并在其中创建一个display.launch.py文件。下面的启动文件在ROS 2中启动一个机器人发布者节点，该节点使用我们的URDF为我们的机器人发布转换。此外，启动文件还会自动启动RVIZ，这样我们就可以按照URDF定义的方式可视化我们的机器人。将下面的代码片段复制并粘贴到display.launch.py文件中。

为了简化可视化过程，我们提供了一个RVIz配置文件，该文件将在启动包时加载。这个配置文件用适当的设置初始化RVIz，因此一旦机器人启动，您就可以立即查看它。在项目的根目录下创建一个名为rviz的目录和一个名为urdf_config的文件。。将以下内容作为urdf_config.rviz的内容

最后，让我们修改项目根目录中的CMakeLists.txt文件，以包含我们在安装包过程中创建的文件。将以下代码片段添加到CMakeLists.txt文件中，最好是在if(BUILD_TESTING)之上:

```
install(
  DIRECTORY src launch rviz
  DESTINATION share/${PROJECT_NAME}
)
```

现在我们已经准备好使用colcon来构建我们的项目了。导航到项目根目录并执行以下命令。

```
ros2 launch sam_bot_description display.launch.py
```

ROS 2现在应该启动一个机器人发布者节点，并使用URDF启动RVIZ。在下一节中，我们将使用RVIZ查看我们的机器人。

# Visualization using RVIZ

RVIZ是一个机器人可视化工具，可以让我们使用它的URDF看到机器人的3D模型。在使用前一节中的命令成功启动后，RVIZ现在应该会显示在屏幕上，如图所示。您可能需要四处移动和操作视图，以便更好地查看您的机器人。

![../../_images/base-bot_3.png](https://navigation.ros.org/_images/base-bot_3.png)

如你所见，我们已经成功地创建了一个简单的差动驱动机器人，并在RVIz中将其可视化。没有必要在RVIz中可视化您的机器人，但是为了查看您是否正确定义了URDF，这是一个很好的步骤。这有助于确保机器人状态发布者发布了正确的转换。

你可能已经注意到另一个窗口被启动了——这是一个发布jointz状态的GUI。 joint state publisher是另一个ROS 2包，它为我们的非固定joint发布状态。您可以通过小型GUI操作这个publisher，并且关节的新姿态将反映在RVIz中。滑动两个轮子中的任何一个都可以转动这些关节。您可以在浏览Joint State Publisher GUI中的滑块时通过查看RVIZ来查看这一点。

![../../_images/base-bot_4.png](https://navigation.ros.org/_images/base-bot_4.png)

robot state publisher 现在正在发布通过解析URDF得到的转换。其他包(如Nav2)可以使用这些转换来获取关于机器人形状和结构的信息。然而，为了在模拟中正确使用这个URDF，我们需要物理属性，以便机器人能像真实机器人一样对物理环境做出反应。可视化场只用于可视化，而不是碰撞，所以你的机器人会直接穿过障碍物。在下一节中，我们将介绍如何在URDF中添加这些属性。

# Adding Physical Properties

作为本指南的附加部分，我们将修改我们当前的URDF，以包含机器人的一些运动学属性。这些信息可能被物理模拟器(如Gazebo)用来建模和模拟我们的机器人将如何在虚拟环境中行动。

让我们首先定义一些宏。将下面的代码片段放在URDF中常量部分的后面:

```xml
  <!-- Define intertial property macros  -->
  <xacro:macro name="box_inertia" params="m w h d">
    <inertial>
      <origin xyz="0 0 0" rpy="${pi/2} 0 ${pi/2}"/>
      <mass value="${m}"/>
      <inertia ixx="${(m/12) * (h*h + d*d)}" ixy="0.0" ixz="0.0" iyy="${(m/12) * (w*w + d*d)}" iyz="0.0" izz="${(m/12) * (w*w + h*h)}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertial>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0" />
      <mass value="${m}"/>
      <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy = "0" ixz = "0" iyy="${(m/12) * (3*r*r + h*h)}" iyz = "0" izz="${(m/2) * (r*r)}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="sphere_inertia" params="m r">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${(2/5) * m * (r*r)}" ixy="0.0" ixz="0.0" iyy="${(2/5) * m * (r*r)}" iyz="0.0" izz="${(2/5) * m * (r*r)}"/>
    </inertial>
  </xacro:macro>
```

让我们从使用标签将collision区域添加到base_link开始。我们还将使用之前定义的box_inertia宏给base_link添加一些惯性属性。在URDF的 `<link name="base_link">` 放入以下代码片段。

```xml
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <xacro:box_inertia m="15" w="${base_width}" d="${base_length}" h="${base_height}"/>
```

接下来，让我们对wheel宏执行同样的操作。在URDF中wheel宏的标记中包含以下代码片段。

```xml
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>

      <xacro:cylinder_inertia m="0.5" r="${wheel_radius}" h="${wheel_width}"/>
```

最后，让我们将类似的属性添加到球形脚轮。在URDF中我们的脚轮的标签中包含以下内容。

```xml
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
      </geometry>
    </collision>

    <xacro:sphere_inertia m="0.5" r="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
```

> We did not add any inertial or collision properties to our `base_footprint` link since this is a virtual and non-physical link.

构建项目，然后使用上一节中的相同命令启动RViz。

```
colcon build
. install/setup.bash
ros2 launch sam_bot_description display.launch.py
```

你可以通过在左侧面板的RobotModel下启用collision Enabled来验证你是否正确设置了碰撞区域(如果你也关闭Visual Enabled可能会更容易看到)。在本教程中，我们定义了一个与视觉属性相似的碰撞区域。注意，这可能并不总是这样，因为您可以根据机器人的外观选择更简单的碰撞区域。

![../../_images/base-bot_5.png](https://navigation.ros.org/_images/base-bot_5.png)

现在，我们将不得不在这里停止，因为我们将需要设置更多的组件来实际开始在Gazebo中模拟我们的机器人。我们将在这些设置指南的过程中回到这个项目，我们最终将看到我们的机器人在虚拟环境中移动，一旦我们进入模拟部分。在这项工作中缺少的主要组件是模拟机器人控制器所需的模拟插件。我们将在相应的章节中介绍它们并将它们添加到这个URDF中。

# Conclusion

就是这样。在本教程中，您已经成功地为一个简单的差动驱动机器人创建了一个URDF。您还设置了一个启动机器人发布者节点的ROS 2项目，然后该节点使用URDF发布机器人的转换。我们还使用RViz可视化我们的机器人，以验证我们的URDF是否正确。最后，我们在URDF中添加了一些物理属性，以便为模拟做准备。
