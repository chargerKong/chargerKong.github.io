---
title: ros2 topic
date: 2021-03-26 16:24:35
tags: ros2
---

## 错误记录

1. rosdep init 错误

打开网页，自己建一个

1. rosdep update错误

修改了hosts, 翻墙

underlay：core ROS 2 workspace
overlays：Subsequent local workspaces

多个工作空间可以使安装多个ROS2成为可能

## 环境配置

每一次打开一个shell都需要运行一下

```
. ~/ros2_dashing/install/setup.bash
```

把这句话加入到`./bashrc`

添加 colcon_cd 到shell的启动bash里面

```
sudo pip3 install -U colcon-common-extensions
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_dashing/install" >> ~/.bashrc
```

检查环境变量是否有问题

```
printenv | grep ROS
```

他应该返回

```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=dashing
```

在使用ROS 2开发环境之前，需要对其进行正确的配置。这可以通过两种方式完成：在打开的每个新Shell中source一下安装文件，或者将source命令添加到启动脚本中

# 介绍turtlesim和rqt

**目标**：安装并使用turtlesim软件包和rqt工具，为即将到来的教程做准备。

### 1. 安装turtlesim

```
sudo apt update
sudo apt install ros-dashing-turtlesim
```

检查是否安装成功

```
ros2 pkg executables turtlesim
```

应该返回

```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

### 2. 启动turtlesim

```
ros2 run turtlesim turtlesim_node
```

会随机出现一直小乌龟，并且再命令行窗口返回他的坐标和旋转角度

### 3.使用turtlesim

打开一个新窗口

```
ros2 run turtlesim turtle_teleop_key
```

现在就可以再新的窗口下，通过按键上下左右来控制小乌龟

可以通过

```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

来查看`node`,`topic`,`service`,`action`东西。

您将在接下来的教程中了解更多关于这些概念的信息。由于本教程的目标只是对turtlesim进行总体概述，所以我们将使用rqt(用于ROS 2的图形用户界面)来更深入地了解服务。

## 安装rqt

打开一个新终端以安装rqt及其插件：

```
sudo apt install ros-dashing-rqt*
```

打开新的终端，run rqt

```
rqt
```

### 1. 使用rqt

点击**Plugins** > **Services** > **Service Caller** ，需要等几分钟的时间， 选择`/spawn`

#### 1.1 使用rqt服务

对name中的expression起一个名字，比如turtle2。点击call, 就会再turtlesim上看见一个新的乌龟。

#### 1.2 使用set_pen服务

选择turtle1/set_pen, 里面的表达式的内容表示着小乌龟跑过路径的设置。设置完点击call。您可能已经注意到，无法移动turtle2。您可以通过将turtle1的`cmd_vel` remap到turtle2上来完成此操作

## remapping

打开一个新的终端

```
ros2 run turtlesim turtle_teleop_key turtle1/cmd_vel:=turtle2/cmd_vel
```

现在可以再新的终端下控制第二个小乌龟了。并且再原来控制第一个小乌龟的终端下也可以控制第一个小乌龟

# 理解 ROS2 结点

ROS2 图是由ROS 2元素同时处理数据的网络

ROS2 结点：ROS中的每个节点都应该负责一个单独的模块，每个节点都可以通过主题，服务，操作或参数向其他节点发送和接收数据

![../_images/Nodes-TopicandService.gif](https://docs.ros.org/en/dashing/_images/Nodes-TopicandService.gif)

完整的机器人系统由许多协同工作的节点组成。在ROS 2中，单个可执行文件（C ++程序，Python程序等）可以包含一个或多个节点。

## 1. ros2 run

ros2 run 启动从一个package中启动一个可执行文件

```
ros2 run <package_name> <executable_name>
```

例子

```
ros2 run turtlesim turtlesim_node
```

这里的package名字为turtlesim, 可执行文件名字为 turtlesim_node

但是，我们仍然不知道节点名称。您可以使用ros2节点列表找到节点名称

## 2.ros2 node list

ros2节点列表将显示所有正在运行的节点的名称。当您要与节点进行交互时，或者当系统运行着许多节点并需要对其进行跟踪时，此功能特别有用。

当turtlesim仍在另一个终端中运行时，打开一个新终端，然后输入以下命令：

```
ros2 node list
```

返回

```
/turtlesim
```

现在再打开一个窗口

```
ros2 run turtlesim turtle_teleop_key
```

还是那个package， 但是可执行文件现在是turtle_teleop_key

再一次查看

```
ros2 node list
```

返回的东西多了一个teleop_turtle

```
/teleop_turtle
/turtlesim
```

### 2.1 remapping

重新映射允许您将默认节点属性(如节点名称、主题名称、服务名称等)重新分配给自定义值。在上一个教程中，您使用了对turtle_teleop_key的重新映射来更改要控制的默认乌龟。

现在，让我们重新分配我们的/ turtlesim节点的名称。在新的终端中，运行以下命令：

```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

由于您再次调用ros2在turtlesim上运行，因此将打开另一个turtlesim窗口。但是如果您现在再一次运行`ros2 node list`，您将看到三个节点名称

```
/turtlesim
/teleop_turtle
/my_turtle
```

## 3.ros2 node info

现在您知道节点的名称，您可以通过以下方式访问有关它们的更多信息：

```
ros2 node info <node_name>
```

节点是ros 2的基本元素，在机器人系统中具有单一、模块化的功能。**节点可以是一个受控制的小乌龟，也可以是控制小乌龟的teleop_turtle**

在本教程中，您通过 运行可执行文件turtlesim_node和turtle_teleop_key 从turtlesim包创建了节点。

您学习了如何使用`ros2 node list`来发现还活跃的节点名称和`ros2 node info <info name>`来查看单个节点的信息。这些工具对于理解一个复杂的、真实世界的机器人系统中的数据流至关重要。

# 理解ros2 topics

目标：使用rqt_graph和命令行工具来理解ROS 2主题。

ROS 2将复杂的系统分解为许多模块化节点。topics是ROS图的重要元素，它充当节点交换消息的总线。

![../../_images/Topic-MultiplePublisherandMultipleSubscriber.gif](https://docs.ros.org/en/dashing/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

一个节点可以发布多个topics，同时订阅多个topics。

## 1. setup

```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

俩默认节点：/turtlesim /teleop_turtle

## 2.rqt_graph

在整个教程中，我们将使用rqt_graph可视化不断变化的节点和主题以及它们之间的连接。

```
rqt_graph
```

也可以通过键入 `rqt`里面的**Plugins** > **Introspection** > **Nodes Graph**来打开**rqt_graph**

![../../_images/rqt_graph.png](https://docs.ros.org/en/dashing/_images/rqt_graph.png)

圈圈是节点，该图描述了/ turtlesim节点和/ teleop_turtle节点如何通过主题相互通信

`/ teleop_turtle`节点正在将数据（您输入的用于移动乌龟的击键）发布到`/ turtle1 /cmd_vel`主题, `/ turtlesim`节点已订阅该主题以接收数据。

rqt_graph是一个图形自省工具。现在，我们将研究一些用于内省主题的命令行工具

## 3. ros2 topic list

`ros2 topic list -t` 将返回主题列表，且主题类型附加在每个主题之后，

```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

主题有名称和类型。这些属性，特别是类型，是节点在转移主题时如何知道它们谈论的是相同的信息。

可以把上面的hide键取消掉，来看所有的主题之间的关系

## 4.ros2 topic echo

要查看有关某个主题的发布数据，请使用：

```
ros2 topic echo <topic_name>
```

由于我们知道`/ teleop_turtle`通过`/ turtle1 / cmd_vel`主题将数据发布到`/ turtlesim`，因此让我们使用echo对该主题进行查看

```
ros2 topic echo /turtle1/cmd_vel
```

起初不会有任何的数据，直到我们开始发布数据（按下键盘操作小乌龟）

返回

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

现在去看一下rqt_graph

![../../_images/debug.png](https://docs.ros.org/en/dashing/_images/debug.png)

`/_ros2cli_26646`是我们刚才运行的`echo`所创建的节点(编号会改变)，**现在您可以看到发布者正在通过cmd_vel主题发布数据，并且有两个订阅者被订阅。**

## 5. ros2 topic info

主题不必只是点对点交流；它可以是一对多，多对一或多对多。可以通过`ros2 topic info`来查看

```
ros2 topic info /turtle1/cmd_vel
Topic: /turtle1/cmd_vel
Publisher count: 1
Subscriber count: 2
```

## 6. ros2 interface（msg） show

节点使用消息通过主题发送数据。发布者和订阅者必须发送和接收相同类型的消息才能进行通信

在运行`ros2 topic list -t`之后，我们在前面看到的主题类型让我们知道每个主题可以发送什么类型的消息。回想一下cmd_vel主题的类型：

```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

这意味着在软件包geometry_msgs中有一个msg，他的名字是Twist

现在我们可以运行`ros2 msg show <type>`来了解它的详细信息

```
ros2 msg show geometry_msgs/msg/Twist
```

返回

```
#This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
```

`/turtlesim`节点订阅了这个主题，这意味着`/ turtlesim`节点正在等待一条消息，该消息包含两个线性和角度向量，每个向量包含三个元素。

## 7.ros2 topic pub

有了消息的数据结构后，您可以使用以下命令直接从命令行将数据发布到主题上：

```
ros2 topic pub <topic_name> <msg_type> '<args>'
```

`'<args>'`参数是您将传递给主题的实际数据，结构可以通过第六节的方式查看

请务必注意，此参数需要以YAML语法输入。输入完整的命令，如下所示：

```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

–once是一个可选参数，意思是“发布一条消息然后退出”。

修改一下参数

```
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

告诉ros2 topic pub以1 Hz的稳定流发布命令。

![../../_images/rqt_graph2.png](https://docs.ros.org/en/foxy/_images/rqt_graph2.png)

再rqt_graph里面会多一个节点`ros 2 topic pub ...` 向 `/turtle1/cmd_vel`主题发布数据，该主题会被两个节点订阅（`\turtlesim`和`/_ros2cli_26646`）

## 8.ros2 topic hz

您可以使用以下方法查看数据发布的速率

```
ros2 topic hz /turtle1/pose
```

它将返回 `/ turtlesim`节点向`pose`主题发布数据的速率

# Summary

节点按主题发布信息，这使任意数量的其他节点可以订阅和访问该信息。在本教程中，您使用rqt_graph和命令行工具检查了主题上多个节点之间的连接。现在，您应该对数据如何在ROS 2系统中移动有了一个了解。