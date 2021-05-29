---
title: ros学习三
date: 2021-03-28 09:59:22
tags:
---

# Using rqt_console

了解rqt_console，这是一个用于审查日志消息的工具。

## 1.启动

```
ros2 run rqt_console rqt_console 
```

![../../_images/console.png](https://docs.ros.org/en/dashing/_images/console.png)

控制台的第一部分将显示系统日志消息。

在中间，您可以选择通过排除严重性级别来过滤消息。您也可以使用右侧的加号按钮添加更多排除过滤器。

底部用于突出显示包含您输入的字符串的消息。您也可以在此部分添加更多过滤器。

现在用以下命令在新的终端中启动turtlesim:

```
ros2 run turtlesim turtlesim_node
```

## 2 Messages on rqt_console

为了在rqt_console中输出一些log的信息，我们让小乌龟撞墙。打开一个新的终端，输入以下信息

```
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: { 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
```

。。但是我的rqt_console这里不会出现错误， 可能是连接失败了

## 3. logger 等级

ROS 2的logger级别按严重性排序

```
Fatal
Error
Warn
Info
Debug
```

对于每个级别所代表的内容并没有确切的标准，但可以放心地假设:

- fatal 指示系统将终止，用于保护自身免受损害。
- error 表示重大问题，不一定会损坏系统，但会阻止系统正常运行。
- message 表明意外的活动或非理想的结果，这些结果可能代表更深层次的问题，但不会完全损害功能。
- info 指示事件和状态更新，可作为视觉验证系统是否按预期运行。
- debug 详细介绍系统执行的整个分步过程。

默认级别为“info”。您将仅看到默认严重级别和更高级别的消息。

通常，只有调试消息是隐藏的，因为它们是唯一比Info级别轻的级别。例如，如果将默认级别设置为Warn，则只会看到严重级别为Warn、Error和Fatal的消息

### 3.1Set the default logger level

```
ros2 run turtlesim turtlesim_node __log_level:=warn
```

这样就看不见info的消息了。

## Summary

`rqt_console`对于检查log message来说十分有用

# 创建一个启动文件

**Goal:** Create a launch file to run a complex ROS 2 system.

之前的学习需要不停的打开终端还要初始化，很无聊。

启动文件使您可以同时启动和配置许多包含ROS 2节点的可执行文件。

使用ros2 launch命令运行单个启动文件将立即启动整个系统——所有节点及其配置。

## 1. 启动

```
mkdir launch
vi launch/turtlesim_mimic_launch.py
```

## 2. 创立启动文件

复制内容到文件

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            node_namespace='turtlesim1',
            node_executable='turtlesim_node',
            node_name='sim'
        ),
        Node(
            package='turtlesim',
            node_namespace='turtlesim2',
            node_executable='turtlesim_node',
            node_name='sim'
        ),
        Node(
            package='turtlesim',
            node_executable='mimic',
            node_name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

### 2.1检查启动文件

```
def generate_launch_description():
   return LaunchDescription([

   ])
```

在LaunchDescription中有一个包含三个节点的系统，所有节点都来自turtlesim包。该系统的目标是启动两个海龟窗口，并让一只海龟模仿另一只海龟的移动

启动两个turtlesim窗口:

```
Node(
    package='turtlesim',
    node_namespace='turtlesim1',
    node_executable='turtlesim_node',
    node_name='sim'
),
Node(
    package='turtlesim',
    node_namespace='turtlesim2',
    node_executable='turtlesim_node',
    node_name='sim'
),
```

唯一不同的是他们的`node_namespace`

在这个系统中，这两只乌龟都接收同一个主题的命令，并在同一个主题上发布它们的姿势

如果没有不同的名称空间，就没有办法区分不同海龟的消息。

最后一个节点也来自turtlesim包，但是有一个不同的可执行文件：mimic

```
Node(
    package='turtlesim',
    node_executable='mimic',
    node_name='mimic',
    remappings=[
      ('/input/pose', '/turtlesim1/turtle1/pose'),
      ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    ]
)
```

该节点以重新映射的形式添加了配置详细信息。

`mimic`的`/input/pose`topic重新映射到`/turtlesim1/turtle1/pose`，`/output/cmd_vel`topic重新映射到`/turtlesim2/turtle1/cmd_vel`。这意味着`mimic`将订阅`/turtlesim1/sim`的pose主题，并将其重新发布给订阅的`/turtlesim2/sim`的velocity命令主题。换句话说，turtlesim2会模仿turtlesim1的动作。

## 3.ros2 launch

```
cd launch
ros2 launch turtlesim_mimic_launch.py
```

**注意：**启动方式有两种，一种如上。第二种通过package的方式启动

两个turtlesim窗口将打开，您将看到以下[INFO]消息，告诉您的启动文件已启动的节点:

打开一个新的终端，让第一个小海龟进行移动

```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

## 4.利用rqt_graph查看系统状态

![img](https://chargerkong.github.io/2021/03/16/ros2%E5%AD%A6%E4%B9%A0%E4%B8%89/fig1.jpg)

每一个小乌龟都是订阅自己相应的cmd_vel topic

我们从命令行发布topic给`turtle1/com_vel` topic，小乌龟1号移动然后发布自己的pose.

通过mimic订阅了这个pose的topic然后发布给`turtle2/com_vel`，这样就可以一起动了

## Summary

启动文件简化了具有许多节点和特定配置详细信息的复杂系统的运行。您可以使用Python创建启动文件，然后使用ros2 launch命令运行它们。

# 记录和回放数据

目标：记录有关某个主题的发布数据，以便您可以随时重播和检查它。

`ros2 bag`可以记录发布再某一个topic上的数据，可以再回放一遍

## 1.启动

打开俩终端，一个是

```
ros2 run turtlesim turtlesim
ros2 run turtlesim turtle_teleop_key
```

让我们新建一个文件夹来存放我们的ros_bag文件

## 2.选择一个topic

ros2 bag只能记录发布主题的数据。

查看一下当前有哪些topic

```
ros2 topic list 
```

返回

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

我们知道，节点`/turtle_teleop`发布数据到`/turtle1/cmd_vel`的topic上，然后turtlesim接受此topic上的数据，进行移动。

查看此主题上的信息

```
ros2 topic echo /turtle1/cmd_vel
```

当我们按下键盘

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

## 3.ros2 bag record

首先进入rosbag,要存储数据的文件夹，输入以下语法

```
ros2 bag record <topic_name>
```

再这里我们选择的是`/turtle1/cmd_vel`

```
ros2 bag record /turtle1/cmd_vel
```

在终端可以看见

```
[INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

现在ros2 bag正在记录发布在/ turtle1 / cmd_vel主题上的数据

按下Ctrl+C

数据将以rosbag2_year_month_day-hour_minute_second的格式存储在bag文件中

## 4. 记录多个topic

```
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

`-o`代表要存储文件的名字，为subset。

这里记录了两个topic

## 5. ros2 bag info

```
ros2 bag info <file_name>
```

例如

```
ros2 bag info subset
```

返回

```
Files:             subset.db3
Bag size:          84.5 KiB
Storage id:        sqlite3
Duration:          17.183s
Start:             Mar 17 2021 11:25:52.449 (1615951552.449)
End                Mar 17 2021 11:26:09.632 (1615951569.632)
Messages:          1085
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 10 | Serialization Format: cdr
                   Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 1075 | Serialization Format: cdr
```

如果要去查看单个的消息，则需要打开数据库来查看，这里是sqlite3

## 6. ros2 bag play

```
ros2 bag play subset
```

小乌龟会按照之前记录好的进行运动。

# Summary

可以使用ros2 bag命令记录在ROS 2系统中主题上传递的数据。无论你是与他人分享你的工作还是反省你自己的实验，这都是一个很好的工具

# 建立一个工作区

目标：创建工作区并学习如何为开发和测试设置覆盖。

工作空间是包含ROS 2软件包的目录。

还可以选择性的source一个overlay。—— 一个次要的工作空间，在这里你可以添加新的包，而不会干扰你正在扩展的现有ross2工作空间，或者“underlay”。您的underlay必须包含覆盖层中所有包的依赖项。

## 1. source ros2 环境

## 2.创建一个新的目录

建立一个新文件夹`dev_ws/src`, 就是development workspace。

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

## 3.clone 一个样例库

接下来，你会创建自己的包。现在还是需要组合现有的包来进行练习

You can see the repo [on GitHub](https://github.com/ros/ros_tutorials/). 如何使用的这个GitHub？

```
git clone https://github.com/ros/ros_tutorials.git -b dashing-devel`
```

`-b`表示选择一个分支。进入repo

前三个软件包可以忽略； 只有turtlesim是ROS2的包。现在您已经使用示例包填充了工作区，但它还不是一个功能完整的工作区。您需要首先解决依赖关系并构建工作区。

## 4.解决依赖关系

构建之前需要解决依赖的问题

```
rosdep install -i --from-path src --rosdistro dashing -y
```

rosdep 之后会说

## 5. 用colcon进行build workspace

再src 目录同级目录下，进行

```
colcon build
```

返回

```
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]

Summary: 1 package finished [5.58s]
```

现在文件夹里有`build`,`install`,`log`,`src`四个文件夹

install文件夹是你工作区启动文件所在的地方，可以source你的overlay

## 6.source the overlay

记住，source之前必须要重新开一个终端，不能和build是一个终端，否则会出现一定的问题

打开一个新终端后，先source一下主ros2环境

```
source ~/ros2_dashing/install/setup.bash
```

然后去工作区的根目录

```
cd ~/dev_ws
```

source 一下自己的overlaysetup.cfg

```
source install/local_setup.bash
```

注意，这两部操作和直接进行

```
source install/setup.bash
```

是一样的.

接下来就可以打开小乌龟了

```
ros2 run turtlesim turtlesim_node
```

但是怎么知道这个是主ros2的环境启动的还是自己的overlay启动的呢

## 7. 修改overlay

我们可以修改小乌龟窗口的名字。打开`~/dev_ws/src/ros_tutorials/turtlesim/src/turtle_frame.cpp`。修改`setWindowTitle("TurtleSim");`里面的这个名字。

然后重新build，打开一个新终端运行这个

```
ros2 run turtlesim turtlesim_node
```

## Summary

在本教程中，您将主要的ROS 2发行版作为underlay(支撑物)，并通过在新工作区中克隆和构建软件包来创建了一个overlay（覆盖物）。overlay的优先级会大于underlay。

建议使用overlay来处理少量的包，因此您不必将所有内容都放在同一工作区中，而在每次迭代时都可以重建一个巨大的工作区。

# 创建一个属于自己的package

## 1. 什么是ROS2 package

一个包可以看做是ROS2代码的容器，当我们需要分享自己的代码给别人或者安装自己的代码时，就需要把代码整理成一个包。使用软件包，您可以发布ROS 2的工作，并允许其他人轻松构建和使用它。

ROS 2中的软件包创建使用ament作为其构建系统，并使用colcon作为其构建工具。您可以使用官方支持的CMake或Python创建软件包，尽管确实存在其他构建类型。

## 2 ROS2封装是由什么组成的

- `package.xml`文件，其中包含有关程序包的元信息
- `setup.py`包含有关如何安装软件包的说明
- `setup.cfg`:软件包包含可执行文件时，需要setup.cfg，因此`ros2 run`可以找到它们
- `/ <package_name>`-与您的软件包同名的目录，由ROS 2工具用来查找您的软件包，包含`__init__.py`

最简单的程序包可能具有如下所示的文件结构：

```
my_package/
      setup.py
      package.xml
      resource/my_package
```

## 3. 工作区中的软件包

一个工作区可以有多个软件包，即可以有c++的也可以有python的，但是不可以有嵌套的软件包

最好是把所有的软件包放在`src`下面，

```
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/package_2
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

## 4.create一个package

```
source ~/ros2_dashing/install/setup.bashsource
cd ~/dev_ws/src
```

下面是建立一个新的package的语法

```
ros2 pkg create --build-type ament_python <package_name>
```

我们将使用一个可选参数`--node-name`，可以建立一个helloWord

```
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

这样再src文件夹里就有会有一个`my_package`的包。这里my_node就是可执行文件

## 5. build 一个 package

把所有的package放在一个工作区中，这样`colcon build`就可以一次性进行build，就不需要为每一个单独的包进行build

```
cd ~/dev_ws
colcon build
```

也可以只选择一个build一个package

```
colcon build --packages-select my_package
```

## 6.source

```
.install/setup.bash
```

## 7. use the package

```
ros2 run my_package my_node
```

## 8.查看包文件

打开`~/dev_ws/src/my_package`

```
my_package  package.xml  resource  setup.cfg  setup.py  test
```

`my_node.py`位于my_package目录中。这是未来所有定制Python节点的位置。

## 9.自定义package.xml

再建立一个包的时候其实会有俩TODO，一个是`licence`，另一个是`description`。 这俩东西再发布一个包的时候是必须的。`maintainer`字段也可能需要填写。

打开`dev_ws/src/my_package/package.xml`。添加修改

```
 1 <?xml version="1.0"?>
 2 <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schema
 3 <package format="3">
 4   <name>mypackage</name>
 5   <version>0.0.0</version>
 6   <description>klqdes</description>
 7   <maintainer email="chargerKong@126.com">kong</maintainer>
 8   <license>Apache2.0</license>                                              
 9 
10   <test_depend>ament_copyright</test_depend>
11   <test_depend>ament_flake8</test_depend>
12   <test_depend>ament_pep257</test_depend>
13   <test_depend>python3-pytest</test_depend>
14 
15   <export>
16     <build_type>ament_python</build_type>
17   </export>
18 </package>
```

setup.py文件里面也有description，licence，和maintainer的信息，因此也要同时修改

```
 1 from setuptools import setup
 2 
 3 package_name = 'mypackage'
 4 
 5 setup(
 6     name=package_name,
 7     version='0.0.0',
 8     packages=[package_name],
 9     data_files=[
10         ('share/ament_index/resource_index/packages',
11             ['resource/' + package_name]),
12         ('share/' + package_name, ['package.xml']),
13     ],
14     install_requires=['setuptools'],
15     zip_safe=True,
16     maintainer='kong',
17     maintainer_email='chargerKong@126.com',
18     description='klqdes',
19     license='Apache2.0',                                                                                                                                   
20     tests_require=['pytest'],
21     entry_points={
22         'console_scripts': [
23             'my_node = mypackage.my_node:main'
24         ],
25     },
26 )
```

## Summary

您已经创建了一个用于组织代码并易于他人使用的程序包。

您的软件包将自动填充必要的文件，然后使用colcon进行构建，以便可以在本地环境中使用其可执行文件。