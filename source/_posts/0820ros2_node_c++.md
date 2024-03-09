---
title: ros2建立c++包
date: 2021-08-20 18:38:29
tags: ros2
---

# 建立一个ROS2的包

在ROS2中，我们使用amend来作为构建包的工具，并且使用colcon来作为编译包的工具。在创建包的时候我们可以使用CMake或者Python，本文主要

# 由什么组成一个包

- package.xml 主要描述着一个包的元数据
- CMakeLists.txt 主要描述如何去构建这个包

最简单的目录结构如下所示

```
my_package/
     CMakeLists.txt
     package.xml
```

# Workspace

一个工作空间可以包含多个包，并且包可以人c++或者Python都可以，最好全部放在src文件夹下。让主目录干净一点

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

# 建立一个包

```
ros2 pkg create --build-type ament_cmake <package_name>
```

当然也可以加入一个节点的名字, 进入src文件夹，输入

```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

# 编译一个包

进入ws，即和src同一个目录下进行编译

```
colcon build --packages-select my_package
```

# source

为了让新的包和节点可以被使用，需要激活一下Node

```
source ~/dev_ws/install/setup.bash
```

# 运行一个节点

```
ros2 run my_package my_node 
```

返回hello world my_package package

# 查看包的内容

在`dev_ws/src/my_package`里面，可以看见有自动生成的文件

```
CMakeLists.txt  include  package.xml  src
```

my_node.cpp在src文件夹里面



# 自定义package.xml

在package.xml有很多的todo需要写，如果你需要发布自己的包

```
 1<?xml version="1.0"?>
 2<?xml-model
 3   href="http://download.ros.org/schema/package_format3.xsd"
 4   schematypens="http://www.w3.org/2001/XMLSchema"?>
 5<package format="3">
 6 <name>my_package</name>
 7 <version>0.0.0</version>
 8 <description>TODO: Package description</description>
 9 <maintainer email="user@todo.todo">user</maintainer>
10 <license>TODO: License declaration</license>
11
12 <buildtool_depend>ament_cmake</buildtool_depend>
13
14 <test_depend>ament_lint_auto</test_depend>
15 <test_depend>ament_lint_common</test_depend>
16
17 <export>
18   <build_type>ament_cmake</build_type>
19 </export>
20</package>
```

需要在这里完善description和licence.