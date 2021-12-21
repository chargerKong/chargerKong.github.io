---
title: 写一个planner插件-navigation2
date: 2021-12-06 18:38:29
tags: navigation2
---

源https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html

![Animated gif with gradient demo](https://navigation.ros.org/_images/nav2_straightline_gif.gif)

# 创建一个新的Planner插件

该教程会指导我们如何制作一个走直线的planner

## 前置要求

- ROS2
- Nav2
- Gazebo
- turtlebot3

## 步骤一：创建一个新的Planner插件

我们会创建一个简单的走直线的Planner插件，代码在[navigation_tutorials](https://github.com/ros-planning/navigation2_tutorials) 仓库中，名字为nav2_straightline_planner。我们的插件继承于`nav2_core::GlobalPlanner`. 基类提供了五个纯虚函数，这个插件会被Planner Server调用来计算轨迹

| **Virtual method** | **Method description**                                       | **Requires override?** |
| ------------------ | ------------------------------------------------------------ | ---------------------- |
| configure()        | Method is called at when planner server enters on_configure state. Ideally this methods should perform declarations of ROS parameters and initialization of planner’s member variables. This method takes 4 input params: shared pointer to parent node, planner name, tf buffer pointer and shared pointer to costmap. | Yes                    |
| activate()         | Method is called when planner server enters on_activate state. Ideally this method should implement operations which are neccessary before planner goes to an active state. | Yes                    |
| deactivate()       | Method is called when planner server enters on_deactivate state. Ideally this method should implement operations which are neccessary before planner goes to an inactive state. | Yes                    |
| cleanup()          | Method is called when planner server goes to on_cleanup state. Ideally this method should clean up resoures which are created for the planner. | Yes                    |
| createPlan()       | Method is called when planner server demands a global plan for specified start and goal pose. This method returns nav_msgs::msg::Path carrying global plan. This method takes 2 input parmas: start pose and goal pose. | Yes                    |

在本教程中，我们会利用StraightLine::configure() 和 StraightLine::createPlan()来创建直线planner.

在Planner中，configure() 方法必须要设置好成员变量

```c++
node_ = parent;
tf_ = tf;
name_ = name;
costmap_ = costmap_ros->getCostmap();
global_frame_ = costmap_ros->getGlobalFrameID();

// Parameter initialization
nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
```

name_ + ".interpolation_resolution" 的值可以从ROS 参数系统中获得，并把他赋值给我们定义的成员变量，interpolation_resolution。Nav2可以同时加载多个插件，通过 `<mapped_name_of_plugin>.<name_of_parameter>` 方式可以检索出特定插件的参数。举个例子，如果这个插件名字为`GridBased`，我们想要指定interpolation_resolution 参数，那么`Gridbased.interpolation_resolution` 即可，

在 `createPlan()`方法中，我们需要指定一条从start到end的路径。该方法会创建一个 `nav_msgs::msg::Path` 并将他返回给planner server。

```c++
nav_msgs::msg::Path global_path;

// Checking if the goal and start state is in the global frame
if (start.header.frame_id != global_frame_) {
  RCLCPP_ERROR(
    node_->get_logger(), "Planner will only except start position from %s frame",
    global_frame_.c_str());
  return global_path;
}

if (goal.header.frame_id != global_frame_) {
  RCLCPP_INFO(
    node_->get_logger(), "Planner will only except goal position from %s frame",
    global_frame_.c_str());
  return global_path;
}

global_path.poses.clear();
global_path.header.stamp = node_->now();
global_path.header.frame_id = global_frame_;
// calculating the number of loops for current value of interpolation_resolution_
int total_number_of_loop = std::hypot(
  goal.pose.position.x - start.pose.position.x,
  goal.pose.position.y - start.pose.position.y) /
  interpolation_resolution_;
double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

for (int i = 0; i < total_number_of_loop; ++i) {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = start.pose.position.x + x_increment * i;
  pose.pose.position.y = start.pose.position.y + y_increment * i;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose.header.stamp = node_->now();
  pose.header.frame_id = global_frame_;
  global_path.poses.push_back(pose);
}

global_path.poses.push_back(goal);

return global_path;
```

## 步骤二：导出插件

我们已经完成了插件的制作，现在需要将其导出，这样Planner Server才可以找到它。在ROS2中，导入和导出一个插件是通过 `pluginlib`来完成的

1. 代码的最后我们需要添加两行

```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
```

Pluginlib会提供一个宏 `PLUGINLIB_EXPORT_CLASS` ，他可以完成导出的所有工作

这两行也可以写在做在文件的最前面

2. 接下来需要添加一下该描述该插件的xml文件，在根目录，创建global_planner_plugin.xml
```xml
<library path="nav2_straightline_planner_plugin">
  <class name="nav2_straightline_planner/StraightLine" type="nav2_straightline_planner::StraightLine" base_class_type="nav2_core::GlobalPlanner">
    <description>This is an example plugin which produces straight path.</description>
  </class>
</library>
```

属性信息：

- `library path`:  插件的名字
- `class name`: 类名
- `class type`: 类的类型
- `base class`: 基类的类名
- `description`: 插件作用的描述

3.  下一步我们需要通过CMakeLists来导出插件，利用`pluginlib_export_plugin_description_file()`.函数，他可以把插件安装到share目录下

```
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
```

4. 插件的描述应该被添加到 `package.xml`中

```xml-dtd
<export>
  <build_type>ament_cmake</build_type>
  <nav2_core plugin="${prefix}/global_planner_plugin.xml" />
</export>
```

5. 编译，生成插件



## 步骤三：使用插件

我们需要去修改Nav2的配置文件，修改原来的Planner插件

```yaml
planner_server:
ros__parameters:
  planner_plugin_types: ["nav2_straightline_planner/StraightLine"] # For Eloquent and earlier
  planner_plugin_ids: ["GridBased"] # For Eloquent and earlier
  plugins: ["GridBased"] # For Foxy and later
  use_sim_time: True
  GridBased:
    plugin: nav2_straightline_planner/StraightLine # For Foxy and later
    interpolation_resolution: 0.1
```

在上面的配置文件中，我们为id为GridBased的插件 指定`nav2_straightline_planner/StraightLine`插件，为了使用插件特定的参数我们可以使用 `<plugin_id>.<plugin_specific_parameter>`.



## 步骤四：跑起来！

```
$ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml
```

