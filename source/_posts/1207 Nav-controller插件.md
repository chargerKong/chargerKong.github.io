---
title: 写一个Controller插件-navigation2
date: 2021-12-07 18:38:29
tags: navigation2
---

在本教程中，我们将会使用 `PurePursuitController::configure`, `PurePursuitController::setPlan` 和 `PurePursuitController::computeVelocityCommands`.

# 创建一个新插件

## configure

在控制器中，configure() 函数必须要完成初始化的操作

```c++
void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
      0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist",
    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
      0.1));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
}
```

参数的名字规则为 `<mapped_name_of_plugin>.<name_of_parameter>` ，如果插件的名字是 `FollowPath` ，要获取参数 `desired_linear_vel `，那么参数名字就为`FollowPath.desired_linear_vel`。



## setPlan

我们会接收到一直更新的全局路径，发布此路径，并且保存到global_plan_中

```c++
void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
}
```

## computeVelocityCommands()

此函数可以计算速度指令，可以根据现有的速度和pose计算指令。在Pure Pursuit中，它会尽可能的去跟随全局路径来发布速度指令，此算法的速度是一个常量，它会基于这个全局路径来计算一个角度。最后返回一个cmd_vel

```c++
geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // Find the first pose which is at a distance greater than the specified lookahed distance
  auto goal_pose = std::find_if(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return hypot(
        global_plan_pose.pose.position.x,
        global_plan_pose.pose.position.y) >= lookahead_dist_;
    })->pose;

  double linear_vel, angular_vel;

  // If the goal pose is in front of the robot then compute the velocity using the pure pursuit algorithm
  // else rotate with the max angular velocity until the goal pose is in front of the robot
  if (goal_pose.position.x > 0) {

    auto curvature = 2.0 * goal_pose.position.y /
      (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
    linear_vel = desired_linear_vel_;
    angular_vel = desired_linear_vel_ * curvature;
  } else {
    linear_vel = 0.0;
    angular_vel = max_angular_vel_;
  }

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = max(
    -1.0 * abs(max_angular_vel_), min(
      angular_vel, abs(
        max_angular_vel_)));

  return cmd_vel;
}
```

其他的方法也是需要重载的，留空即可

# 导出插件

我们已经完成了插件的制作，现在需要将其导出，这样controller Server才可以找到它。在ROS2中，导入和导出一个插件是通过 `pluginlib`来完成的

1. 代码的最后我们需要添加两行

```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
```

Pluginlib会提供一个宏 `PLUGINLIB_EXPORT_CLASS` ，他可以完成导出的所有工作

这两行也可以写在做在文件的最前面，最好写在最后面

2. 接下来需要添加一下该描述该插件的xml文件，在根目录，创建pure_pursuit_controller_plugin.xml

```xml
<library path="nav2_pure_pursuit_controller">
  <class type="nav2_pure_pursuit_controller::PurePursuitController" base_class_type="nav2_core::Controller">
    <description>
      This is pure pursuit controller
    </description>
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
pluginlib_export_plugin_description_file(nav2_core pure_pursuit_controller_plugin.xml)
```

4. 插件的描述应该被添加到 `package.xml`中

```xml-dtd
<export>
  <build_type>ament_cmake</build_type>
  <nav2_core plugin="${prefix}/pure_pursuit_controller_plugin.xml" />
</export>
```

5. 编译，生成插件



## 步骤三：使用插件

我们需要去修改Nav2的配置文件，修改原来的controller插件参数

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_pure_pursuit_controller::PurePursuitController"
      debug_trajectory_details: True
      desired_linear_vel: 0.2
      lookahead_dist: 0.4
      max_angular_vel: 1.0
      transform_tolerance: 1.0
```

在上面的配置文件中，我们为id为FollowPath的插件 指定`nav2_pure_pursuit_controller::PurePursuitController`插件，为了使用插件特定的参数我们可以使用 `<plugin_id>.<plugin_specific_parameter>`.

## 步骤四：跑起来！

```
$ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml
```

