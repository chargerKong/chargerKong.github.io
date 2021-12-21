---
title: 如何写recovery插件-navigation
date: 2021-12-09 18:38:29
tags: navigation
---

# 写一个recovery插件

recovery插件是有recovery server管理的，不像planner和controller，每一个recovery是负责他自己的一个action，planner和controller有相同的API，因为他们完成的是差不多的任务，然而recoveries 可以用来完成各种各样的任务，因此每一个 recoveries 都有一个自己的action message和 server. 这对recoveries 提供了巨大的灵活性

我们的例子继承于 `nav2_recoveries::Recovery` ，这个类又是继承于nav2_core::Recovery的，所以他可以作为一个Recovery插件。

基函数nav2_core::Recovery提供了四个纯虚函数，需要去实现。每一个插件都会提供自己的action 服务的接口。详细情况如下，对于nav2_core::Recovery的纯虚函数

| **Virtual method** | **Method description**                                       | **Requires override?** |
| ------------------ | ------------------------------------------------------------ | ---------------------- |
| configure()        | Method is called at when server enters on_configure state. Ideally this methods should perform declarations of ROS parameters and initialization of recovery’s member variables. This method takes 4 input params: shared pointer to parent node, recovery name, tf buffer pointer and shared pointer to a collision checker | Yes                    |
| activate()         | Method is called when recovery server enters on_activate state. Ideally this method should implement operations which are neccessary before the recovery to an active state. | Yes                    |
| deactivate()       | Method is called when recovery server enters on_deactivate state. Ideally this method should implement operations which are neccessary before recovery goes to an inactive state. | Yes                    |
| cleanup()          | Method is called when recovery server goes to on_cleanup state. Ideally this method should clean up resoures which are created for the recovery. | Yes                    |

对于nav2_recoveries，他提供了ROS2的action接口和样板，有四个纯虚函数需要去实现，

| **Virtual method** | **Method description**                                       | **Requires override?** |
| ------------------ | ------------------------------------------------------------ | ---------------------- |
| onRun()            | Method is called immediately when a new recovery action request is received. Gives the action goal to process and should start recovery initialization / process. | Yes                    |
| onCycleUpdate()    | Method is called at the recovery update rate and should complete any necessary updates. An example for spinning is computing the command velocity for the current cycle, publishing it and checking for completion. | Yes                    |
| onConfigure()      | Method is called when recovery server enters on_configure state. Ideally this method should implement operations which are neccessary before recovery goes to an configured state (get parameters, etc). | No                     |
| onCleanup()        | Method is called when recovery server goes to on_cleanup state. Ideally this method should clean up resoures which are created for the recovery. | No                     |

在本教程中，我们会使用onRun()，onCycleUpdate()，和onConfigure()来创建SMS recovery。为简洁起见，将跳过onConfigure()，但只包含参数的声明。

OnRun方法必须接受一个初始状态，然后开始recovery行为，

```c++
Status SMSRecovery::onRun(const std::shared_ptr<const Action::Goal> command)
{
  std::string response;
  bool message_success = _twilio->send_message(
    _to_number,
    _from_number,
    command->message,
    response,
    "",
    false);

  if (!message_success) {
    RCLCPP_INFO(node_->get_logger(), "SMS send failed.");
    return Status::FAILED;
  }

  RCLCPP_INFO(node_->get_logger(), "SMS sent successfully!");
  return Status::SUCCEEDED;
}
```

我们接收到一个action goal,command。这个command包含一个message，包含着通信的消息。 比如这是“呼救”的信息，我们想通过短信发送给我们在作战中心的兄弟。

我们基于Twilio服务来完成这个task，所以首先要创建一个账户，获取此服务的相关信息，如 `account_sid`, `auth_token`等。我们可以通过配置文件来配置这些参数。

我们使用_twilio对象来发送信息，并把结果打印到screen上

由于我们的短时间恢复行为，onCycleUpdate()非常简单。如果是长时间任务比如导航这样的，那么这个函数就会被用于检查超时或者计算control value。在这里我们简单的return  success。因为我们的任务以及在on_run中完成了

```c++
Status SMSRecovery::onCycleUpdate()
{
  return Status::SUCCEEDED;
}
```

# 导出插件

我们已经完成了插件的制作，现在需要将其导出，这样recovery Server才可以找到它。在ROS2中，导入和导出一个插件是通过 `pluginlib`来完成的

1. 代码的最后我们需要添加两行

```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_sms_recovery::SMSRecovery, nav2_core::Recovery)
```

Pluginlib会提供一个宏 `PLUGINLIB_EXPORT_CLASS` ，他可以完成导出的所有工作

这两行也可以写在做在文件的最前面

2. 接下来需要添加一下该描述该插件的xml文件，在根目录，创建recovery_plugin.xml

```xml
<library path="nav2_sms_recovery_plugin">
  <class name="nav2_sms_recovery/SMSRecovery" type="nav2_sms_recovery::SMSRecovery" base_class_type="nav2_core::Recovery">
    <description>This is an example plugin which produces an SMS text message recovery.</description>
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
pluginlib_export_plugin_description_file(nav2_core recovery_plugin.xml)
```

4. 插件的描述应该被添加到 `package.xml`中

```xml-dtd
<export>
  <build_type>ament_cmake</build_type>
  <nav2_core plugin="${prefix}/recovery_plugin.xml" />
</export>
```

5. 编译，生成插件

# 添加参数

修改原来的

```
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

为

```
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait", "call_for_help"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    call_for_help:
      plugin: "nav2_sms_recovery/SMSRecovery"
      account_sid: ... # your sid
      auth_token: ... # your token
      from_number: ... # your number
      to_number: ... # the operations center number
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

# 运行

```
$ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml
```

打开另外一个窗口

```
$ ros2 action send_goal "call_for_help" nav2_sms_recovery/action/SmsRecovery "Help! Robot 42 is being mean :( Tell him to stop!"
```



