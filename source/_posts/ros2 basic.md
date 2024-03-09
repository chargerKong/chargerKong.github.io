---
title: ros2 basic
date: 2021-03-27 09:59:22
tags: ros2
---

# 理解ros2 service

服务是ROS图上节点通信的另一种方法。服务基于呼叫响应( call-and-response )模型，而不是主题的发布者-订阅(publisher-subscriber )者模型。主题允许节点订阅数据流并获得持续更新，而服务只在客户端特定地调用它们时才提供数据

![../../_images/Service-SingleServiceClient.gif](https://docs.ros.org/en/dashing/_images/Service-SingleServiceClient.gif)

![../../_images/Service-MultipleServiceClient.gif](https://docs.ros.org/en/dashing/_images/Service-MultipleServiceClient.gif)

## 1. 启动

打开俩节点

```
ros2 run turtlesim turtlesim_node
```

换终端

```
ros2 run turtle_teleop_key
```

## 2. ros2 server list

打开一个新终端，看一下

```
ros2 service list
```

应该返回

```
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```

您将看到两个节点都有相同的6个服务，这些服务的名称中都带有`parameters`。在ROS 2中，几乎每个节点都有这些构建参数的基础设施服务。关于参数的事情下一节讨论

现在，让我们专注于`turtlesim-specific`的服务。`/clear`, `/kill`, `/reset`, `/spawn`, `/turtle1/set_pen`, `/turtle1/teleport_absolute`, 以及 `/turtle1/teleport_relative`.

## 3. ros2 service type(dashing 不支持)

`service`是有类型的，这些类型描述着`service`的请求和响应数据是如何构建的。服务类型的定义与主题类型类似，不同的是，服务类型包括两部分：一个消息用于请求，另一个消息用于响应。

可以通过rqt选择了服务之后直接查看，比如点击`/clear`, 它的type是`std_srvs/srv/Empty`

`Empty`类型意味着服务调用在发出请求时不发送数据，在接收响应时不接收数据

或者直接通过

```
ros2 service list -t
```

来查看服务的类型

## 4.ros2 service find (dashing不支持)

通过指定寻找特定的类型来找寻找服务

## 5.ros2 interface show

查看某一个接口的数据类型

```
ros2 interface show geometry_msgs/msg/Twist
```





查看某一个服务下的数据类型，show后面接服务的type，这个可以通过rqt查看或者`ros2 service list -t`。例如`\spawn`的类型`turtlesim/srv/Spawn`

要查看`/ spawn`服务 调用和请求中的参数，请运行以下命令：

```
ros2 srv show turtlesim/srv/Spawn
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

再`---`的上方的信息告诉我们调用`/ spawn`所需的参数有`x`,`y`,`theta`, 它是用于确定初始乌龟的位置

## 6. ros2 service call

现在，您知道什么是服务类型，如何找到服务的类型以及如何找到该类型的参数的结构，您可以使用以下方法调用服务：

```
ros2 service call <service_name> <service_type> <arguments>
```

`<arguments>` is optional

这就是相当于再`rqt`进行call的操作，

例如

```
ros2 service call /clear std_srvs/srv/Empty
```

`call`的是 `\clear`服务，类型为`std_srvs/srv/Empty`。这个操作会是的小乌龟的轨迹被清除。

接下来看`\spawn`如何操作

```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0,y: 0.0,theta: 3.0}"
```

返回

```
waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='None')

response:
turtlesim.srv.Spawn_Response(name='None')
```

## summary

节点可以使用ROS 2中的服务进行通信。服务仅在该节点明确request时，才将信息传递给该节点，并且每个请求只能这样做一次（而不是连续流）。您通常不希望使用服务进行连续通信，topic或者actions获取会比较合适连续通信

# 理解ros2 parameters

parameters是用于配置节点的参数，可以把他视为节点的配置。节点可以将参数存储为整数，浮点数，布尔值，字符串和列表。在ROS 2中，每个节点都维护自己的参数。所有参数都是可动态重新配置的，并且是基于ROS 2服务构建的。

## 1.启动小乌龟服务

打开俩节点

```
ros2 run turtlesim turtlesim_node
```

换终端

```
ros2 run turtle_teleop_key
```

## 2.ros2 param list

```
ros2 param list
```

可以查看每一个节点的参数, 可以看见如下情况

```
/teleop_turtle:
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
```

每一个节点都会有一个`use_sim_time`

根据它们的名称，可以发现`/turtlesim`的参数使用RGB颜色值确定turtlesim窗口的背景颜色。

要确定参数类型，可以使用`ros2 param get`

## 3. ros2 param get

获取参数值的命令

```
ros2 param get <node_name> <parameter_name>
```

例如

```
ros2 param get /turtlesim background_b
```

返回

```
Integer value is: 86
```

现在您知道background_b拥有一个整数值。

## 4.ros2 param set

要在运行时更改参数的值，请使用以下命令：

```
ros2 param set <node_name> <parameter_name> <value>
```

例如

```
ros2 param set /turtlesim background_b 100
```

这只是对目前窗口的更改，要永久保存，您可以保存设置更改，并在下次启动节点时重新加载它们。

## 5.ros2 param dump

dashing没有这个命令，但是可以从文件里去加载

将以下内容保存在名为`./turtlesim.yaml`的文件中：

```
turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
    use_sim_time: false
```

如果您希望将来使用相同的参数重新加载节点，则转储参数会很方便。

## 6.加载参数文件

利用保存的参数值来启动同一个node

```
ros2 run turtlesim turtlesim_node __params:=./turtlesim.yaml
```

## Summary

节点具有定义其默认配置值的参数。您可以从命令行获取和设置参数值。您还可以保存参数设置以在新会话中重新加载。

# 理解ROS2 actions

actions 应用于长时间的通信。由三部分组成：目标，结果和反馈。actions建立在topic和service上。它们的功能类似于service，除了动作是可抢占的(您可以在执行时取消它们)。它们还提供稳定的反馈，而不是只返回一个响应的服务。

操作使用客户端-服务器模型，类似于发布者-订阅者模型。“action client”节点向“action service”节点发送目标，后者确认目标并返回反馈流和结果

![../_images/Action-SingleActionClient.gif](https://docs.ros.org/en/dashing/_images/Action-SingleActionClient.gif)

## 1.启动

打开俩节点

```
ros2 run turtlesim turtlesim_node
```

换终端

```
ros2 run turtle_teleop_key
```

## 2. 使用actions

启动`/ teleop_turtle`节点时，您将在终端中看到以下消息：

```
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```

第二条对应于一个actions。

请注意，字母键G | B | V | C | D | E | R | T在键盘上的F键周围形成一个“框”，对应着各个方向。

注意`/ turtlesim`节点正在运行的终端，每次按这些键之一，就向一个目标服务器发送`goal`，该服务器是`/ turtlesim`节点的一部分。目标是旋转乌龟以使其朝向特定方向。乌龟完成旋转后，将显示一条消息，传达目标的结果：

```
[INFO] [turtlesim]: Rotation goal completed successfully
```

F键将取消目标的中间执行，表明操作具有可抢占的功能。

不仅客户端(您在teleop中的输入)可以抢占目标，服务器端(`/turtlesim`节点)也可以。

尝试先按D键，然后再按G键，然后才能完成第一次旋转。在运行`/ turtlesim`节点的终端中，您将看到以下消息：

```
[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
```

服务器端中止了第一个目标，因为它被中断了。

## 3.ros2 node info

查看某一package的节点的actions信息.

我们现在有两个节点`/turtlesim`和`/teleop_turtle`

```
ros2 node info /turtlesim
```

这将返回`/ turtlesim`的订阅者，发布者，服务，动作服务器和动作客户端的列表：

```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
    /turtle1/rotate_absolute/_action/feedback: turtlesim/action/RotateAbsolute_FeedbackMessage
    /turtle1/rotate_absolute/_action/status: action_msgs/msg/GoalStatusArray
  Services:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/rotate_absolute/_action/cancel_goal: action_msgs/srv/CancelGoal
    /turtle1/rotate_absolute/_action/get_result: turtlesim/action/RotateAbsolute_GetResult
    /turtle1/rotate_absolute/_action/send_goal: turtlesim/action/RotateAbsolute_SendGoal
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
```

请注意，`/ turtlesim的/ turtle1 / rotate_absolute`操作已在service中，这意味着`/ turtlesim`会响应`/ turtle1 / rotate_absolute`操作并提供反馈。

`/ teleop_turtle`节点在“Action Clients”下具有名称`/ turtle1 / rotate_absolute`，这意味着它为该操作名称发送目标。

```
ros2 node info /teleop_turtle
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/rotate_absolute/_action/feedback: turtlesim/action/RotateAbsolute_FeedbackMessage
    /turtle1/rotate_absolute/_action/status: action_msgs/msg/GoalStatusArray
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Services:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /turtle1/rotate_absolute/_action/cancel_goal: action_msgs/srv/CancelGoal
    /turtle1/rotate_absolute/_action/get_result: turtlesim/action/RotateAbsolute_GetResult
    /turtle1/rotate_absolute/_action/send_goal: turtlesim/action/RotateAbsolute_SendGoal
```

## 4.ros2 action list

```
ros2 action list -t
```

返回

```
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

左边是action的名字，括号里面是action的类型。这会再命令行进行执行操作和写代码的时候会用到

## 5.ros2 action info

```
ros2 action info <action_name>
ros2 action info /turtle1/rotate_absolute
```

返回

```
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

`/ teleop_turtle`节点具有一个action clients，`/ turtlesim`节点具有一个用于`/ turtle1 / rotate_absolute`动作的action servers。

## 6.ros2 interface show

在发送或执行自己的行动目标之前，你还需要了解行动类型的结构。回想一下，您在运行`ros2 action list -t`命令时标识了`/turtle1/rotate_absolute`的类型。在终端中输入以下带有操作类型的命令:

```
ros2 action show turtlesim/action/RotateAbsolute
```

show 后面是需要加类型的

返回

```
The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

第一个是goal的数据类型，第二个是result 的数据类型，最后一个是feedback的数据类型

注释：

```
ros2 action list -t
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

## 7. ros2 action send_goal

下面利用如下语法来在命令行发送一条action goal

```
ros2 action send_goal <action_name> <action_type> <values>
```

`<values>` need to be in YAML format.

输入一下命令的时候，注意看小乌龟

```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "theta: 0.5"
```

注释：value里面的数据类型可以通过show来查看

返回

```
Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
```

所有目标都有唯一的ID，如返回消息所示。您还可以看到结果，一个名为delta的字段，它是到起始位置的位移。

要查看此goal的feedback，请向您运行的最后一个命令添加——feedback。首先，确保你改变了的值。

```
Sending goal:
   theta: -1.57

Goal accepted with ID: e6092c831f994afda92f0086f220da27

Feedback:
  remaining: -3.1268222332000732

Feedback:
  remaining: -3.1108222007751465

…

Result:
  delta: 3.1200008392333984

Goal finished with status: SUCCEEDED
```

你将继续收到反馈，剩下的弧度，直到目标完成。

## Summary

Actions 就像service 一样，允许您执行长时间运行的任务，提供定期的反馈，并且可以取消。

机器人系统可能会使用actions进行导航。一个action goal可以告诉机器人移动到一个位置。当机器人导航到该位置时，它可以在沿途发送更新信息(即反馈)，然后在到达目的地时发送最终结果消息。Turtlesim有一个action server，action 客户端可以将目标发送到这个服务器上进行海龟的旋转。