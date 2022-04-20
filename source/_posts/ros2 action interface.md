---
title: ros2 action interface
date: 2021-04-02 09:59:22
tags: ros2
---

# ROS2接口扩展

目标：了解更多方法来在ROS 2中实现自定义接口

虽然最好来说定义接口是在一个新的package中，比如上一节中的tutorial_interface中，但是有时也会把接口的定义和使用都再一个包里面

## 如何使用一个已存在的interface

# 在类中使用参数

**Goal:** Create and run a class with ROS parameters using Python (rclpy).

在制作自己的节点时，您有时需要添加可以从启动文件设置的参数。

本教程将向您展示如何在Python类中创建这些参数，以及如何将它们设置为在启动文件中。

```
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
```

## 更新update.xml以及setup.py

```
<description>Python parameter tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

## python代码

再`dev_ws/src/python_parameters/python_parameters` 目录下，新建`python_parameters_node.py`

```
import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 代码审查

注意：在获取或设置之前必须声明参数，或者将提出`parameternotdeclaredException`异常。

```
import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
```

下一段代码创建类和构造函数。timer被初始化(timer_period设置为2秒)，这将导致timer_callback函数每两秒执行一次。构造函数的self.declare_parameter('my_parameter'， 'world')一行创建了一个名为my_parameter的形参，默认值为world。

```
class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')
```

timer_callback函数的第一行从节点获取参数my_parameter，并将其存储在my_param中。接下来，get_logger函数确保打印了消息。然后，我们将参数' my_parameter '设置回默认的字符串值' world '。

```
def timer_callback(self):
    my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

    self.get_logger().info('Hello %s!' % my_param)

    my_new_param = rclpy.parameter.Parameter(
        'my_parameter',
        rclpy.Parameter.Type.STRING,
        'world'
    )
    all_new_parameters = [my_new_param]
    self.set_parameters(all_new_parameters)
```

timer_callback之后是初始化ROS 2的主函数。然后定义名为node的MinimalParam类的实例。最后，rclpy.spin开始处理来自节点的数据。

```
def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 添加对参数的描述parameterDescriptor.

还可以为参数设置描述符。描述符允许您指定参数的类型和一些描述文本。为了让它工作，`__init__`代码必须更改为:

```
class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        from rcl_interfaces.msg import ParameterDescriptor
        my_parameter_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                      description='This parameter is mine!')

        self.declare_parameter('my_parameter',
                               'default value for my_parameter',
                               my_parameter_descriptor)
```

其余的代码保持不变。运行节点之后，可以运行`ros2 param description /minimal_param_node my_parameter`查看类型和描述。

## Add an entry point

```
entry_points={
    'console_scripts': [
        'param_talker = python_parameters.python_parameters_node:main',
    ],
},
```

## build and run

```
rosdep install -i --from-path src --rosdistro dashing -y
cd ~/dev_ws
colcon build
```

新打开一个终端，两个source之后

```
ros2 run python_parameters param_talker
```

会出现

```
[INFO] [minimal_param_node]: Hello world!
[INFO] [minimal_param_node]: Hello world1!
[INFO] [minimal_param_node]: Hello world1!
[INFO] [minimal_param_node]: Hello world1!
```

两种方式去改变这个参数

### 通过命令行

我们可以通过命令行还修改参数的设置

首先查看现在有什么参数

```
ros2 param list
/minimal_param_node:
  my_parameter
  use_sim_time
```

接下来使用命令行还修改参数

```
ros2 param set /minimal_param_node my_parameter earth
```

现在去查看另外一个终端 就会出现hello earth

### 通过launch文件

您还可以在启动文件中设置参数，但首先需要添加一个启动目录。在dev_ws/src/python_parameters/目录中，创建一个名为launch的新目录。在这里，创建一个名为python_parameters_launch.py的新文件

`emulate_tty`, which prints output to the console, is not available in Dashing.

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            node_executable='param_talker',
            node_name='custom_parameter_node',
            output='screen',
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])
```

在这里，您可以看到我们在启动Node Parameter_Node时将My_Parameter设置为earth。

现在打开setup.py文件。将导入语句添加到文件的顶部，并将另一条新语句添加到data_files参数中，以包含所有启动文件:

```
ros2 launch python_parameters python_parameters_launch.py
```

# Summary

您创建了一个带有自定义参数的节点，该参数可以从启动文件或命令行设置。您编写了一个参数对话器的代码:一个Python节点声明，然后循环获取和设置一个字符串参数。您添加了入口点，以便可以构建和运行它，并使用ros2 param与参数对话器交互。

# creating an action

```
cd dev_ws/src
ros2 pkg create action_tutorials_interfaces
```

## 定义一个action

action在.action文件中定义:

```
# Request
---
# Result
---
# Feedback
```

操作定义由三个用---分隔的消息定义组成。

- Request消息从操作客户端发送到启动新goal的操作服务器。
- Result 从action服务端出到action客户端，当goal已经完成。
- feedback 从动作服务器周期性地发送到具有关于目标的更新的动作客户端。

action的一个实例通常被称为goal。

假设我们想定义一个新的动作“Fibonacci”来计算Fibonacci序列。

```
cd action_tutorials_interfaces
mkdir action
vi Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

目标请求是我们想要计算的斐波纳契序列的order，结果是最终sequence，反馈是到目前为止计算的Partial_sequence。

## building an action

在代码中使用新的Fibonacci动作类型之前，必须将定义传递给rosidl code generation pipeline.

这是通过在action_tutorials_interfaces的ament_package()行之前添加以下几行CMakeLists.txt来完成的:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

我们还应该将所需的依赖项添加到package.xml中:

注意，我们需要依赖于action_msgs，因为action操作定义包括其他元数据（例如目标ID）。

打开`package.xml`

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

然后进行

```
ros2 action show action_tutorials_interfaces/action/Fibonacci
```

# 写一个action的客户端和服务端

action是ROS 2中异步通信的一种形式。action客户端向action服务器发送goal请求。action服务器向action客户端发送目标feedback和result。

## 写一个action的服务端

下面写一个服务端用之前定义好的action。为了教程的简单，我们会把服务端单独做成一个文件

打开home目录，建立文件`fibonacci_action_server.py`

```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

第8行定义了一个类FibonacciActionServer，它是Node的子类。通过调用节点的构造函数来初始化该类，将我们的节点命名为fibonacci_action_server:

```
super().__init__('fibonacci_action_server')
```

在构造函数中，我们还实例化了一个新的action server:

```
self._action_server = ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    self.execute_callback)
```

action server需要四个参数:

1. ROS 2节点将动作客户端添加到：self。
2. 动作的类型：Fibonacci（以第5行导入）。action_tutorials_interfaces
3. 动作名称：'fibonacci'。
4. 用于执行接受的目标的回调函数：self.execute_callback。此回调必须返回操作类型的结果消息。

```
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')action_tutorials_interfaces
    result = Fibonacci.Result()
    return result
python3 fibonacci_action_server.py
```

在另一个终端中，我们可以使用命令行界面来发送目标：

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

将看见

```
Waiting for an action server to become available...
Sending goal:
     order: 5

Goal accepted with ID: 0a7d8ac416a14368b467604786edb81a

Result:
    sequence: []

Goal finished with status: ABORTED
```

再服务端，我们有

```
[INFO] [fibonacci_action_server]: Executing goal...
[WARN] [fibonacci_action_server]: Goal state not set, assuming aborted. Goal ID: [ 10 125 138 196  22 161  67 104 180 103  96  71 134 237 184  26]
```

在运行操作服务器的终端中，您应该看到一条日志消息“Executing goal...”，后面是一个目标状态未设置的警告。默认情况下，如果在执行回调中没有设置目标句柄状态，它将假定为aborted状态。

我们可以使用目标句柄上的succeed()方法来指示目标成功:

```
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')
    goal_handle.succeed()
    result = Fibonacci.Result()
    return result
```

现在，如果重新启动操作服务器并发送另一个goal，您应该会看到goal已经完成，状态为SUCCEEDED。

```
Waiting for an action server to become available...
Sending goal:
     order: 5

Goal accepted with ID: 7ca482d0b76546cdbaaa4bf03647d8fa

Result:
    sequence: []

Goal finished with status: SUCCEEDED
```

现在，让我们的目标执行实际计算并返回所请求的斐波那契序列:

```
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')

    sequence = [0, 1]

    for i in range(1, goal_handle.request.order):
        sequence.append(sequence[i] + sequence[i-1])

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = sequence
    return result
```

在计算序列之后，我们在返回之前将其分配给result消息字段。

.再次重启操作服务器并发送另一个目标。您应该看到目标以正确的结果序列结束。

```
Waiting for an action server to become available...
Sending goal:
     order: 5

Goal accepted with ID: 06a9917c3e664f6d8e39f51bc5f31802

Result:
    sequence: [0, 1, 1, 2, 3, 5]

Goal finished with status: SUCCEEDED
```

### Publishing feedback

关于action的一个很好的事情是在目标执行期间提供对动作客户端的反馈的能力。我们可以通过调用目标句柄的publish_feedback（）方法使我们的action服务器发布action客户端的反馈。

我们将替换sequence变量，并使用feedback消息来存储序列。在for循环中每次更新反馈信息后，我们都会发布feedback信息并休眠以获得戏剧性效果:

```
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```

在重新启动action服务器后，我们可以通过使用命令行工具和--feedback选项确认反馈现在已经发布了:

## 写一个action客户端

也是单个文件,`fibonacci_action_client.py`

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```

我们还在FibonAcciaInctionClient类中定义了一个方法send_goal：此方法等待可用的action服务器，然后向服务器发送目标。它返回一个future，这个future是我们未来可以获取结果的一个对象

### 获得结果

我们可以发送一个目标，但是我们如何知道它何时完成呢?我们可以通过几个步骤得到结果信息。首先，我们需要为发送的目标获取一个目标句柄。然后，我们可以使用目标句柄来请求结果。

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

ActionClient.send_goal_async()方法为目标句柄返回一个future。首先，当future完成时注册一个回调函数:

```
self._send_goal_future.add_done_callback(self.goal_response_callback)
```

注意，当操作服务器接受或拒绝goal请求时，future就完成了。让我们更详细地看看goal_response_callback。

我们可以检查目标是否被拒绝，并提前返回，因为我们知道不会有结果:

```
def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal rejected :(')
        return

    self.get_logger().info('Goal accepted :)')
```

现在我们已经有了一个目标句柄，我们可以使用它来通过get_result_async()方法请求结果。

与发送目标类似，我们将得到一个future，当结果准备好时，它将完成。让我们注册一个回调，就像我们为目标响应所做的那样:

```
self._get_result_future = goal_handle.get_result_async()
self._get_result_future.add_done_callback(self.get_result_callback)
```

在回调中，我们记录了结果序列并关闭了ROS 2以获得干净的退出:

```
def get_result_callback(self, future):
    result = future.result().result
    self.get_logger().info('Result: {0}'.format(result.sequence))
    rclpy.shutdown()
```

再一次尝试运行服务端和客户端的代码

客户端

```
[INFO] [fibonacci_action_client]: Goal accepted :)
[INFO] [fibonacci_action_client]: Result: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
```

服务端

```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
```

### 获得feedback

我们的action客户端可以发送目标。nice！但如果我们可以获得关于我们从Action Server发送的目标的一些反馈，那就太好了。

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

以下是反馈信息的回调函数:

```
def feedback_callback(self, feedback_msg):
       feedback = feedback_msg.feedback
       self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
```

在回调中，我们获得消息的反馈部分，并将partial_sequence字段打印到屏幕上。

我们需要使用Action Client注册回调。当我们发送目标时，这将通过此外将回调传递给行动客户端来实现：

```
self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
```

运行客户端

```
[INFO] [fibonacci_action_client]: Goal accepted :)
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34])
[INFO] [fibonacci_action_client]: Received feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
[INFO] [fibonacci_action_client]: Result: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
```

服务端

```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55])
```

# 使用Launch启动/监视多个节点

## ros2 启动系统

ROS 2中的启动系统负责帮助用户描述他们的系统配置，然后按照描述执行。系统的配置包括运行什么程序，在哪里运行它们，传递什么参数，以及ROS特定的约定，这些约定通过为组件提供不同的配置，使组件在整个系统中易于重用，它还负责监视已启动流程的状态，并报告和/或对这些流程状态的变化作出反应。

用Python编写的启动文件可以启动和停止不同的节点，也可以触发和处理各种事件。提供这个框架的包是launch_ros，它使用底层的非特定于ros的启动框架。

## 写一个启动文件

当我们进行了

```
ros2 pkg create --build-type = ament_python my_package --dependency [deps]
```

应该就会有如下文件结构, 再建一个了launch文件夹

```
src/
    my_package/
        launch/
        setup.py
        setup.cfg
        package.xml
```

为了让colcon找到启动文件，我们需要使用setup的data_files参数通知Python的安装工具我们的启动文件。

在setup.py里面

```
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ]
)
```

## 写一个launch文件

在启动目录中，使用.launch.py后缀创建一个新的启动文件。例如my_script.launch.py。

```
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='talker', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    ])
```

.launch.py这个后缀其实并没有要求，`_lannch.py`也是可以的。

launch文件应该定义一个generate_launch_description，并且返回一个LaunchDescription。这样ros2 launch才可以找到

### 使用

虽然可以将启动文件编写为独立的脚本，但ROS中典型的使用方式是由ROS 2工具调用启动文件。

再colcon build之后，应该可以调用

```
ros2 launch my_package script.launch.py
```