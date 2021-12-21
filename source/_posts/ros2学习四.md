---
title: ros2学习四
date: 2021-04-01 09:59:22
tags: ros2
---

# ros2学习四

## 写一个简单的publisher和subscriber

在本教程中，您将创建节点，这些节点以字符串消息的形式在主题之间相互传递信息。这里使用的例子是一个简单的“talker”和“listener”系统;一个节点发布数据，另一个节点订阅主题，以便接收数据。

## 1.创建一个package

```
source ros2_dashing/install/setup.bash 
cd dev_ws/src/
ros2 pkg create --build-type ament_python py_pubsub
```

## 2. 写publisher的一个node

下载一个example talker。

```
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

打开文件

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.1.代码审视

节点使用该类型来构造它在主题上传递的数据

```
from std_msgs.msg import String
```

下面建立一个类，继承了Node

```
self.publisher_ = self.create_publisher(String, 'topic', 10)
```

这里表示node传送的信息是通过一个名为‘topic’的一个topic，并且消息的类型是String类，队列长度为10.

此队列大小是必须的一个设置，如果订阅者接收队列消息的速度不够快，它将限制队列消息的数量。

接下来，创建一个带有回调的计时器，以每0.5秒执行一次。 self.i是在回调中使用的计数器。

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

timer_callback创建一个附加计数器值的消息，并使用get_logger().info将其发布到控制台

```
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

最后定义main函数

```
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

首先初始化rclpy库，然后创建节点，然后它“spin”节点，以便调用它的回调。

### 2.2添加依赖项

回到之前所说的，完善一下description，licence

```
cd /dev_ws/src/py_pubsub
vi package.xml
```

除了完善上述信息，还需要添加一下执行时所需要的依赖内容

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### 2.3 添加入口点

再一次打开`setup.py`，完善一下description 和 licence

并且添加以下内容

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

这里写的是py_pubsub文件夹下的 py_pubsub/publisher_member_function 文件

### 2.4 查看setup.cfg

setup.cfg文件的内容应该自动正确填充，如下所示:

```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```

这只是告诉setuptools将可执行文件放到lib中，因为ros2 run将在那里查找它们。

您现在可以构建自己的包、source local setup文件并运行它，但是让我们首先创建subscriber节点，以便您可以看到工作中的整个系统。

## 3.写一个subscriber节点

```
cd ~/dev_ws/src/py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/dashing/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

### 3.1审视代码

查看`subscriber_member_function.py`

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning  

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

订阅服务器节点的代码与发布服务器节点的代码几乎相同。构造函数使用与publisher相同的参数创建subscriber。回想一下主题教程，发布者和订阅者使用的主题名称和消息类型必须匹配，才允许它们通信。

```
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

subscriber的构造函数和回调函数不包括任何计时器定义，因为它不需要计时器定义。一旦收到消息，它的回调函数就会被调用。

回调定义只是将一条信息消息及其接收到的数据打印到控制台。回想一下，发布者定义了msg.data ='Hello World：％d'％self.i

```
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

主要定义几乎完全相同，用subscriber代替了publisher的创建和旋转。

```
minimal_subscriber = MinimalSubscriber()

rclpy.spin(minimal_subscriber)
```

由于此节点与发布者具有相同的依赖关系，因此没有新内容可添加到package.xml中。 setup.cfg文件也可以保持不变。

### 3.2 添加一个entry point

打开setup.py

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

## 4.build and run

您可能已经安装了rclpy和std_msgs软件包作为ROS 2系统的一部分。最好的做法是在工作区的根目录（dev_ws）中运行rosdep，以在构建之前检查缺少的依赖项：

```
rosdep install -i --from-path src --rosdistro dashing -y
```

前往工作区的根目录

```
cd ~/dev_ws
colcon build
```

打开新终端运行，记得先source

## 5.Summary

您创建了两个节点来发布和订阅主题上的数据。在运行它们之前，您已将它们的依赖项和入口点添加到程序包配置文件中。

## 写一个简单的客户端和服务端

## 1. 建立一个package

进入src目录

```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

`--dependencies`代表需要的依赖，他会自动的添加到setup.py里面，`example_interfaces` 是包含`.srv`文件的软件包，您需要用它来构建请求和响应

```
int64 a
int64 b
---
int64 sum
```

前两行是请求的参数，短划线下方是响应。

### object has no attribute '_parameters'1.1 更新package.xml和setup.py

更新`<description>,<maintainer>,<licence>`

## 2.写服务端节点

打开

```
cd ~/dev_ws/src/py_srvcli/py_srvcli
service_memeber_function.py
```

输入以下内容

```
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.1代码审查

可以通过`ros2 srv show example_interface/srv/AddTwoInts`查看

返回

```
int64 a
int64 b
---
int64 sum
```

client 这里会发布a和b, 服务端返回一个response，sum

第一条import语句从example_interfaces包中导入AddTwoInts服务类型。

```
from example_interfaces.srv import AddTwoInts
```

MinimalService类构造函数初始化名称为minimal_service的节点。然后，它创建一个service并定义类型、名称和回调。

```
def __init__(self):
    super().__init__('minimal_service')
    self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
```

服务回调的定义接收请求数据，将其求和，然后将总和作为响应返回。

```
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    return response
```

最后，main类初始化ROS 2 Python**客户端**(服务端？)库，实例化MinimalService类以创建服务节点，旋转节点以处理回调。

### 2.2 添加启动项

要允许ros2 run命令运行您的节点，必须将入口点添加到setup.py（位于dev_ws / src / py_srvcli目录中）。

## 3.写客户端代码

```
cd ~/dev_ws/src/py_srvcli/py_srvcli
vi client_member_function.py
```

输入

```
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.1 代码审查

客户端的唯一不同的导入语句是import sys。客户端节点代码使用sys.argv来访问请求的命令行输入参数。

构造函数构造了一个与服务节点具有相同类型和名称的客户端，构造函数中的while循环每秒检查一次与客户端类型和名称匹配的服务是否可用。

客户主体的唯一显着差异是while循环。只要系统正在运行，循环就会检查`future`是否有服务响应。如果服务已发送响应，则结果将写入日志消息中

## 3.2 添加entry point

与服务节点一样，您还必须添加一个入口点才能运行客户端节点。

```
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

## 4.build and run

首先利用`rosdep`检查一下依赖是否都已经安装

```
rosdep install -i --from-path src --rosdistro dashing -y
```

去根目录进行

```
colcon build --packages-select py_srvcli
```

新打开一个终端运行，工作区根目录，运行服务端

```
source ~/ros2_dashing/install/setup.bash
source install/local_setup.bash 
ros2 run py_srvcli service
```

打开另外一个终端，运行客户端

```
source ~/ros2_dashing/install/setup.bash
source install/local_setup.bash 
ros2 run py_srvcli client 2 4
```

这是，客户端返回

```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 4 = 6
```

服务端返回

```
[INFO] [minimal_service]: Incoming request
a: 2 b: 4
```

## 5.Summary

您创建了两个节点来通过服务请求和响应数据。您将它们的依赖项和可执行文件添加到程序包配置文件中，以便可以构建和运行它们，从而使您可以看到正在使用的服务/客户端系统。

# 自定义接口

目标：定义自定义接口文件（.msg和.srv），并将其与Python和C ++节点一起使用。

## 1.新建一个package

```
cd ~/dev_ws/src
ros2 pkg create --build-type ament_python tutorial_interfaces
```

请注意，它是一个CMake软件包。目前还没有一种在纯Python包中生成.msg或.srv文件的方法。您可以在CMake包中创建一个自定义接口，然后在Python节点中使用它，这将在最后一部分中介绍。

将.msg和.srv文件保存在包中各自的目录中是一种很好的做法。在dev_ws/src/tutorial_interfaces中创建目录:

```
mkdir msg

mkdir srv
```

## 2.自定义definition

在`tutorial_interfaces/msg`文件里新建文件`Num.msg`

```
int64 num
```

在`tutorial_interfaces/s`rv文件里新建文件`AddThreeInts.srv`

```
int64 a
int64 b
int64 c
---
int64 sum
```

这是您的自定义服务，它请求三个名为a，b和c的整数，并以一个称为sum的整数进行响应。

## 3.CMakeLists

要将定义的接口转换为特定语言的代码(如c++和Python)，以便它们可以在这些语言中使用，请在CMakeLists.txt中添加以下行:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )
```

## 4.package.xml

由于接口依赖于rosidl_default_generators来生成特定于语言的代码，因此您需要声明对它的依赖关系。将以下行添加到package.xml

```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

## 5.build

```
colcon build --packages-select tutorial_interfaces
```

现在，其他ROS 2软件包将可以发现这些接口。

## 6.确定msg和srv的创建

现在打开一个新的终端

```
source . install/setup.bash
```

就可以尝试使用

```
ros2 msg show tutorial_interfaces/msg/Num
```

会返回

```
int64 num
```

这将作为发布和订阅传输的一个格式

还可以尝试

```
ros2 srv show tutorial_interfaces/srv/AddThreeInts
```

返回

```
int64 a
int64 b
int64 c
---
int64 sum
```

这将是作为服务中的调用和请求的参数使用

## 7 测试一下新的interface

您可以使用在前面的教程中创建的包。对节点、CMakeLists和包文件做一些简单的修改就可以使用新的接口

### 7.1 利用pub/sub来测试Num.msg

打开pub/sub下的Publisher：

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

subscriber:

```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

package.xml

```
<exec_depend>tutorial_interfaces</exec_depend>
```

进入根目录

```
colcon build --packages-select py_pubsub
```

打开俩新的终端

```
ros2 run py_pubsub talker
ros2 run py_pubsub listener
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

### 7.2 通过service/client 来测试 `AddThreeInts.srv`

修改之前的项目`py_srvcli`

打开服务端代码

```
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

打开客户端代码

```
from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

package.xml

```
colcon build --packages-select py_srvcli
```

打开俩终端

```
ros2 run py_srvcli service
ros2 run py_srvcli client 2 3 1
```

## Summary

在本教程中，您学习了如何在自己的程序包中创建自定义接口，以及如何在其他程序包中利用这些接口。