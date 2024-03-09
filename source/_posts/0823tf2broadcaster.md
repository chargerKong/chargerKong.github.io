---
title: tf2 broadcaster listener
date: 2021-08-23 15:59:22
tags: [tf2,ros2]
---

在之前的教程中，我们学会了如何创建了一个静态变换

# Python 

## 写一个broadcaster

```python
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf_transformations

from turtlesim.msg import Pose


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter
        self.declare_parameter('turtlename', 'turtle')
        self.turtlename = self.get_parameter(
            'turtlename').get_parameter_value().string_value

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)
        self.subscription

    def handle_turtle_pose(self, msg):
        # Initialize the transform broadcaster
        br = TransformBroadcaster(self)
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        br.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
```

## 代码审查

我们首先来看发布到tf2的位姿的相关代码，首先我们定义了一个参数`turtlename`，默认值为`turtle`

```
self.declare_parameter('turtlename', 'turtle')
self.turtlename = self.get_parameter(
    'turtlename').get_parameter_value().string_value
```

之后，结点订阅了一个topic，在每一次收到消息的时候运行函数`handle_turtle_pose`

```
self .subscription = self.create_subscription(
    Pose,
    f'/{self.turtlename}/pose',
    self.handle_turtle_pose,
    1)
```

 现在，我们创建一个Transform对象，并且给予一些元数据

1. 建立一个时间戳
2. 给parent frame创建一个名字
3. 给child_frame一个名字，在这里，就是他自己

```
t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = 'world'
t.child_frame_id = self.turtlename
```

变换为从world到自己。

下面把3D的位姿转换到3D的transform

```
# Turtle only exists in 2D, thus we get x and y translation
# coordinates from the message and set the z coordinate to 0
t.transform.translation.x = msg.x
t.transform.translation.y = msg.y
t.transform.translation.z = 0.0

# For the same reason, turtle can only rotate around one axis
# and this why we set rotation in x and y to 0 and obtain
# rotation in z axis from the message
q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
t.transform.rotation.x = q[0]
t.transform.rotation.y = q[1]
t.transform.rotation.z = q[2]
t.transform.rotation.w = q[3]
```

之后，进行消息的发布

```
# Send the transformation
br.sendTransform(t)
```

> You can also publish static transforms with the same pattern by instantiating a `tf2_ros.StaticTransformBroadcaster` instead of a `tf2_ros.TransformBroadcaster`. The static transforms will be published on the `/tf_static` topic and will be sent only when required, not periodically. For more details see [here](https://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html#writingatf2staticbroadcasterpy).

## 写一个listener

```python
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'turtle1')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Create a client to spawn a turtle
        self.client = self.create_client(Spawn, 'spawn')

        # Check if the service is available
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

        # Initialize request with turtle name and coordinates
        # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
        request = Spawn.Request()
        request.name = 'turtle2'
        request.x = float(4)
        request.y = float(2)
        request.theta = float(0)
        # Call request
        self.client.call_async(request)

        # Create turtle2 velocity publisher
        self.turtle_vel_ = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self._output_timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            now = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now,
                timeout=Duration(seconds=1.0))
        except LookupException:
            self.get_logger().info('transform not ready')
            return

        msg = Twist()
        msg.angular.z = 1.0 * math.atan2(
            trans.transform.translation.y,
            trans.transform.translation.x)

        msg.linear.x = 0.5 * math.sqrt(
            trans.transform.translation.x ** 2 +
            trans.transform.translation.y ** 2)

        self.turtle_vel_.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
```

在上面的代码中，主要逻辑为：孵化一个小乌龟`turtle2`，监听一个从1到2的变换，让`turtle2`实时往turtle1的变换位置跑即可

tf2_ros包里面已经包装好了 `TransformListener` 可以让接受变换消息更容易，

```
from tf2_ros.transform_listener import TransformListener
```

在这里，我们创建了一个 TransformListener 对象。 创建侦听器后，它开始通过线路接收 tf2 转换，并将它们缓冲最多 10 秒。

```
self._tf_listener = TransformListener(self._tf_buffer, self)
```

最后，我们调用了一个lookup_transform函数去找到一个特定的变换，其参数为

1. target frame
2. source frame
3. 时刻

然后发布小乌龟的速度



## launch文件

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ...,
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])
```

## 添加一个frame

```
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

   def __init__(self):
      super().__init__('fixed_frame_tf2_broadcaster')
      self.br = TransformBroadcaster(self)
      self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
      t = TransformStamped()
      t.header.stamp = self.get_clock().now().to_msg()
      t.header.frame_id = 'turtle1'
      t.child_frame_id = 'carrot1'
      t.transform.translation.x = 0.0
      t.transform.translation.y = 2.0
      t.transform.translation.z = 0.0
      t.transform.rotation.x = 0.0
      t.transform.rotation.y = 0.0
      t.transform.rotation.z = 0.0
      t.transform.rotation.w = 1.0

      self.br.sendTransform(t)


def main():
   rclpy.init()
   node = FixedFrameBroadcaster()
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()
```

这一段代码和写一个broadcaser非常类似，唯一不同的是，这里的transform是固定的，不要忘记去完善setup.py

注意，from的frame都是在父节点，就是tf树上面的点，to的结点都是子节点。定义的变换都是相对于父节点，比如这里，子节点相对于父节点在y轴上偏差2m。

## launch文件

```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
   demo_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
      )

   return LaunchDescription([
      demo_nodes,
      Node(
            package='learning_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
      ),
   ])
```

如果需要改变，原来文件中的一个参数，我们可以通过如下设置

```
def generate_launch_description():
   demo_nodes = IncludeLaunchDescription(
      ...,
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )
```

这样，可以让turtle2跟着carrot1跑

