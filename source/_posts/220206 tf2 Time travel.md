---
title: Time travel with tf2
date: 2022-02-06 12:59:22
tags: [tf2, ros2]
---

在之前的教程中，我们讨论了 tf2 和 time 的基础知识。 本教程将带我们更进一步，并揭示一个强大的 tf2 技巧：时间旅行(time travel.)。 简而言之，tf2 库的关键特性之一是它能够在时间和空间上转换数据。

此 tf2 时间旅行功能可用于各种任务，例如长时间监控机器人的位姿或构建跟随领导者“步骤”的跟随机器人。 我们将使用该时间旅行功能及时查找变换，并编程turtle2 使其落后于carrot1 5 秒。

# time travel

现在，我们不再让第二个乌龟跟着carrot跑，而是让第二个乌龟跟着五秒之前的carrot跑。打开文件`turtle_tf2_listener.cpp`，

```c++
rclcpp::Time when = this->get_clock()->now() - rclcpp::Duration(5, 0);
transformStamped = tf_buffer_->lookupTransform(
    toFrameRel,
    fromFrameRel,
    when,
    50ms);
```

如果我们启动此文件，前面五秒，第二个乌龟会不知道去哪儿，因为carrot没有前五秒的数据。但是五秒以后他的行为会跟着carrot五秒前的行为

```
ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py
```

![../../_images/turtlesim_delay1.png](https://docs.ros.org/en/foxy/_images/turtlesim_delay1.png)

你会发现，五秒之后，第二只乌龟会出现和上图一样的不受控制的行为。原因如下

1. 在我们的代码中，我们向 tf2 询问了以下问题：“carrot1 5 秒前相对于 turtle2 5 秒前的位姿是什么？”。 这意味着我们根据 5 秒前的位置以及第一个胡萝卜 5 秒前的位置来控制第二个海龟。

2. 然而，我们真正想问的是：carrot1 5 秒前的pose，相对于当前时刻的turtle2 的变换是什么？”。

# lookupTransform 的高级应用

为了向 tf2 提出这个特定问题，我们将使用一个高级 API，它使我们能够明确说明何时获取指定的转换。 这是通过调用lookupTransform() 方法来完成的。 您的代码现在看起来像这样：

```c++
rclcpp::Time now = this->get_clock()->now();
rclcpp::Time when = now - rclcpp::Duration(5, 0);
transformStamped = tf_buffer_->lookupTransform(
    toFrameRel,
    now,
    fromFrameRel,
    when,
        "world",
    50ms);
```

这个高级的API有六个参数需要指定

1. Target frame
2. 转变到target的时间
3. source frame
4. 计算source frame变换的时间
5. 一个相对不会变换的frame：这里是world
6. timeout

总的来说，tf2 做了如下事情，他计算了之前carrot1和world的坐标变换，在世界frame中，tf2 时间从过去穿越到现在。 在当前时间，tf2 计算从世界到 turtle2 的变换。



# 检查结果

再一次运行结果查看

```
ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py
```

![../../_images/turtlesim_delay2.png](https://docs.ros.org/en/foxy/_images/turtlesim_delay2.png)

在本教程中，您已经了解了 tf2 的一项高级功能。 您了解到 tf2 可以及时转换数据，并通过 turtlesim 示例学习了如何做到这一点。 tf2 允许您使用高级 lookupTransform() API 及时返回并在海龟的旧姿势和当前姿势之间进行帧转换。

