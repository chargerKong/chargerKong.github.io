---
title: tf2 and time
date: 2022-01-30 15:59:22
tags: tf2
---

# 背景

在之前的教程中，我们通过tf2的广播者和监听者对小乌龟案例进行了研究，我们还学习了如何向一个tf树添加一个frame。该tf树会随着时间而变化，并且 tf2 会为每个转换存储一个时间快照（默认情况下最多 10 秒）。 到目前为止，我们使用 lookupTransform() 函数来访问该 tf2 树中最新的可用转换，而不知道该转换是在什么时间记录的。 本教程将教您如何在特定时间进行转换。

## 1 tf2 and time

打开之前的文件 `turtle_tf2_listener.cpp`，查看一下函数 `lookupTransform()`的使用

```
transformStamped = tf_buffer_->lookupTransform(
   toFrameRel,
   fromFrameRel,
   tf2::TimePointZero);
```

`tf2::TimePointZero`表示的是查询tf树中最近的一次变换。现在我们改变这一行为查询当前时刻的变换,`this->get_clock()->now()`:

```c++
rclcpp::Time now = this->get_clock()->now();
transformStamped = tf_buffer_->lookupTransform(
   toFrameRel,
   fromFrameRel,
   now);
```

现在我们再一次去运行

```
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
```

我们会发现如下的错误提示

```c++
[INFO] [1629873136.345688064] [listener]: Could not transform turtle1 to turtle2: Lookup would require extrapolation into the future.  Requested time 1629873136.345539 but the latest data is at time 1629873136.338804, when looking up transform from frame [turtle1] to frame [turtle2]
```

这里的提醒为，当前时刻请求的数据暂无

要了解为什么会发生这种情况，我们需要了解缓冲区的工作原理。 首先，每个listener都有一个缓冲区，该缓冲区存储来自不同 tf2 广播器的所有坐标变换。 其次，当广播者发送一个转换时，转换进入缓冲区需要一些时间（通常是几毫秒）。 因此，当您在“现在”时间请求帧变换时，您应该等待几毫秒以使该信息到达。

## 2 Wait for transforms

tf2 提供了一个工具，它可以等待一段时间，保证在等待这一段时间内不报错。这只需要在 `lookupTransform()`添加一个`timeout`参数即可，为了修复上述错误，我们把代码改为如下形式

```c++
rclcpp::Time now = this->get_clock()->now();
transformStamped = tf_buffer_->lookupTransform(
   toFrameRel,
   fromFrameRel,
   now,
   50ms);
```

 `lookupTransform()` 函数接受四个参数，最后一个是可选的参数，它将阻塞直到等待它超时的时间。

## 3 Checking the results

现在我们再一次运行

```
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
```

您应该注意到，lookupTransform() 实际上会阻塞，直到两个海龟之间的转换变得可用（这通常需要几毫秒）。 一旦超时（在这种情况下为 50 毫秒），只有在转换仍然不可用时才会引发异常。 

# 总结

在本教程中，您学习了如何在特定时间戳获取转换，以及如何在使用 lookupTransform() 函数时等待转换在 tf2 树上可用。 

