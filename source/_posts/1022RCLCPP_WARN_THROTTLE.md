---
title: RCLCPP_WARN_THROTTLE 说明
date: 2021-10-22 09:59:22
tags: ros2
---

[RCLCPP_WARN_THROTTLE](https://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html#a451bee77c253ec72f4984bb577ff818a)的官方说明在此，即

```
#define RCLCPP_WARN_THROTTLE	(logger,
 	clock,
 	duration,
 	... 
)		
```

此声明是用于控制告警信息的一个时间间隔，duration的时间为毫秒，如果在duration时间之内已经发送过一次警告了，则不再执行本次警告。clock则记录本次打出警告的时间

# 使用示例

```c++
rclcpp::Clock steady_clock(RCL_STEADY_TIME);
RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock, 1000, "RCLCPP_WARN_THROTTLE was called");
```

这句话的意思就表示，打印warn的时间间隔不得小于1000ms。

Clock中可以传入多种时间，分别为系统时间`RCL_SYSTEM_TIME`，ROS时间`RCL_ROS_TIME`，和稳定的时间`RCL_STEADY_TIME`。有什么区别暂时不知道，打印出来是一样的。。。

# 输出结果示例

我把此warn写入一个topic callback，每100ms接受到一个message，但是warn要1000ms一次

```
[WARN] [1634871226.715184170] [minimal_subscriber]: RCLCPP_WARN_THROTTLE was called
[INFO] [1634871226.814115514] [minimal_subscriber]: I heard: 'Hello, world! 48'
[INFO] [1634871226.913323879] [minimal_subscriber]: I heard: 'Hello, world! 49'
[INFO] [1634871227.012755620] [minimal_subscriber]: I heard: 'Hello, world! 50'
[INFO] [1634871227.111737842] [minimal_subscriber]: I heard: 'Hello, world! 51'
[INFO] [1634871227.210964278] [minimal_subscriber]: I heard: 'Hello, world! 52'
[INFO] [1634871227.310321142] [minimal_subscriber]: I heard: 'Hello, world! 53'
[INFO] [1634871227.409564982] [minimal_subscriber]: I heard: 'Hello, world! 54'
[INFO] [1634871227.508874625] [minimal_subscriber]: I heard: 'Hello, world! 55'
[INFO] [1634871227.608132403] [minimal_subscriber]: I heard: 'Hello, world! 56'
[INFO] [1634871227.707389626] [minimal_subscriber]: I heard: 'Hello, world! 57'
[INFO] [1634871227.806639776] [minimal_subscriber]: I heard: 'Hello, world! 58'
[WARN] [1634871227.806787053] [minimal_subscriber]: RCLCPP_WARN_THROTTLE was called
```

# 注意

dashing 版本还没有此功能，可以使用其他的进行替换