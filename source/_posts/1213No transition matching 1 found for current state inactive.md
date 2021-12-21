---
title: No transition matching 1 found for current state inactive
date: 2021-12-13 18:38:29
tags: ros2
---

# 报错信息

启动一个ROS2的lifecycle节点，报错信息为,

No transition matching 1 found for current state inactive

与

Unable to start transition 1 from current state inactive

```
[lifecycle_talker_node-1] [WARN] [1639387231.224649917] [rcl_lifecycle]: No transition matching 1 found for current state inactive
[lifecycle_talker_node-1] [ERROR] [1639387231.224720175] []: Unable to start transition 1 from current state inactive: Transition is not registered., at /tmp/binarydeb/ros-foxy-rcl-lifecycle-1.1.11/src/rcl_lifecycle.c:350
```



看看是否状态已经被转换掉了。不要重复发出状态转换的指令

