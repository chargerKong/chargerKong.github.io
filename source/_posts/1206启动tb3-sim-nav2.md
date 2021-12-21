---
title: 启动tb3_simulation.launch.py-navigation
date: 2021-12-05 18:38:29
tags: navigation2
---

启动tb3_simulation 的时候，gazebo会卡顿，原因是找不到world，设置model的目录即可

```
export GAZEBO_MODEL_PATH=/opt/ros/foxy/share/turtlebot3_gazebo/models/
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py
```

