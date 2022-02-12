---
title: cartographer 变量记录
date: 2021-12-23 18:38:29
tags: cartographer
---

# 通过雷达直接转过来的点云

ToPointCloudWithIntensities：

```
{xyzti_0, xyzti_1, xyzti_2, ... ,}
```



# 单雷达时间同步之前的点云

TimedPointCloudData:

```
{time, origin, {xyzt_0, xyzt_1,...,}, {i_0, i_1, ...}}}
```

这里的origin：

1. 求sensor_to_tracking的变换
2. 求此变换的平移
3. 平移的值就是laser在imu（tracking frame）下的坐标



# C++ Optional

定义

```
absl::optional<common::Time> last_sensor_time_;
```

使用

```c++
const common::Time current_sensor_time = synchronized_data.time;    
if (last_sensor_time_.has_value()) {
    sensor_duration = current_sensor_time - last_sensor_time_.value();
}
```

# sensor::RangeData

对一帧的雷达数据进行时间的同步，去畸变后，求出点在local坐标系下的坐标，存入accumulated_range_data_。

根据设置存够足够多的帧数，就进行匹配



# 位姿估计器

到目前发现有两个位姿估计器

- LocalTrajectoryBuilder2D 里面有一个 位姿估计器
- Node 里面也有一个 `std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;`

LocalTrajectoryBuilder2D中的位姿估计器，同时添加多个数据，Pose: 扫描匹配后的Pose，imu 和 odom



Imu_tracker->advance的作用，是根据时间差 * 上一时刻的角速度 来获取的旋转量，然后更新到位姿和线性加速度上。

Imu_tracker的AddImuLinearAccelerationObservation： 也是更新位姿和线性加速度用的

- 通过指数滑动平均法融合计算一下加速度，z轴为重力方向
- 加速度是包含重力方向的，所以只要imu有所转动，重力方向可以立马感应到的，他产生的旋转差，就刚好可以补充到位姿上进行更新。

# PoseGraph2D 

在这个类中，有一个变量data_，他包含这所有后端优化的数据

```
PoseGraphData data_
```

## data_.trajectory_nodes

```c++

MapById<NodeId, TrajectoryNode> trajectory_nodes;
```

这是每一个节点的ID 对应的 轨迹和节点的信息，主要包括节点在global坐标系下的位姿, 与前端的结果

## data_.submap_data

submap的信息

```c++
  // submap_data_ 里面,包含了所有的submap
  MapById<SubmapId, InternalSubmapData> submap_data;
```

```c++
// 保存的子图的指针与属于这张子图的节点的id
struct InternalSubmapData {
  std::shared_ptr<const Submap> submap;
  SubmapState state = SubmapState::kNoConstraintSearch;

  // IDs of the nodes that were inserted into this map together with
  // constraints for them. They are not to be matched again when this submap
  // becomes 'kFinished'.
  // 插入到此地图中的节点的 ID
  // 当此子图变为“kFinished”后, 这些节点将不再与这个地图进行匹配.
  std::set<NodeId> node_ids;
};
```

