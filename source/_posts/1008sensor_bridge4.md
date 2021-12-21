---
title: HandleLaserScan-cartographer
date: 2021-10-08 09:59:22
tags: cartographer
---

上一次说道雷达如何转换为点云数据，无论是LaserScan还是多回声波雷达数据，都通过ToPointCloudWithIntensities函数进行转换，本节需要通过HandleLaserScan把点云数据进行处理。

激光雷达首先从订阅的回调函数做过滤，然后到达下面的处理。转换为点云的部分在上一节已经介绍

```c++
// 处理LaserScan数据, 先转成点云,再传入trajectory_builder_
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 处理MultiEchoLaserScan数据, 先转成点云,再传入trajectory_builder_
void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIframe_idntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}
```

HandleLaserScan的形参分别为

- 传感器名字
- 点云最后一个点的时间戳
- 雷达传感器的frame_id
- 点云数据

下面介绍HandleLaserScan是如何处理点云数据的

# HandleLaserScan

HandleLaserScan 函数主要根据参数配置，将一帧点云数据分成几段来处理，并且更新没一段点云的时间，保证每一段点云的最后一个点的时间为0。

最后将点云的坐标系从雷达坐标系转换到tracking frame中

函数完整的代码如下

```c++
// 根据参数配置,将一帧雷达数据分成几段, 再传入trajectory_builder_
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  // CHECK_LE: 小于等于
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.

  // 意为一帧雷达数据被分成几次处理, 一般将这个参数设置为1
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    
    // 生成分段的点云
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        // 上一段点云的时间不应该大于等于这一段点云的时间
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    // 更新对应sensor_id的时间戳
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    
    // 检查点云的时间
    for (auto& point : subdivision) {
      point.time -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back().time, 0.f);
    // 将分段后的点云 subdivision 传入 trajectory_builder_
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  } // for 
}
```

## 代码解析

如果点云数据为空，直接返回

```c++
  if (points.points.empty()) {
    return;
  }
```

检查点云最后一个点的时间是否是小于等于0，在之前我们就设置了最后一个点应该为0

```c++
  // CHECK_LE: 小于等于
  CHECK_LE(points.points.back().time, 0.f);
```

### 点云分段

假设一个点云一共有100个点，

若num_subdivisions_per_laser_scan_=1，则start_index=0, end_inndex=100

若num_subdivisions_per_laser_scan_=2，则start_index=0, end_inndex=50以及start_index=50, end_inndex=100

```c++
  // 意为一帧雷达数据被分成几次处理, 一般将这个参数设置为1
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
```

反正一共就是这么100个点，就看要被 分为几批数据来处理

分好批次后，即生成相对应的点云

```c++
carto::sensor::TimedPointCloud subdivision(
points.points.begin() + start_index, points.points.begin() + end_index);
if (start_index == end_index) {
continue;
}
```

### 时间处理

下面是分段点云的处理，

```c++
const double time_to_subdivision_end = subdivision.back().time;
```

由于只有最后一个点的时间为0，因此`time_to_subdivision_end<=0`.

time为点云的最后一个点的时间戳，subdivision_time即为此分段点云最后一个点的时间戳

```
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
```

寻找在此之前是否有过数据，如果有则判定时间，上一段点云的时间不应该大于等于这一段点云的时间

如果之前没有sensor_id，则`it==sensor_to_previous_subdivision_time_.end()`，判断完了之后更新最新的对应的sensor_id的时间

```c++
auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
if (it != sensor_to_previous_subdivision_time_.end() &&
    // 上一段点云的时间不应该大于等于这一段点云的时间
    it->second >= subdivision_time) {
    LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
        << sensor_id << " because previous subdivision time "
        << it->second << " is not before current subdivision time "
        << subdivision_time;
    continue;
}
// 更新对应sensor_id的时间戳
sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
```

更新此分段点云中每一个点的时间，同样的需要做到此点云中最后一个点的时间为0

```c++
// 更新点云中点的时间
for (auto& point : subdivision) {
	point.time -= time_to_subdivision_end;
}
```

最后送入trajectory_builder_

```c++
// 将分段后的点云 subdivision 传入 trajectory_builder_
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
```

# HandleRangefinder

   以 tracking 到 sensor_frame 的坐标变换为TimedPointCloudData 的 origin
   将点云的坐标转成 tracking 坐标系下的坐标, 再传入trajectory_builder_

```c++
void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));

  // 以 tracking 到 sensor_frame 的坐标变换为TimedPointCloudData 的 origin
  // 将点云的坐标转成 tracking 坐标系下的坐标, 再传入trajectory_builder_
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, 
                       sensor_to_tracking->translation().cast<float>(),
                       // 将点云从雷达坐标系下转到tracking_frame坐标系系下
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>())} ); // 强度始终为空
  }
}
```

## 坐标变换

点云坐标变换，遍历循环点云中的每一个点，point

通过transform * point即可

```c++
/**
 * @brief 返回坐标变换后的点云
 * 
 * @param[in] point_cloud 点云数据
 * @param[in] transform 旋转变换矩阵
 * @return TimedPointCloud 返回坐标变换后的点云
 */
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform) {
  TimedPointCloud result;
  result.reserve(point_cloud.size());
  for (const TimedRangefinderPoint& point : point_cloud) {
    result.push_back(transform * point);
  }
  return result;
}
```

最后传入trajectory_builder_的点云数据全部都是tracking frame下的

