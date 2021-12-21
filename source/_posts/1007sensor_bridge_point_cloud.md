---
title: 雷达转点云-cartographer
date: 2021-10-07 09:59:22
tags: cartographer
---

在cartographer中，有两种雷达的消息格式，一个是LaserScan，另外一个是MultiEchoLaserScan，即多回声波雷达。

# 雷达消息处理流程

下面是cartographer在接收到雷达消息的之后的流程示意图



![](Screenshot from 2021-10-07 18-32-56.png)



## node类筛选数据

在Node类中，会定义订阅者Node::HandleLaserScanMessage来处理雷达消息的处理，通过配置文件里配置的采样频率，来判定此帧数据要不要处理，不要处理的直接的return

```c++
void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}
```

```c++
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  // 根据配置,是否将传感器数据跳过
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}
```



## sensor类处理

如果需要处理，则通过map_builder_bridge来获取对应轨迹ID的sensor_bridge，通过sensor_bridge来处理

```c++
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
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}
```

无论是LaserScan还是MultiEchoLaserScan，都是通过函数`ToPointCloudWithIntensities`将ros的消息转换为点云

将生成的点云，再通过`HandleLaserScan`一样处理。因此，下面我们着重看一下如何把msg转换为点云

# 点云的转换

无论是普通的雷达还是多回声波雷达，都是通过函数`ToPointCloudWithIntensities`把他们转为点云的数据，下面查看此函数的定义，在msg_conversion中，有此函数的重载

```c++
// 由ros格式的LaserScan转成carto格式的PointCloudWithIntensities
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

// 由ros格式的MultiEchoLaserScan转成carto格式的PointCloudWithIntensities
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::MultiEchoLaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}
```

因此，在调用的时候会自动识别传入参数的类别进行不同函数的加载。但是对于函数`LaserScanToPointCloudWithIntensities` 并没有进行重载，而是通过一个模板类对代码进行了缩减。

## 数据类型

在说明代码之前，我们需要对两种数据结构做一下强调

### LaserScan

```
$ rosmsg show sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```
### MultiEchoLaserScan
```
$ rosmsg show sensor_msgs/MultiEchoLaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
sensor_msgs/LaserEcho[] ranges
  float32[] echoes
sensor_msgs/LaserEcho[] intensities
  float32[] echoes  
```

这两种数据结构非常相似，除了ranges和intensitites的定义不同，一个是简单的float32的数组，一个是数组里面套一个数组。在处理的时候只需要注意这两个不同即可

### PointCloudWithIntensities

这是返回的点云数据类型

```c++
struct TimedRangefinderPoint {
  Eigen::Vector3f position;
  float time;
};

using TimedPointCloud = std::vector<TimedRangefinderPoint>;

struct PointCloudWithIntensities {
  TimedPointCloud points;
  std::vector<float> intensities;
};
```

此点云数据由两个vector组成，

- 其中一个vector的元素由位置和时间组成

- 其中一个vector的元素由强度组成

## 代码解析

下面为此函数的全部内容

```c++
// For sensor_msgs::LaserScan and sensor_msgs::MultiEchoLaserScan.
// 将LaserScan与MultiEchoLaserScan转成carto格式的点云数据
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }

  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    // c++11: 使用auto可以适应不同的数据类型
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {

      const float first_echo = GetFirstEcho(echoes);
      // 满足范围才进行使用
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const cartographer::sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()), // position
            i * msg.time_increment};                            // time
        // 保存点云坐标与时间信息
        point_cloud.points.push_back(point);
        
        // 如果存在强度信息
        if (msg.intensities.size() > 0) {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          // 使用auto可以适应不同的数据类型
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }

  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    // 以点云最后一个点的时间为点云的时间戳
    timestamp += cartographer::common::FromSeconds(duration);

    // 让点云的时间变成相对值, 最后一个点的时间为0
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}
```

函数首先对数据的合规性进行了检查

```c++
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) 转{
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
```

定义了需要返回的点云point_cloud，然后对雷达的每一点进行处理

```c++
 PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
      ...
  }
```

检查第一个点的数据是否存在

```c++
// c++11: 使用auto可以适应不同的数据类型
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
    	...
    }
```

这里通过简单的重载即可实现

- 如果是echoes是一个数值，直接返回true
- 如果是echoes是sensor_msgs::LaserEcho，则检查是他是不是空的

```c++
bool HasEcho(float) { return true; }

// For sensor_msgs::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::LaserEcho& echo) {
  return !echo.echoes.empty();
}
```

下面对返回数据的第一个属性进行填充，即points，他是一个TimedRangefinderPoint为元素的一个vector

```c++
const float first_echo = GetFirstEcho(echoes);
// 满足范围才进行使用
if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
    const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
    const cartographer::sensor::TimedRangefinderPoint point{
        rotation * (first_echo * Eigen::Vector3f::UnitX()), // position
        i * angle.time_increment};                            // time
    // 保存点云坐标与时间信息
    point_cloud.points.push_back(point);
    ...
}
```

在这里，我们把点云重新定义到了一个新的坐标系，保持各个点的相对距离和原来雷达的点一致

每一个点都以$(1,0,0)$为基准，乘以此点的距离`first_echo`，若rotation为正，则逆时针 旋转，若rotation为负，则 顺时针旋转，循环遍历每一个点，则可以获得一个点云

添加强度信息，如果没有则赋予0

```c++
if (msg.intensities.size() > 0) {
    CHECK_EQ(msg.intensities.size(), msg.ranges.size());
    // 使用auto可以适应不同的数据类型
    const auto& echo_intensities = msg.intensities[i];
    CHECK(HasEcho(echo_intensities));
    point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
} else {
    point_cloud.intensities.push_back(0.f);
}
```

最后更新一下角度，即此帧的下一个雷达点

```
angle += msg.angle_increment;
```

继续重新获取雷达点的距离，开始转换等

最后是时间的处理

```c++
::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    // 以点云最后一个点的时间为点云的时间戳
    timestamp += cartographer::common::FromSeconds(duration);

    // 让点云的时间变成相对值, 最后一个点的时间为0
    for (auto& point : point_cloud.points) {
        point.time -= duration;
    }
}
```

在往point_cloud里面添加point的时候，我们就是按照time_increment一个个增加上去的，因此取的最后一个点，也就可以得到此帧数据扫描所需要的时间，就是duration。

timestamp本来是此帧数据的第一个点发射的时间，现在加上了duration，就是最后一个点发射出去的时间，也作为此函数的返回数据

最后把此帧点云所有的点的时间减去了duration，即最后一个点为0，第一个点为`-duration`。

最后返回数据

```
return std::make_tuple(point_cloud, timestamp);
```



