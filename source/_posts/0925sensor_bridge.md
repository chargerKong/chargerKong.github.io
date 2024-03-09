---
title: sensor-brifge-cartographer
date: 2021-09-25 15:59:23
tags: cartographer
---

SensorBridge类将ROS的消息转换为在tracking frame上的传感器数据，为了方便建图

# 构造函数

首先看一下构造函数

```c++
class SensorBridge {
 public:
  explicit SensorBridge(
      int num_subdivisions_per_laser_scan, const std::string& tracking_frame,
      double lookup_transform_timeout_sec, tf2_ros::Buffer* tf_buffer,
      ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);
    ...
    ...
  const int num_subdivisions_per_laser_scan_;
  std::map<std::string, cartographer::common::Time>
      sensor_to_previous_subdivision_time_;
  const TfBridge tf_bridge_;
  ::cartographer::mapping::TrajectoryBuilderInterface* const
      trajectory_builder_;

  absl::optional<::cartographer::transform::Rigid3d> ecef_to_local_frame_;
};

}  // namespace cartographer_ros
```

成员变量有

- const int num_subdivisions_per_laser_scan_;
- std::map<std::string, cartographer::common::Time> sensor_to_previous_subdivision_time_;
- const TfBridge tf_bridge_;
- ::cartographer::mapping::TrajectoryBuilderInterface* const trajectory_builder_;
- absl::optional\<::cartographer::transform::Rigid3d\> ecef_to_local_frame_

对应的实现为

```c++
/**
 * @brief 构造函数, 并且初始化TfBridge
 * 
 * @param[in] num_subdivisions_per_laser_scan 一帧数据分成几次发送
 * @param[in] tracking_frame 数据都转换到tracking_frame
 * @param[in] lookup_transform_timeout_sec 查找tf的超时时间
 * @param[in] tf_buffer tf_buffer
 * @param[in] trajectory_builder 轨迹构建器
 */
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}
```

在构造函数中，主要对两个成员变量，进行了赋值

## 构造函数使用实例

在我们新添一条轨迹的时候，需要为每一条轨迹添加一个对应的sensor_bridge，回顾一下添加的方式

```c++
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.tracking_frame,
      node_options_.lookup_transform_timeout_sec, 
      tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id));
```

# ToOdometryData
将ros格式的里程计数据 转成tracking frame的pose, 再转成carto的里程计数据类型
```c++
// 将ros格式的里程计数据 转成tracking frame的pose, 再转成carto的里程计数据类型
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  // 找到 tracking坐标系 到 里程计的child_frame_id 的坐标变换, 所以下方要对sensor_to_tracking取逆
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }

  // 将里程计的footprint的pose转成tracking_frame的pose, 再转成carto的里程计数据类型
  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}
```

先看一下返回的数据类型是carto::sensor::OdometryData

```c++
namespace cartographer {
namespace sensor {

struct OdometryData {
  common::Time time;
  transform::Rigid3d pose;
};

}}
```

Rigid3D是由一个旋转和平移组成一个类。

## 寻找变换

note: LookupToTracking 查找从tracking_frame_到frame_id的坐标变换

```c++
// 找到 tracking坐标系 到 里程计的child_frame_id 的坐标变换, 所以下方要对sensor_to_tracking取逆
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
```

tf_bridge_在构造函数的时候就已经做了赋值，通过tf_bridge 来查找从tracking_frame到child_frame_id的变换

如果IMU为tracking frame，base_link为child_frame，并且IMU在base的上方0.1m，则返回的平移为-0.1。

## LookupToTracking

```c++
std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string& frame_id) const {
  ::ros::Duration timeout(lookup_transform_timeout_sec_);
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
  try {
    const ::ros::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
                              timeout)
            .header.stamp;
    const ::ros::Time requested_time = ToRos(time);
    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = ::ros::Duration(0.);
    }
    return absl::make_unique<::cartographer::transform::Rigid3d>(
        ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
                                           requested_time, timeout)));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}
```

上述函数主要是通过tf的lookupTransform来查找

```c++
		buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
                              timeout)
            .header.stamp;
```

表示从当前时间开始查找，超时时间为timeout的变换，如果找到了，并且时间是在要求的时间之内的，那么就再一次查找这个时间的tf，最后返回



## 坐标变换

如果找到了对应的变换，则最后需要把在当前frame下的坐姿转变到tracking frame下的坐标，比如IMU。

```c++
 // 将里程计的footprint的pose转成tracking_frame的pose, 再转成carto的里程计数据类型
  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
```

在这里注意使用inverse()，因为现在要做的变换相当于是从base_link到imu，而我们得到的是从imu到base_link的。因此再求imu的位姿的时候，需要base_link_pose * imu_to_baselink.inverse()

# GPS数据处理

在sensorBridge中，处理GPS数据的函数为HandleNavSatFixMessage。

如果雷达的数据不是STATUS_NO_FIX，则就直接返回不处理



```c++
// 将ros格式的gps数据转换成相对坐标系下的坐标,再调用trajectory_builder_的AddSensorData进行数据的处理
void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  // 如果不是固定解,就加入一个固定的空位姿
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});
    return;
  }

  // 确定ecef原点到局部坐标系的坐标变换
  if (!ecef_to_local_frame_.has_value()) {
    ecef_to_local_frame_ =
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  // 通过这个坐标变换 乘以 之后的gps数据,就相当于减去了一个固定的坐标,从而得到了gps数据间的相对坐标变换
  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::FixedFramePoseData{
                     time, absl::optional<Rigid3d>(Rigid3d::Translation(
                               ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude, msg->longitude,
                                                msg->altitude)))});
}
```



