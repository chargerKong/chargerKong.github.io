---
title: global_trajectory_builder-cartographer
date: 2021-11-05 18:38:29
tags: cartographer
---

处理排好序的数据，为什么是排好序？在数据分发dispatch的时候，通过寻找it->second.queue.Peek<Data>();来找到时间最老的一个数据，因此是排好序的。

# global_trajectory_builder.h文件

在global_trajectory_builder.h中，只是声明了三个函数

- std::unique_ptr\<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D
- std::unique_ptr\<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D
- void GlobalTrajectoryBuilderRegisterMetrics

## 定义

于global_trajectory_builder.cc文件中，直接返回一个GlobalTrajectoryBuilder2D/3D。直接把参数全部用上

```c++
// 2d的完整的slam
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}
```

## 调用

调用的地方位于 map_builder.cc 的MapBuilder::AddTrajectoryBuilder

```c++
CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
```

### 参数说明

- local_trajectory_builder：前端的轨迹

  ```c++
  local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
            trajectory_options.trajectory_builder_2d_options(),
            SelectRangeSensorIds(expected_sensor_ids));
  ```

- 轨迹ID

- pose_graph_ ：SLAM后端，在node的构造函数中，会有map_builder的构造函数，给初始化了

- LocalSlamResultCallback：实际是一个lambda 函数， 返回的是OnLocalSlamResult

  ```c++
  [this](const int trajectory_id, 
               const ::cartographer::common::Time time,
               const Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const ::cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
          // 保存local slam 的结果数据 5个参数实际只用了4个
          OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
        });
  ```

- pose_graph_odometry_motion_filter：这个参数是没有被初始化的，没有使用到



# global_trajectory_builder.cc文件

类GlobalTrajectoryBuilder直接定义在了类中，这个类是一个完整的slam, 连接起了前端与后端。下面是构造函数，对前端，轨迹ID，后端，前端的回调函数，以及里程计的滤波器分别进行赋值

```c++
template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
  // 'TimedPointCloudData' may be added in that case.

  /**
   * @brief 完整的slam, 连接起了前端与后端
   * 
   * @param[in] local_trajectory_builder 2d or 3d local slam 前端
   * @param[in] trajectory_id 轨迹id
   * @param[in] pose_graph 2d or 3d pose_graph 后端
   * @param[in] local_slam_result_callback 前端的回调函数
   * @param[in] pose_graph_odometry_motion_filter 里程计的滤波器
   */
  GlobalTrajectoryBuilder(
      std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
      const int trajectory_id, PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback,
      const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback),
        pose_graph_odometry_motion_filter_(pose_graph_odometry_motion_filter) {}
  ~GlobalTrajectoryBuilder() override {}
```

他的成员函数就是添加数据，除了点云的处理之外，其他的都是直接加入前端或者后端

比如

## IMU

```c++
  // imu数据的处理, 数据走向有两个,一个是进入前端local_trajectory_builder_,一个是进入后端pose_graph_
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddImuData(imu_data);
    }
    pose_graph_->AddImuData(trajectory_id_, imu_data);
  }
```

## 里程计

```c++
  // 加入到后端之前, 先做一个距离的计算, 只有时间,移动距离,角度 变换量大于阈值才加入到后端中
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddOdometryData(odometry_data);
    }
    // TODO(MichaelGrupp): Instead of having an optional filter on this level,
    // odometry could be marginalized between nodes in the pose graph.
    // Related issue: cartographer-project/cartographer/#1768
    if (pose_graph_odometry_motion_filter_.has_value() &&
        pose_graph_odometry_motion_filter_.value().IsSimilar(
            odometry_data.time, odometry_data.pose)) {
      return;
    }
    pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
  }
```

## GPS

```c++
  // gps数据只在后端中使用
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    if (fixed_frame_pose.pose.has_value()) {
      CHECK(fixed_frame_pose.pose.value().IsValid())
          << fixed_frame_pose.pose.value();
    }
    pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }
```

## LandMark

```c++
  // Landmark的数据只在后端中使用
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
  }
```

## local slam

将local slam的结果加入到后端中, 作为位姿图的一个节点

```c++
  // 将local slam的结果加入到后端中, 作为位姿图的一个节点
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                         "local_trajectory_builder_ present.";
    local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
  }
```

## 点云数据添加

添加数据即进行扫描匹配，匹配成功就把结果作为结点加入位姿图，这玩意儿比较麻烦，再说

```c++
/**
   * @brief 点云数据的处理, 先进行扫描匹配, 然后将扫描匹配的结果当做节点插入到后端的位姿图中
   * 
   * @param[in] sensor_id topic名字
   * @param[in] timed_point_cloud_data 点云数据
   */
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";

```
  进行扫描匹配,调用addRangeData这个函数， 返回匹配后的结果
```
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            sensor_id, timed_point_cloud_data);
```


```c++
  // 扫描匹配的result
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local; // 经过扫描匹配之后位姿校准之后的雷达数据
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
  };
```

