---
title: 添加轨迹-MapBuilder-cartographer
date: 2021-10-14 11:59:22
tags: cartographer
---

在MapBuilder类中，AddTrajectoryBuilder函数会创建一个新的 TrajectoryBuilder 并返回它的 trajectory_id。

建立轨迹的过程就需要用到前端和后端的数据

下面首先给出该函数的全部内容

```c++
/**
 * @brief 创建一个新的 TrajectoryBuilder 并返回它的 trajectory_id
 * 
 * @param[in] expected_sensor_ids 所有需要的topic的名字的集合
 * @param[in] trajectory_options 轨迹的参数配置
 * @param[in] local_slam_result_callback 需要传入的回调函数
 *        实际上是map_builder_bridge.cc 中的 OnLocalSlamResult() 函数
 * @return int 新生成的轨迹的id
 */
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {

  // id是从零开始的, 所以新trajectory_id就是trajectory_builders_的size()
  const int trajectory_id = trajectory_builders_.size();

  // 运动过滤器, 运动太小没必要进行更新
  // 配置文件中没有 pose_graph_odometry_motion_filte
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;

  // LOG(INFO) << "pose_graph odometry_motion_filter is " << trajectory_options.has_pose_graph_odometry_motion_filter();
  // 上面会打印出0, 所以没有使用后端的里程计的motion_filter

  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }

  // LocalTrajectoryBuilder 就是前端, 不带 Loop Closure 
  // 包含了 Pose Extrapolator, Scan Matching, 生成submap 等

  // 3d的轨迹
  if (options_.use_trajectory_builder_3d()) {
    // local_trajectory_builder(前端)的初始化
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    } 

    /**
     * c++11: static_cast关键字（编译时类型检查）: static_cast < type-id > ( expression )
     * 该运算符把expression转换为type-id类型, 但没有运行时类型检查来保证转换的安全性
      （1）用于基本数据类型之间的转换, 如把int转换为char, 把int转换成enum, 
      （2）把空指针转换成目标类型的空指针
      （3）把任何类型的表达式类型转换成void类型
      （4）用于类层次结构中父类和子类之间指针和引用的转换.

     * c++11: dynamic_cast关键字（运行时类型检查）: dynamic_cast < type-id > ( expression )
        该运算符把 expression 转换成 type-id 类型的对象. Type-id必须是类的指针、类的引用或者void *
        如果type-id是类指针类型, 那么expression也必须是一个指针
        如果type-id是一个引用, 那么expression也必须是一个引用

        dynamic_cast主要用于类层次间的上行转换（子类到父类）和下行转换（父类到子类）, 还可以用于类之间的交叉转换.
        在类层次间进行上行转换时, dynamic_cast和static_cast的效果是一样的；
        在进行下行转换时, dynamic_cast具有类型检查的功能, 比static_cast更安全.
     */
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));

    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        // 将3D前端与3D位姿图打包在一起, 传入CollatedTrajectoryBuilder
        CreateGlobalTrajectoryBuilder3D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph3D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  } 
  // 2d的轨迹
  else {
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      // local_trajectory_builder(前端)的初始化
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }

    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));

    // CollatedTrajectoryBuilder初始化
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        // 将2D前端与2D位姿图打包在一起, 传入CollatedTrajectoryBuilder
        CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  }

  // 是否是纯定位模式, 如果是则只保存最近3个submap
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  pose_graph_.get());

  // 如果给了初始位姿
  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    
    // 在位姿图中设置初始位姿
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }

  // 保存轨迹的配置文件
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}
```

# 代码解析

trajectory_builders_ 的声明如下

```c++
std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
```

因此他的size刚好也可以作为轨迹ID的序号

```
  // id是从零开始的, 所以新trajectory_id就是trajectory_builders_的size()
  const int trajectory_id = trajectory_builders_.size();
```

下面这一段代码并没有 什么用。。。

```c++
  // 运动过滤器, 运动太小没必要进行更新
  // 配置文件中没有 pose_graph_odometry_motion_filte
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;

  // LOG(INFO) << "pose_graph odometry_motion_filter is " << trajectory_options.has_pose_graph_odometry_motion_filter();
  // 上面会打印出0, 所以没有使用后端的里程计的motion_filter

  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }
```

  LocalTrajectoryBuilder 为前端, 不带 Loop Closure ，包含了 Pose Extrapolator, Scan Matching, 生成submap 等

## 前端指针建立

下面根据使用的是3d还是2d，对前端进行初始化，建立一个指向LocalTrajectoryBuilder3D或者LocalTrajectoryBuilder2D的指针，

```c++
// 3d的轨迹
  if (options_.use_trajectory_builder_3d()) {
    // local_trajectory_builder(前端)的初始化
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    } 
```

local_trajectory_builder为一个指向LocalTrajectoryBuilder3D的智能指针，传入的参数是3d轨迹建立的参数，以及sensor type为RANGE的所有topic的集合

在运行时检查pose_graph的类型. get 方法可以获取unique指针的原指针

```
 DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));
```

将3D前端与3D位姿图打包在一起, 传入CollatedTrajectoryBuilder

```c++
trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        // 将3D前端与3D位姿图打包在一起, 传入CollatedTrajectoryBuilder
        CreateGlobalTrajectoryBuilder3D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph3D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)))
```

2D部分和3D部分是完全一模一样的

检查是否是纯定位模式，如果是则只保存最近3个submap。可以通过trajectory_builder.lua的pure_localization_trimmer参数 进行设置

```c++
MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  pose_graph_.get());
```

## 设定初始位姿保存轨迹

最后设定初始位姿

```c++
  // 如果给了初始位姿
  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    
    // 在位姿图中设置初始位姿
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }
```

保存轨迹的配置文件

```
proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
```

