---
title: collator类
date: 2021-10-21 09:59:22
tags: c++
---

此类的作用是整理传感器的数据，不区分2d和3d。先让数据按照时间排列，然后输入前端。只对传感器数据进行处理没有其他的操作

# 继承类TrajectoryBuilderInterface

CollatedTrajectoryBuilder 类继承于 TrajectoryBuilderInterface，此接口里面定义了两个struct，以及一些接口

```
struct InsertionResult {
    NodeId node_id;
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };
  
  struct SensorId {
      enum class SensorType {
          RANGE = 0,
          IMU,
          ODOMETRY,
          FIXED_FRAME_POSE,
          LANDMARK,
          LOCAL_SLAM_RESULT
  };
```

下面查看一下CollatedTrajectoryBuilder的头文件

# CollatedTrajectoryBuilder头文件

他有一些系列的AddSensorData函数，分别处理雷达点云数据，IMU数据，里程计数据，GPS，landmark

譬如

```c++
  // 处理雷达点云数据
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
  }
```

都是通过一个sensor::MakeDispatchable然后再添加数据AddData 

处理GPS和landmark的时候有些许不同

```c++
  // 根据参数决定gps数据是否需要排序
  // AddData与wrapped_trajectory_builder_->AddSensorData只能选一种
  // 因为AddData最终调用的就是wrapped_trajectory_builder_->AddSensorData
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override {
    if (collate_fixed_frame_) {
      AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id,
                                               fixed_frame_pose_data);
  }
```

collate_fixed_frame_ 参数是通过配置文件获取的，在trajectory_builder.lua中可以获取

wrapped_trajectory_builder_为一个global trajectory builder。后续再说 

最后还有定义了一些变量

```c++
 private:
  void AddData(std::unique_ptr<sensor::Data> data);

  void HandleCollatedSensorData(const std::string& sensor_id,
                                std::unique_ptr<sensor::Data> data);

  sensor::CollatorInterface* const sensor_collator_;
  const bool collate_landmarks_;
  const bool collate_fixed_frame_;
  const int trajectory_id_;
  std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;

  // Time at which we last logged the rates of incoming sensor data.
  std::chrono::steady_clock::time_point last_logging_time_;
  std::map<std::string, common::RateTimer<>> rate_timers_;
```

# CollatedTrajectoryBuilder.cc文件

在构造函数中，分别对以下变量进行赋值

```c++
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      
      // 以下两个参数在 configuration_files/trajectory_builder.lua 中
      // collate_landmarks 为 false, 不要将landmark数据放入到阻塞队列中
      collate_landmarks_(trajectory_options.collate_landmarks()),
      // collate_fixed_frame 为 true, 将gps数据放入阻塞队列中
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now())
      {}
```

## CollatedTrajectoryBuilder构造函数调用

该构造函数的调用在map_bulder 中的AddTrajectBuilder里面

```c++
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {

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
```

注意最后一个参数CreateGlobalTrajectoryBuilder2D，是被wrapped_trajectory_builder所接受的，

# CollatedTrajectoryBuilder构造函数内容

根据配置来决定是否把GPS和landmark的topic名字加到集合中去

集合定义为expected_sensor_id_strings，保存的是topic 名字

```
absl::flat_hash_set<std::string> expected_sensor_id_strings;
```

循环遍历所有的topic，如果collate_landmarks_ 为false或者collate_fixed_frame_ 为false就不加入集合中

```c++
for (const auto& sensor_id : expected_sensor_ids) {
    // collate_landmarks 为 false, sensor_collator_不处理LANDMARK数据
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) {
      continue;
    }
    // collate_fixed_frame 为 true, sensor_collator_处理gps数据
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) {
      continue;
    }
    expected_sensor_id_strings.insert(sensor_id.id);
  }
```

调用sensor_collator_ 的AddTrajectory 

```c++
  // sensor::Collator的初始化
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
```

在这里，把HandleCollatedSensorData相当于作为一个回调函数传递进入.Data类是一个基类，可以处理多种函数

## HandleCollatedSensorData

首先是寻找rate_timers_ 中对应的topic名字，rate_timers_ 定义为

std::map<std::string, common::RateTimer<>> rate_timers_;

如果找不到则创建一个传递进去

```c++
/**
 * @brief 处理 按照时间顺序分发出来的传感器数据
 * 
 * @param[in] sensor_id 传感器的topic的名字
 * @param[in] data 需要处理的数据(Data是个类模板,可处理多种不同数据类型的数据)
 */
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  // 找不到就新建一个
  if (it == rate_timers_.end()) {
    // map::emplace()返回一个pair
    // emplace().first表示新插入元素或者原始位置的迭代器
    // emplace().second表示插入成功,只有在key在map中不存在时才插入成功
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, 
                 std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
```

下面是对时间的一些处理

rate_timers_ 维护着一个队列，一次Pulse，他就把此数据加入到队列中，并且删除时间老于kSensorDataRatesLoggingPeriodSeconds 的数据

```c++
  // 对数据队列进行更新
  it->second.Pulse(data->GetTime());

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  // 也就是跑carto时候的消息：
  // [ INFO]: collated_trajectory_builder.cc:72] imu rate: 10.00 Hz 1.00e-01 s +/- 4.35e-05 s (pulsed at 100.44% real time)
  // [ INFO]: collated_trajectory_builder.cc:72] scan rate: 19.83 Hz 5.04e-02 s +/- 4.27e-05 s (pulsed at 99.82% real time)

  // 将排序好的数据送入 GlobalTrajectoryBuilder中的AddSensorData()函数中进行使用
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}
```

