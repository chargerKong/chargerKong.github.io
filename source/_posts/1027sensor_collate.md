---
title: sensor_collate-cartographer
date: 2021-10-27 09:59:22
tags: cartographer
---

# sensor_collator的创建

map_builder.cc文件：MapBuilder的构造函数

```c++
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) { 
      // param: num_background_threads
  ...
  ...
  // 在 cartographer/configuration_files/map_builder.lua 中设置
  // param: MAP_BUILDER.collate_by_trajectory 默认为false
  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    // sensor_collator_初始化, 实际使用这个
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}
```

在AddTrajectoryBuilder函数中，CollatedTrajectoryBuilder初始化，trajectory_builders_ 中的元素是指向CollatedTrajectoryBuilder的

```c++
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
    
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, 
        sensor_collator_.get(), 
        trajectory_id,
        expected_sensor_ids,
        // 将2D前端与2D位姿图打包在一起, 传入CollatedTrajectoryBuilder
        CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  }
```

# c++11

`absl::flat_hash_set<T>` 是一个无序关联容器，在大多数常见用例中已经针对速度和内存占用进行了优化。 它的接口类似于`std::unordered_set<T>`，但有以下显着区别：

- 要求键为需要可复制构造的键
- 支持异构查找，通过 `find()`、`operator[]()` 和 `insert()`，前提是该集合提供了兼容的异构哈希函数和相等运算符。
- 在“rehash()”之后使表中元素的任何引用和指针无效。
- 包含一个`capacity()`成员函数，指示散列集中元素槽（打开、删除和空）的数量。 从 `erase(iterator)` 重载返回 `void`。

```c++
  absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_;
```

key为int，value为一个vector

# sensor_collator类：Collator

Collator继承于CollatorInterface，定义了一个Callback函数，和如下接口

```c++
class CollatorInterface {
 public:
  using Callback =
      std::function<void(const std::string&, std::unique_ptr<Data>)>;

  CollatorInterface() {}
  virtual ~CollatorInterface() {}
  CollatorInterface(const CollatorInterface&) = delete;
  CollatorInterface& operator=(const CollatorInterface&) = delete;

  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  virtual void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      const Callback& callback) = 0;

  // Marks 'trajectory_id' as finished.
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
  // in time order.
  virtual void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) = 0;

  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  virtual void Flush() = 0;

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before CollatorInterface is
  // unblocked. Returns 'nullopt' for implementations that do not wait for a
  // particular trajectory.
  virtual absl::optional<int> GetBlockingTrajectoryId() const = 0;
};
```

Collator类在除了继承的重写之外，还额外添加了一个`OrderedMultiQueue queue_`和`absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_`



# AddTrajectory

## 调用位置

在文件，`cartographer/mapping/internal/collated_trajectory_builder.cc`的类CollatedTrajectoryBuilder构造函数中

```c++
/**
 * @brief Construct a new Collated Trajectory Builder:: Collated Trajectory Builder object
 * 
 * @param[in] trajectory_options 轨迹的参数配置
 * @param[in] sensor_collator 传入的整理传感器的类,有2种类型
 * @param[in] trajectory_id 新生成的轨迹的id
 * @param[in] expected_sensor_ids 所有需要的topic的名字的集合
 * @param[in] wrapped_trajectory_builder 完整的slam GlobalTrajectoryBuilder
 */
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      
  // sensor::Collator的初始化
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });

}
```

传入的CallBack为一个lambda函数，lambda函数即为一个可调用容器，

## 定义

循环遍历topic名字，通过和轨迹ID组合称为一个queue_key，并且加入queue_. 类型为OrderedMultiQueue，以后再说。

最后把queue_key添加到队列

```c++
absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_;
```

主要把每一个topic_id 绑定到一个回调函数上，并且保存key, topic_id与轨迹ID的组合

```c++
/**
 * @brief 添加轨迹以生成排序的传感器输出, 每个topic设置一个回调函数
 * 
 * @param[in] trajectory_id 新生成的轨迹的id
 * @param[in] expected_sensor_ids 需要排序的topic名字的集合
 * @param[in] callback 2个参数的回调函数, 实际是CollatedTrajectoryBuilder::HandleCollatedSensorData()函数
 */
void Collator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    const Callback& callback) {
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    queue_.AddQueue(queue_key,
                    // void(std::unique_ptr<Data> data) 带了个默认参数sensor_id
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}
```



# AddSensorData

## 调用位置

在 `cartographer/mapping/collated_trajectory_builder.cc`文件中，

类CollatedTrajectoryBuilder::AddData中调用

```c++
void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}
```

## 定义

即把数据添加到队列中

```c++
// 向数据队列中添加 传感器数据 
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}
```

# Flush

清空操作

```c++
// 将所有数据队列标记为已完成,分派所有剩下的传感器数据
// 只能调用一次, 在 Flush 之后不能再调用 AddSensorData()
void Collator::Flush() { queue_.Flush(); }
```

# 总结

此类就近似于一个queue_ 的封装，所有的函数都是直接操作 queue_ , 即

```
OrderedMultiQueue queue_;
```

