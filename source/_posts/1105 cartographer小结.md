---
title: cartographer-数据分发小结
date: 2021-11-05 18:38:29
tags: cartographer
---

在处理数据之前，cartographer需要进行数据分发器的创建，然后数据的处理

# 数据分发器的创建

整一个数据分发器的建立就是AddTrajectory的过程，最后需要达到如下效果

针对每一个轨迹下的每一个topic, 即QueueKey, 绑定一个回调函数，方便此topic下的数据可以进行处理

```c++
/**
 * @brief 添加一个数据队列,并保存回调函数 CollatedTrajectoryBuilder::HandleCollatedSensorData
 * 
 * @param[in] queue_key 轨迹id与topic名字
 * @param[in] callback void(std::unique_ptr<Data> data) 型的函数
 * 这里的callback已经是对应sensor_id的callback了
 */
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}
```



在添加数据队列之前，即生成轨迹的过程中，还需要为每一个轨迹添加sensor_bridge, in MapBuilderBridge类的AddTrajectory函数， sensor_bridge将每一个ros数据转换为tracking frame下的数据



# 数据分发的过程



- Node类的HandleLaserScanMessage函数：过滤采样的消息，继续下传
- SensorBridge类的HandleLaserScanMessage函数：转为点云，记录最后一个点的时间
- SensorBridge::HandleLaserScan：根据配置，把一帧作为几段然后再处理
- SensorBridge::HandleRangefinder：将点云的坐标转成 tracking 坐标系下的坐标
- CollatedTrajectoryBuilder类的AddSensorData函数：调用下面这个
-  CollatedTrajectoryBuilder::AddData: 调用下面
- Collator类的AddSensorData函数：向数据队列中添加传感器数据
- OrderedMultiQueue::Add： 如果队列中没有此数据的QueueKey，则跳过，添加数据
- OrderedMultiQueue::Dispatch()： 数据分发，调用数据的回调函数
- CollatedTrajectoryBuilder::HandleCollatedSensorData:按照时间顺序分发出来的传感器数据
- GlobalTrajectoryBuilder中的AddSensorData()

```C++
void Dispatchable::AddToTrajectoryBuilder(
mapping::TrajectoryBuilderInterface *const trajectory_builder) override
{
trajectory_builder->AddSensorData(sensor_id_, data_);
}
```

trajectory_builder 是指向GlobalTrajectoryBuilder2D类的指针，开始真正的slam的前后端部分



