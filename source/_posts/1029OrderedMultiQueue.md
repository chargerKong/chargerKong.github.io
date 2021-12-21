---
title: OrderedMultiQueue(一)-cartographer
date: 2021-10-29 09:59:22
tags: cartographer
---

类Collator是对传感器数据的处理

他的全部操作相当于是对如下数据结构的数据进行封装

```c++
  // Queue keys are a pair of trajectory ID and sensor identifier.
  OrderedMultiQueue queue_;
```

打开OrderedMultiQueue 的头文件，即位于`cartographer/sensor/internal/ordered_muti_queue.h`

# 结构体 QueueKey

可以用于排序使用

```	c++
struct QueueKey {
  int trajectory_id;      // 轨迹id
  std::string sensor_id;  // topic名字

  // 重载小于运算符, map根据这个规则对QueueKey进行排序
  // 以tuple规则比较2者, tuple定义了<运算符, 逐个元素进行比较
  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};
```

即轨迹ID和topic名字的一个组合，用tuple重载了小于运算符



# OrderedMultiQueue 头文件

```c++

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
// 维护排序后的传感器数据的多个队列, 并按合并排序的顺序进行调度
// 它将等待为每个未完成的队列查看至少一个值, 然后再在所有队列中分派下一个按时间排序的值.

// This class is thread-compatible. 此类是线程兼容的

class OrderedMultiQueue {
 public:
  // note: OrderedMultiQueue::Callback 1个参数
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue();

  // c++11: 移动构造函数, 只在使用的时候编译器才会自动生成
  // 这里是显示指定让编译器生成一个默认的移动构造函数
  OrderedMultiQueue(OrderedMultiQueue&& queue) = default;

  ~OrderedMultiQueue();

  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  void AddQueue(const QueueKey& queue_key, Callback callback);

  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  void MarkQueueAsFinished(const QueueKey& queue_key);

  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  void Flush();

  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  QueueKey GetBlocker() const;

 private:
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;   // 存储数据的队列
    Callback callback;                                    // 本数据队列对应的回调函数
    bool finished = false;                                // 这个queue是否finished
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.
  common::Time last_dispatched_time_ = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_;
  std::map<QueueKey, Queue> queues_;   // 多个数据队列
  QueueKey blocker_;
};
```

queues_ 保存的是多个数据队列，即每一个topic的数据队列都需要保存

## AddQueue

### 调用位置

位于`cartographer/sensor/internal/collator.cc`

```c++
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

### 定义

对 对应的queueKey 添加一个callback函数， 注意queue_ 的value类型为Queue，定义为private中

Queue 包含

- 包含数据的的队列
- 本数据队列对应的回调函数
- 这个queue是否finished

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

## MarkQueueAsFinished

### c++11 it->second

it 找到话返回一个std::pair 

- it->first 表示第一个元素
- it->second 表示第二个元素

```c++
// 将queue_key对应的Queue的finished设置成true
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";

  auto& queue = it->second;
  CHECK(!queue.finished);

  queue.finished = true;
  Dispatch();
}
```

queue_ 首先从map中寻找相应的键，如果没有，则报错

若是找到了，则设置finish=true

调用dispatch！将处于数据队列中的数据根据时间依次传入回调函数(数据分发)

## Add

### 调用位置

位于`cartographer/sensor/internal/collator.cc`

```c++
// 向数据队列中添加 传感器数据 
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}
```

### 定义

```c++
// 向数据队列中添加数据
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  // 如果queue_key不在queues_中, 就忽略data
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }

  // 向数据队列中添加数据
  it->second.queue.Push(std::move(data));

  // 传感器数据的分发处理
  Dispatch();
}
```

总体看着非常简单，首先找一下此键对应的队列在不在，如果在的话直接push添加数据。

调用dispatch！将处于数据队列中的数据根据时间依次传入回调函数(数据分发)



## Flush

### 定义

此函数将所有处于未完成状态的数据队列标记为完成状态，

首先遍历所有的队列，找到还没有finished的队列，加到vector 容器中，然后循环此容器，调用MarkQueueAsFinished 结束队列。

```c++
// 将所有处于未完成状态的数据队列标记为完成状态
void OrderedMultiQueue::Flush() {
  // 找到所有unfinished的数据队列
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  // 将unfinished_queues标记为完成状态
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}
```

## getBlocker

### 定义

```
// 返回阻塞的队列的QueueKey
QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}
```



