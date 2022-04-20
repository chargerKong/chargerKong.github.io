---
title: OrderedMultiQueue(二)-cartographer
date: 2021-10-29 09:59:23
tags: cartographer
---

OrderedMultiQueue 类，位于`cartographer/sensor/internal/ordered_muti_queue.h`

维护排序后的传感器数据的多个队列, 并按合并排序的顺序进行调度，
它将等待为每个未完成的队列查看至少一个值, 然后再在所有队列中分派下一个按时间排序的值.

在OrderedMultiQueue(一)-cartographer中对简单的函数做了讲解，再次继续

# OrderedMultiQueue

## Dispatch

数据分发，即把队列中的数据放入回调函数进行处理，分发下去。

他其实是一个大的while(true)的死循环

```c++
/**
 * @brief 将处于数据队列中的数据根据时间依次传入回调函数(数据分发)
 * 
 * 3种退出情况:
 * 退出条件1 某个话题的数据队列为空同时又不是完成状态, 就退出
 * 退出条件2 多队列queues_为空, 就退出
 * 退出条件3 数据队列中数据的个数只有1个,又不是完成状态,不能确定状态, 就先退出
 */
void OrderedMultiQueue::Dispatch() {
  while (true) {
    /*
      queues_: 
        (0, scan): {      4,     }
        (0, imu):  {1,  3,   5,  }
        (0, odom): {  2,       6,}
    */
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;

    // Step: 1 遍历所有的数据队列, 找到所有数据队列的第一个数据中时间最老的一个数据
    for (auto it = queues_.begin(); it != queues_.end();) {

      // c++11: auto*(指针类型说明符), auto&(引用类型说明符), auto &&(右值引用)

      // 获取当前队列中时间最老的一个的一个数据
      const auto* data = it->second.queue.Peek<Data>();

      if (data == nullptr) {
        // 如果队列已经处于finished状态了, 就删掉这个队列
        if (it->second.finished) {
          queues_.erase(it++);
          continue;
        }
        // 退出条件1: 某个话题的数据队列为空同时又不是完成状态, 就先退出, 发布log并标记为阻塞者
        CannotMakeProgress(it->first);
        return;
      }

      // 第一次进行到这里或者data的时间比next_data的时间小(老数据)
      // 就更新next_data, 并保存当前话题的数据队列以及queue_key
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }

      // 数据的时间戳不是按顺序的, 就报错
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      
      ++it;
    } // end for

    // 退出条件2: 只有多队列queues_为空, 才可能next_data==nullptr
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    // 如果我们还没有为这个轨迹分配任何数据, 快进这个轨迹的所有队列, 直到达到一个共同的开始时间
    
    // Step: 2 获取对应轨迹id的所有数据队列中的最小共同时间戳, 作为轨迹开始的时间
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);

    // Step: 3 将 next_queue 的时间最老的一个数据传入回调函数进行处理 

    // 大多数情况, 数据时间都会超过common_start_time的
    if (next_data->GetTime() >= common_start_time) {
      // Happy case, we are beyond the 'common_start_time' already.
      // 更新分发数据的时间
      last_dispatched_time_ = next_data->GetTime();
      // 将数据传入 callback() 函数进行处理,并将这个数据从数据队列中删除
      next_queue->callback(next_queue->queue.Pop());
    } 
    // 数据时间小于common_start_time,同时数据队列数据的个数小于2,只有1个数据的情况 罕见
    else if (next_queue->queue.Size() < 2) {
      // 退出条件3: 数据队列数据的个数少,又不是完成状态, 不能确定现在到底是啥情况, 就先退出稍后再处理
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      } 
      // 处于完成状态了, 将数据传入 callback() 函数进行最后几个数据的处理
      // 更新分发数据的时间,将数据传入 callback() 进行处理,并将这个数据从数据队列中删除
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } 
    // 数据时间小于common_start_time,同时数据队列数据的个数大于等于2个
    else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.
    const Data* next_data = nullptr;

      // 只处理数据在common_start_time的前一个数据, 其他更早的数据会被丢弃掉
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        // 更新分发数据的时间,将数据传入 callback() 进行处理
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}
```

### 代码解析

首先给出一个例子

```
queues_: 
        (0, scan): {      4,     }
        (0, imu):  {1,  3,   5,  }
        (0, odom): {  2,       6,}
```

前面是一个QueueKey(轨迹ID， topic名字)

后面代表此Key的队列，imu最快，在时间1，3，5处都有数据，其实是odom，在数据2，6处有数据



const在*的左边，表示指针指向的内容不可以变化，但是地址是可以变的，例子看const_ptr.cpp

```c++
    const Data* next_data = nullptr;
	Queue* next_queue = nullptr;
    QueueKey next_queue_key;
```



for循环每一个队列，找到每一个队列中最老的数据，即peek()

如果该队列的peek数据为nullptr，且标记为finish了，说明队列已经结束，删除

```c++
    // Step: 1 遍历所有的数据队列, 找到所有数据队列的第一个数据中时间最老的一个数据
    for (auto it = queues_.begin(); it != queues_.end();) {

      // c++11: auto*(指针类型说明符), auto&(引用类型说明符), auto &&(右值引用)

      // 获取当前队列中时间最老的一个的一个数据
      const auto* data = it->second.queue.Peek<Data>();

      if (data == nullptr) {
        // 如果队列已经处于finished状态了, 就删掉这个队列
        if (it->second.finished) {
          queues_.erase(it++);
          continue;
        }
```

否则，该队列为空，但是又不是完成的状态, 就直接返回，并且标记阻塞.

表示消费者消耗 的太快了，把队列中的数据都用完了，直接返回。对本次dispatch结束，等一等数据的加入

```c++
		// 退出条件1: 某个话题的数据队列为空同时又不是完成状态, 就先退出, 发布log并标记为阻塞者
        CannotMakeProgress(it->first);
        return;
      }
```

更新数据，第一次到达这里，记录此数据为next_data=data，即该队列中最早的那个数据, 并且记录对应的队列next_queue和QueueKey。

若是之后的队列最早的数据的时间，比next_data的更早，则next_data进行替换。

总之找出所有队列中，时间最早的队列和QueueKey

- next_data：所有队列中，时间最早的数据

```c++
      // 第一次进行到这里或者data的时间比next_data的时间小(老数据)
      // 就更新next_data, 并保存当前话题的数据队列以及queue_key
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
```

time_point(duration::min()); 见time_test.cpp

判断找出来的最老的数据是不是比上次数据分发的时间要晚

```c++
 // 数据的时间戳不是按顺序的, 就报错
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      
      ++it;
} //end for
```

如果所有队列中最早的数据为空的话，说明所有数据全部分发完毕，结束

```c++
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }
```

// 如果我们还没有为这个轨迹分配任何数据, 快进这个轨迹的所有队列, 直到达到一个共同的开始时间    

// Step: 2 获取对应轨迹id的所有数据队列中的最小共同时间戳, 作为轨迹开始的时间

```c++
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id)
```

// Step: 3 将 next_queue 的时间最老的一个数据传入回调函数进行处理，开始分发数据

- next_data 的时间大于共同开始时间的：

  - 更新last_dispatched_time_ ，发放数据（调用callback）

    ```c++
     if (next_data->GetTime() >= common_start_time) {
          // Happy case, we are beyond the 'common_start_time' already.
          // 更新分发数据的时间
          last_dispatched_time_ = next_data->GetTime();
          // 将数据传入 callback() 函数进行处理,并将这个数据从数据队列中删除
          next_queue->callback(next_queue->queue.Pop());
        } 
    ```

- 时间小于共同时间的，并且当前队列的数据少于2个的

  - 如果队列不是完成状态，调用CannotMakeProgress
  - 否则直接处理，更新last_dispatched_time_ ，更新last_dispatched_time_ ，发放数据（调用callback）

- 时间小于共同时间的，并且当前队列还长着，

  - 只处理比common_start_time 早一帧的数据，其他的数据不要

    ```c++
    std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
    if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        // 更新分发数据的时间,将数据传入 callback() 进行处理
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
    }
    ```

    next_data_owner 就是这一帧的数据，Pop出来之后，直接看栈顶的数据，有没有大于common_start_time。如果大于的画，则处理Pop出来的数据





## CannotMakeProgress

1. 做好此为阻塞队列的标记
2. 发布log，若有其他队列长度大于kMaxQueueSize的topic数据，则发布log，等待这个队列为0的消息

```c++
// 标记queue_key为阻塞者,并按条件发布log,等等这个数据
void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  // 标记queue_key为阻塞者
  blocker_ = queue_key;
  for (auto& entry : queues_) {
    // queue_key对应的数据队列为空,而某一个传感器数据队列的数据已经大于kMaxQueueSize了
    // 有问题, 进行报错
    if (entry.second.queue.Size() > kMaxQueueSize) {
      // 在该语句第1、61、121……次被执行的时候, 记录日志信息
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;

      // [ WARN] [1628516438.493835120, 1606808659.273453929]: W0809 21:40:38.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      // [ WARN] [1628516439.089736487, 1606808659.869309184]: W0809 21:40:39.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      return;
    }
  }
}
```

此错误，一个队列为0，另一个又太多

一般情况下，**就是topic名字没有设置正确才有会有这样的情况**



## GetCommonStartTime

找到此轨迹的所有数据队列所有第一帧的最大时间(共同时间)

```c++
/**
 * @brief 找到数据队列所有第一帧的最大时间(共同时间)
 * 对于某个id的轨迹的 common_start_time 只会计算一次
 * 
 * @param[in] trajectory_id 轨迹id
 * @return common::Time 返回数据队列所有第一帧的最大时间
 */
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {

  // c++11: map::emplace() 返回的 pair 对象
  // pair 的成员变量 first 是一个指向插入元素或阻止插入的元素的迭代器
  // 成员变量 second 是个布尔值, 表示是否插入成功, 如果这个元素的索引已经存在插入会失败,返回false
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  common::Time& common_start_time = emplace_result.first->second;

  // 如果插入成功了就找到时间戳最大的对common_start_time进行更新, 失败了就不更新
  // 只会在轨迹开始时插入成功一次
  if (emplace_result.second) {
    // 找到这个轨迹下,所有数据队列中数据的时间戳最大 的时间戳
    // 执行到这里时, 所有的数据队列都有值了, 因为没值的情况在Dispatch()中提前返回了
    for (auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";

    // [ INFO] [1628516134.243770381, 1606808649.533687125]: I0809 21:35:34.000000  8604 ordered_multi_queue.cc:264] All sensor data for trajectory 0 is available starting at '637424054495384530'.

  }

  return common_start_time;
}
```

### 代码解析

首先对common_start_time_per_trajectory_ 插入一个此轨迹的最小时间

只有第一次才可以插入成功，插入成功之后遍历所有的队列

找出所有此轨迹的队列，并且对比最老的数据，即时间最早的数据（Peek）,

找出时间最大的第一帧数据
