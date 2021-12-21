---
title: 后端线程池-num_background_threads-cartographer
date: 2021-10-14 09:59:22
tags: cartographer
---

# 变量传递过程

MapBuilder的构造函数中，对线程池进行了构造，其定义在`cartographer/cartographer/mapping/map_builder.h`的类MapBuilder下

```c++
common::ThreadPool thread_pool_;
```

它的赋值在map_builder的构造函数里，使用options.num_background_threads()对thread_pool_进行赋值

```c++
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
    ...
    }
```

options.num_background_threads()参数在`cartographer/configuration_files/map_build.lua`中定义。在这里，线程池的数量为4。

```lua
include "pose_graph.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}
```

# ThreadPool构造

ThreadPool位于`cartographer/common/thread_pool.h`文件中，其构造函数为

```c++
// 根据传入的数字, 进行线程池的构造, DoWork()函数开始了一个始终执行的for循环
ThreadPool::ThreadPool(int num_threads) {
  CHECK_GT(num_threads, 0) << "ThreadPool requires a positive num_threads!";
  absl::MutexLock locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}
```

简单来说，定义了几个线程数，for循环就是运行几次，每运行一次，就在pool_中添加一个线程

每一个DoWork都有一个死循环，直到执行任务队列为空，并且running_为false的时候才会结束

```c++
// 开始一个不停止的for循环, 如果任务队列不为空, 就执行第一个task
void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif

  const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return !task_queue_.empty() || !running_;
  };

  // 始终执行, 直到running_为false时停止执行
  for (;;) {
    std::shared_ptr<Task> task;
    {
      absl::MutexLock locker(&mutex_);
      mutex_.Await(absl::Condition(&predicate));

      // map_builder.lua中设置的线程数, 4个线程处理同一个task_queue_
      // 如果任务队列不为空, 那就取出第一个task
      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      } else if (!running_) {
        return;
      }
    }
    CHECK(task);
    CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);

    // 执行task
    Execute(task.get());
  }
}
```

到此现有一个印象，后续讲解后端的时候继续
