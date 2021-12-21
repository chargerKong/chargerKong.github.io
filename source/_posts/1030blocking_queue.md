---
title: Blocking-cartographer
date: 2021-10-29 09:59:23
tags: cartographer
---

一个阻塞队列，可以看成是一个缓冲区。

生产者把数据放入队列，消费者把数据从队列里取出来进行消费

# 为什么要用生产，消费模式

- 解耦：生产者和消费者之间不能直接依赖，从队列里取，
- 并发：  生产者直接调用消费者, 两者是同步（阻塞）的, 如果消费者吞吐数据很慢, 这时候生产者白白浪费大好时光。 而使用这种模式之后, 生产者将数据丢到缓冲区, 继续生产, 完全不依赖消费者, 程序执行效率会大大提高。
- 复用：独立开两个类，如果有一个被修改了，另外一个也不会受到影响

#  类定义

```c++
template <typename T>
class BlockingQueue {
    ...
private:
  absl::Mutex mutex_;
  const size_t queue_size_ GUARDED_BY(mutex_);
  std::deque<T> deque_ GUARDED_BY(mutex_);	
}
```

这是一个类模板。他所拥有的类变量为

- mutex：一个互斥锁
- queue_size_： 表示队列大小
- deque_： 一个双端队列

## 类调用

在ordered_multi_queue.h文件中，private变量定义的结构体为

```
struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;   // 存储数据的队列
    Callback callback;                                    // 本数据队列对应的回调函数
    bool finished = false;                                // 这个queue是否finished
  };
```

其中，模板则为指向Data的一个智能指针

## 构造函数

构造函数调用了另外一个构造函数，并且赋值queue_size_ 为0

```c++
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  // 构造一个具有无限队列大小的阻塞队列
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}
  
  // Constructs a blocking queue with a size of 'queue_size'.
  // 构造一个大小为 queue_size 的阻塞队列
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}
```

## QueueNotFullCondition

判断队列是不是没有满，没有满返回true。

如果调用了 BlockingQueue() 而产生的对象，则他是一个永远都不会满的队列。因为`queue_size_ == kInfiniteQueueSize`，一直成立

```c++
// Returns true if the queue is not full.
  // 如果队列未满, 则返回true
  bool QueueNotFullCondition() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }
```

## push

mutex_.Await(absl::Condition(&predicate)); 

只有当predicate为true的时候，才会继续往下走，false会继续等待。

而这里QueueNotFullCondition会一直返回true，所以会一直加，此队列无限长

加入数据的方式则为简单的push_back即可

```c++
 // Pushes a value onto the queue. Blocks if the queue is full.
  // 将值压入队列. 如果队列已满, 则阻塞
  void Push(T t) {
    // 首先定义判断函数
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return QueueNotFullCondition();
    };

    // absl::Mutex的更多信息可看: https://www.jianshu.com/p/d2834abd6796
    // absl官网: https://abseil.io/about/

    // 如果数据满了, 就进行等待
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(&predicate));

    // 将数据加入队列, 移动而非拷贝
    deque_.push_back(std::move(t));
  }
```

因此，他就是一个双端队列，加上一个条件判断是否要堵塞就完成了这个阻塞队列，缓冲区的功能

## pop

pop函数和push函数比较类似

判断的是队列是否为空，如果是空，则等待，不能继续pop

如果不空，则先取值，然后pop ，最后return取出来的值即可

```c++
  // Pops the next value from the queue. Blocks until a value is available.
  // 取出数据, 如果数据队列为空则进行等待
  T Pop() {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return !QueueEmptyCondition();
    };
    // 等待直到数据队列中至少有一个数据
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(&predicate));

    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }
```

## peek

```c++
  // 返回第一个数据的指针, 如果队列为空则返回nullptr
  template <typename R>
  const R* Peek() {
    absl::MutexLock lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }
```

