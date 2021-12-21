---
title: 定时器和topic名字check-Node类-cartographer
date: 2021-09-03 15:59:22
tags: cartographer

---

上回说道添加轨迹函数`Node::AddTrajectory`中的订阅话题与注册回调函数，在此之前计算了一个由topicname和sensor类型组合成的一个集合expected_sensor_ids，轨迹IDtrajectory_id，新添加了一个位姿估计器，一个传感器采样器以及如何订阅topic和注册回调函数。下面我们介绍node.cc>Node::AddTrajectory中的定时器以及topic检查

```c++
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTopicMismatchCheckDelaySec), // kTopicMismatchCheckDelaySec = 3s
      &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
```

## wall_timers_定义

```c++
  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
```

这是一个WallTimer的vector。



## 定时器

上面这个定时器

```c++
node_handle_.createWallTimer(
      ::ros::WallDuration(kTopicMismatchCheckDelaySec), // kTopicMismatchCheckDelaySec = 3s
      &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true)
```

每kTopicMismatchCheckDelaySec个时间运行Node::MaybeWarnAboutTopicMismatch函数一次。`oneshot`表示只执行一次。

### MaybeWarnAboutTopicMismatch

此函数用于检查topic的订阅名字是否正确

```c++
void Node::MaybeWarnAboutTopicMismatch(
    const ::ros::WallTimerEvent& unused_timer_event) {

  // note: 使用ros的master的api进行topic名字的获取
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);

  std::set<std::string> published_topics;
  std::stringstream published_topics_string;

  // 获取ros中的实际topic的全局名称,resolveName()是获取全局名称
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }

  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {

      // 获取实际订阅的topic名字
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);

      // 如果设置的topic名字,在ros中不存在,则报错
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
  // 告诉使用者哪些topic可用
  if (print_topics) {
    LOG(WARNING) << "Currently available topics are: "
                 << published_topics_string.str();
  }
}
```

#### 获取master里所有topic

```c++
  // note: 使用ros的master的api进行topic名字的获取
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);
```

#### 获取ros中实际topic的全局名称并放入

```c++
  // 获取ros中的实际topic的全局名称,resolveName()是获取全局名称
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }
```

#### 获取所有订阅者的topic

```c++
for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
    	...
    }
}
```

subscribers_是一个std::unordered_map，键为轨迹ID，即 entry.first;，值为一个订阅者的vector， entry.second

#### 匹配订阅的topic和实际在ros中已经有的topic

```c++
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {

      // 获取实际订阅的topic名字
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);

      // 如果设置的topic名字,在ros中不存在,则报错
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
```

通过循环订阅者，查看订阅者所订阅的topic是不是均在ros的现有topic中，如果不在则报错



