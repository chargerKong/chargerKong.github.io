---
title: MapBuilder构造函数-cartographer
date: 2021-10-14 10:59:22
tags: cartographer
---

MapBuilder包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 的完整的SLAM，其位置位于`cartographer/mapping/map_builder.h`

本文将简要介绍一下他的构造函数，大致内容为

- 保存配置参数,
- 根据给定的参数初始化线程池,
- 初始化pose_graph_ 与sensor_collator_ 

下面为构造函数的全部代码内容

```c++
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) { 
      // param: num_background_threads
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());

  // 2d位姿图(后端)的初始化
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  // 3d位姿图(后端)的初始化
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  } 

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

# 代码解析

`^`为异或操作，只有两个数不一样才会返回true。因此use_trajectory_builder_2d和use_trajectory_builder_3d中必须要有一个为true。并且也不可以同时为true或者为false。

```
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());
```

后续根据是2d还是3d建立不同的pose_graph。

```c++
  // 2d位姿图(后端)的初始化
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  // 3d位姿图(后端)的初始化
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  } 
```

可以看到map_builder在进行构造的时候，是在node_main.cc的时候，后端就可以构造了，还没有传感器数据输入的时候，后端已经初始化好了，四个线程也已经在执行了

后续对sensor_collator进行设置, 而options.collate_by_trajectory()是定义在map_builder.lua文件中，一般不需要配置，因此他实际使用的是`absl::make_unique<sensor::Collator>()`

```c++
  // 在 cartographer/configuration_files/map_builder.lua 中设置
  // param: MAP_BUILDER.collate_by_trajectory 默认为false
  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    // sensor_collator_初始化, 实际使用这个
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
```

