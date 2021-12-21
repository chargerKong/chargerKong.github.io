---
title: HandleWorkQueue
date: 2021-12-17 18:38:29
tags: cartographer
---

在之前我们讲解了RunOptimization 函数，他添加了基于ceres的problem，并添加了五种残差进行优化，并且把优化结果保存在了相应的优化类的data中。

# Handleworkqueue 流程

- 把新计算的约束添加到data_.constraints 向量的末尾处

```c++
  {
    absl::MutexLock locker(&mutex_);
    // Step: 把新计算出的约束信息添加到data_.constraints向量的末尾处
    data_.constraints.insert(data_.constraints.end(), result.begin(),
                             result.end());
  }
```

- 执行优化

  submap的global位姿，节点的global位姿，landmark下的global 位姿，和gps的位姿都会发生变化

```c++
  // Step: 执行优化
  RunOptimization();
```

- 根据轨迹状态删除轨迹
- 进行子图的裁剪, 如果没有裁剪器就不裁剪

- 改变量到90，就进行优化，重置为num_nodes_since_last_loop_closure_ = 0
- 重新启动DrainWorkQueue

