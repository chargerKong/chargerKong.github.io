---
title: 后端调用流程-cartographer
date: 2021-12-01 18:38:29
tags: cartographer
---

# PoseGraph2D:AddNode

调用之处：global_trajectory_builder.cc 的AddSensorData函数中.

显示进行扫描匹配，匹配成功后把匹配后的结果加入到结点中

```c++
  /**
   * @brief 点云数据的处理, 先进行扫描匹配, 然后将扫描匹配的结果当做节点插入到后端的位姿图中
   * 
   * @param[in] sensor_id topic名字
   * @param[in] timed_point_cloud_data 点云数据
   */
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";

    // 进行扫描匹配, 返回匹配后的结果
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            sensor_id, timed_point_cloud_data);

    if (matching_result == nullptr) {
      // The range data has not been fully accumulated yet.
      return;
    }
    kLocalSlamMatchingResults->Increment();
    std::unique_ptr<InsertionResult> insertion_result;

    // matching_result->insertion_result 的类型是 LocalTrajectoryBuilder2D::InsertionResult
    // 如果雷达成功插入到地图中
    if (matching_result->insertion_result != nullptr) {
      kLocalSlamInsertionResults->Increment();

      // 将匹配后的结果 当做节点 加入到位姿图中
      auto node_id = pose_graph_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
      
      CHECK_EQ(node_id.trajectory_id, trajectory_id_);

      // 这里的InsertionResult的类型是 TrajectoryBuilderInterface::InsertionResult
      insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
          node_id, 
          matching_result->insertion_result->constant_data,
          std::vector<std::shared_ptr<const Submap>>(
              matching_result->insertion_result->insertion_submaps.begin(),
              matching_result->insertion_result->insertion_submaps.end())});
    }
```

# 总体调用流程

![](2021-12-01 10-32-17屏幕截图.png)

![](2021-12-01 10-33-04屏幕截图.png)
