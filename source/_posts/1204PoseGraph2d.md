---
title: PoseGraph2D-cartographer
date: 2021-12-04 18:38:29
tags: cartographer
---

PoseGraph2D文件位于`mapping/internel/2d`

继承于`PoseGraphInterface`

# PoseGraphInterface

节点: tracking_frame的位姿, 子图原点的位姿

约束: tracking_frame与子图原点间的坐标变换

## 约束

```c++
  // 包含了子图的id, 节点的id, 节点j相对于子图i的坐标变换, 以及节点是在子图内还是子图外的标志
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;
      double translation_weight;
      double rotation_weight;
    };
    SubmapId submap_id;  // 'i' in the paper.
    NodeId node_id;      // 'j' in the paper.

    // Pose of the node 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };
```

# LandmarkNode

这里包含一个landmark数据是相对于tracking_frame的相对坐标变换

```c++
  struct LandmarkNode {
    // landmark数据是相对于tracking_frame的相对坐标变换
    struct LandmarkObservation {
      int trajectory_id;
      common::Time time;
      transform::Rigid3d landmark_to_tracking_transform;
      double translation_weight;
      double rotation_weight;
    };
    // 同一时刻可能会观测到多个landmark数据
    std::vector<LandmarkObservation> landmark_observations;
    // 这帧数据对应的tracking_frame在global坐标系下的位姿
    absl::optional<transform::Rigid3d> global_landmark_pose;
    bool frozen = false;
  };
```

其他

```c++
  struct SubmapPose {
    int version;
    transform::Rigid3d pose; //子图原点的一个坐标
  };

  struct SubmapData {//指向了具体栅格指针
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };

  // tag: TrajectoryData
  struct TrajectoryData {
    double gravity_constant = 9.8;
    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
    absl::optional<transform::Rigid3d> fixed_frame_origin_in_map;
  };

  enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };

  using GlobalSlamOptimizationCallback =
      std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
                         const std::map<int /* trajectory_id */, NodeId>&)>;
```

# PoseGraph2D

PoseGraphData数据结构下包含了后端优化需要的所有数据

```c++
struct PoseGraphData {
  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.

  // submap_data_ 里面,包含了所有的submap,以及以及完成插入的节点的ID
  MapById<SubmapId, InternalSubmapData> submap_data;

  // Global submap poses currently used for displaying data.
  // submap 在 global 坐标系下的坐标
  MapById<SubmapId, optimization::SubmapSpec2D> global_submap_poses_2d;
  MapById<SubmapId, optimization::SubmapSpec3D> global_submap_poses_3d;

  // Data that are currently being shown.
  // 所有的轨迹节点的id与 节点的在global坐标系下的坐标, 在local map 下的坐标与时间
  MapById<NodeId, TrajectoryNode> trajectory_nodes;

  // Global landmark poses with all observations.
  std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
      landmark_nodes;

  // How our various trajectories are related.
  TrajectoryConnectivityState trajectory_connectivity_state;
  // 节点的个数
  int num_trajectory_nodes = 0;
  // 轨迹与轨迹的状态
  std::map<int, InternalTrajectoryState> trajectories_state;

  // Set of all initial trajectory poses.
  std::map<int, PoseGraph::InitialTrajectoryPose> initial_trajectory_poses;

  // 所有的约束数据
  std::vector<PoseGraphInterface::Constraint> constraints;
};

```

# 构造函数

被调用处

`mapping/map_builder.cc`中

```c++
  // 2d位姿图(后端)的初始化
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
```

###  数据传入

各个数据都是从global_trajetory中获得

# AddNode

## 调用之处：

`mapping/internal/global_trajectory_builder.cc`的AddSensorData函数中，AddSensorData是先进行扫描匹配，然后再将扫描匹配的结果当做节点插入到后端的位姿图中

```c++
    // matching_result->insertion_result 的类型是 LocalTrajectoryBuilder2D::InsertionResult
    // 如果雷达成功插入到地图中
    if (matching_result->insertion_result != nullptr) {
      kLocalSlamInsertionResults->Increment();

      // 将匹配后的结果 当做节点 加入到位姿图中
      auto node_id = pose_graph_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
```

下面是前端匹配的结果的数据结构，很明显，返回的数据是一个点云数据，包含 sensor::RangeData ,这里有点云的原点在local坐标系下的坐标，以及点云的数据在PointCloud类里面，还有global的坐标系下的pose，会随着优化而改变

```c++
  // 将点云插入到地图后的result
  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps; // 最多只有2个子图的指针
  };
  // 扫描匹配的result
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local; // 经过扫描匹配之后位姿校准之后的雷达数据
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
  };
```

其中

```c++
struct TrajectoryNode {
  // 前端匹配所用的数据与计算出的local坐标系下的位姿
  struct Data {
    common::Time time;

    // Transform to approximately gravity align the tracking frame as
    // determined by local SLAM.
    Eigen::Quaterniond gravity_alignment;

    // Used for loop closure in 2D: voxel filtered returns in the
    // 'gravity_alignment' frame.
    sensor::PointCloud filtered_gravity_aligned_point_cloud;

    // Used for loop closure in 3D.
    sensor::PointCloud high_resolution_point_cloud;
    sensor::PointCloud low_resolution_point_cloud;
    Eigen::VectorXf rotational_scan_matcher_histogram;

    // The node pose in the local SLAM frame.
    transform::Rigid3d local_pose;
  };
  common::Time time() const { return constant_data->time; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  std::shared_ptr<const Data> constant_data; // 不会被后端优化修改的数据, 所以是constant

  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose; // 后端优化后, global_pose会发生改变
```



所以这里的调用，传入的内容是

- 指向TrajectoryNode::Data的 前端匹配使用的数据，他不会被后端优化所修改
- 轨迹ID
- 活跃的子图



## 实现过程

把节点在local坐标系上的Pose，转为global坐标系下的，注意，变换是一个
$$
T_{local->global}*Pose_{local}=Pose_{global}
$$

```c++
  // 将节点在local坐标系下的坐标转成global坐标系下的坐标
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
```

向节点列表中添加一个新的节点, 并保存新生成的submap

```c++
  // 向节点列表加入节点,并得到节点的id
  const NodeId node_id = AppendNode(constant_data, trajectory_id,
                                    insertion_submaps, optimized_pose);
```

把状态已经完成的节点放入WorkItem，并计算节点的约束

```c++
  // 获取第一个submap是否是完成状态
  const bool newly_finished_submap =
      insertion_submaps.front()->insertion_finished();

  // 把计算约束的工作放入workitem中等待执行
  AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    return ComputeConstraintsForNode(node_id, insertion_submaps,
                                     newly_finished_submap);
  });
```

返回已经加入后的节点ID

```c++
return node_id;
```

## AppendNode

首先是检查记录该轨迹的状态的键值对是否建立，没有的话就建立一个, 并为轨迹添加采样器

```
  AddTrajectoryIfNeeded(trajectory_id);
```

检查是否可以添加任务，**然后添加节点**，只有这一个地方可以添加节点，添加的内容为、

- constant_data，前端匹配所用的数据与计算出的local坐标系下的位姿
- optimized_pose，global坐标系下的位姿

```c++
  // 向节点列表中添加一个新的节点
  const NodeId node_id = data_.trajectory_nodes.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  // 节点总个数加1
  ++data_.num_trajectory_nodes;
```

**子图的生成和保存**

后端是不会改变子图的内容的，只有前端会改变，一旦子图变成finish状态了，就都不会改变了。



所有代码如下

```c++
/**
 * @brief 向节点列表中添加一个新的节点, 并保存新生成的submap
 * 
 * @param[in] constant_data 节点数据的指针
 * @param[in] trajectory_id 轨迹id
 * @param[in] insertion_submaps 子地图指针的vector
 * @param[in] optimized_pose 当前节点在global坐标系下的坐标
 * @return NodeId 返回新生成的节点id
 */
NodeId PoseGraph2D::AppendNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
    const transform::Rigid3d& optimized_pose) {
  absl::MutexLock locker(&mutex_);

  // 如果轨迹不存在, 则将轨迹添加到连接状态里并添加采样器
  AddTrajectoryIfNeeded(trajectory_id);

  // 根据轨迹状态判断是否可以添加任务
  if (!CanAddWorkItemModifying(trajectory_id)) {
    LOG(WARNING) << "AddNode was called for finished or deleted trajectory.";
  }

  // 向节点列表中添加一个新的节点
  const NodeId node_id = data_.trajectory_nodes.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  // 节点总个数加1
  ++data_.num_trajectory_nodes;

  // Test if the 'insertion_submap.back()' is one we never saw before.
  // 如果是刚开始的轨迹, 或者insertion_submaps.back()是第一次看到, 就添加新的子图
  if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
              ->data.submap != insertion_submaps.back()) {
    // We grow 'data_.submap_data' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.

    // 如果insertion_submaps.back()是第一次看到, 也就是新生成的
    // 在data_.submap_data中加入一个空的InternalSubmapData
    const SubmapId submap_id =
        data_.submap_data.Append(trajectory_id, InternalSubmapData());
    
    // 保存后边的地图, 将后边的地图的指针赋值过去
    // 地图是刚生成的, 但是地图会在前端部分通过插入点云数据进行更新, 这里只保存指针
    // tag: 画图说明一下
    data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    LOG(INFO) << "Inserted submap " << submap_id << ".";
    kActiveSubmapsMetric->Increment();
  }
  return node_id;
}
```

