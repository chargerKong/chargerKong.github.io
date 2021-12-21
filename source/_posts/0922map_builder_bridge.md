---
title: MapBuilderBridge类-cartographer
date: 2021-09-13 15:59:22
tags: cartographer
---

本文主要介绍map_builder类的大致实现逻辑

# AddTrajectoy

```c++
int MapBuilderBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  // Step: 1 开始一条新的轨迹, 返回新轨迹的id,需要传入一个函数
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      // lambda表达式 local_slam_result_callback_
      [this](const int trajectory_id, 
             const ::cartographer::common::Time time,
             const Rigid3d local_pose,
             ::cartographer::sensor::RangeData range_data_in_local,
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult>) {
        // 保存local slam 的结果数据 5个参数实际只用了4个
        OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      });
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  // Step: 2 为这个新轨迹 添加一个SensorBridge
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.tracking_frame,
      node_options_.lookup_transform_timeout_sec, 
      tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id)); // CollatedTrajectoryBuilder
  
  // Step: 3 保存轨迹的参数配置
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}
```

## 第一步：开始一条新的轨迹, 返回新轨迹的id

通过调用map_builder来添加一条轨迹，调用函数AddTrajectoryBuilder，第三个参数为一个lambda表达式

\[this]: 以值的形式捕获this指针，传入了五个参数，在lambda表达式的函数体中调用了另外一个函数

```c++
  // Step: 1 开始一条新的轨迹, 返回新轨迹的id,需要传入一个函数
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      // lambda表达式 local_slam_result_callback_
      [this](const int trajectory_id, 
             const ::cartographer::common::Time time,
             const Rigid3d local_pose,
             ::cartographer::sensor::RangeData range_data_in_local,
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult>) {
        // 保存local slam 的结果数据 5个参数实际只用了4个
        OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      });
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";
```

## 第二步，为轨迹添加一个sensorBridge

每一个轨迹都会有一个sensorBridge, 保存在变量sensor_bridges_之中

```
std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
```

因此，在新增了轨迹之后，需要把SensorBridge的信息也要添加进变量之中。

```
  // Step: 2 为这个新轨迹 添加一个SensorBridge
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.tracking_frame,
      node_options_.lookup_transform_timeout_sec, 
      tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id)); // CollatedTrajectoryBuilder
```

## 第三步，  保存轨迹的参数配置

保存轨迹 的参数配置到trajectory_options_ 中，返回的值的第二个参数可以判断是否插入元素成功

```
  // Step: 3 保存轨迹的参数配置
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
```

# 结束指定id的轨迹
首先判断此ID的轨迹是否活跃，调用map_builder的FinishTrajectory方法结束指定轨迹。最后也需要在sensor_bridges_ 里删除对应的ID

```c++
void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK(GetTrajectoryStates().count(trajectory_id));
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}
```

# 处理地图信息

处理地图信息的函数一个service，它的数据格式为

```
int32 trajectory_id
int32 submap_index
---
cartographer_ros_msgs/StatusResponse status
int32 submap_version
cartographer_ros_msgs/SubmapTexture[] textures
```

request的内容为轨迹的ID，和submap的index

response的有栅格的具体的信息 textures，数据格式为

```
uint8[] cells  此值表示栅格的值
int32 width
int32 height
float64 resolution
geometry_msgs/Pose slice_pose  具体的位姿
```

下面是具体的函数，具体的submap信息其实都存储在map_builder_之中，通过SubmapToProto来获得

```c++
/** 
 * @brief 获取对应id轨迹的 索引为 submap_index 的地图的栅格值及其他信息
 * 
 * @param[in] request 轨迹id与submap的index
 * @param[in] response 是否成功
 */
void MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
    
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  // 获取压缩后的地图数据
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    response.status.message = error;
    return;
  }

  response.submap_version = response_proto.submap_version();

  // 将response_proto中的地图栅格值存入到response中
  for (const auto& texture_proto : response_proto.textures()) {
    response.textures.emplace_back();
    // 获取response中存储地图变量的引用
    auto& texture = response.textures.back();
    // 对引用的变量进行赋值
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response.status.message = "Success.";
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
}
```

# 获取轨迹的状态

轨迹的状态也是通过map_builder_中的pose_graph获取，但是有的轨迹还没有到pose_graph中，需要到sensor bridge中查找

```c++
// 向pose graph中添加新的active状态的轨迹, 并返回所有的轨迹状态
std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  auto trajectory_states = map_builder_->pose_graph()->GetTrajectoryStates();
  // Add active trajectories that are not yet in the pose graph, but are e.g.
  // waiting for input sensor data and thus already have a sensor bridge.
  // 为活跃的轨迹添加active状态, 如果trajectory_states中存在这个轨迹id,则会被忽略不会被添加进去
  for (const auto& sensor_bridge : sensor_bridges_) {
    trajectory_states.insert(std::make_pair(
        sensor_bridge.first,
        ::cartographer::mapping::PoseGraphInterface::TrajectoryState::ACTIVE));
  }
  return trajectory_states;
}
```

# 获取所有submap的信息

包括 trajectory_id,submap_index,submap_version,pose。submaplist的消息也是在map_builder中。从pose_graph中提取，

```c++
cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {
    cartographer_ros_msgs::SubmapEntry submap_entry;
    submap_entry.is_frozen = map_builder_->pose_graph()->IsTrajectoryFrozen(
        submap_id_pose.id.trajectory_id);
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}
```

# 获得前端的轨迹数据

获取local坐标系下的TrajectoryData

从sensor_bridge_中循环找每一条轨迹的消息，把消息全部放入local_trajectory_data，local_trajectory_data中的数据结构为

```c++
  struct LocalTrajectoryData {
    // Contains the trajectory data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    
    // LocalSlamData中包含了local slam的一些数据, 包含当前时间, 当前估计的位姿, 以及累计的所有雷达数据
    struct LocalSlamData {
      ::cartographer::common::Time time;
      ::cartographer::transform::Rigid3d local_pose;
      ::cartographer::sensor::RangeData range_data_in_local;
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;
    cartographer::transform::Rigid3d local_to_map;  // local frame 到 global frame间的坐标变换

    // published_frame 到 tracking_frame 间的坐标变换
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;

    // c++11: std::shared_ptr 主要的用途就是方便资源的管理, 自动释放没有指针引用的资源
    // 使用引用计数来标识是否有其余指针指向该资源.(注意, shart_ptr本身指针会占1个引用)
    // 引用计数是分配在动态分配的, std::shared_ptr支持拷贝, 新的指针获可以获取前引用计数个数
  };
```



从local_slam_data_中获取local_slam_data

从map_builder_的pose_graph中获取local frame 到 global frame间的坐标变换

从sensor_bridge中获得从published_frame 到 tracking_frame 间的坐标变换

```c++
// 获取local坐标系下的TrajectoryData
std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData>
MapBuilderBridge::GetLocalTrajectoryData() {
  std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    // 获取local slam 数据
    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data;
    {
      absl::MutexLock lock(&mutex_);
      if (local_slam_data_.count(trajectory_id) == 0) {
        continue;
      }
      // 读取local_slam_data_要上锁
      local_slam_data = local_slam_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);

    // 填充LocalTrajectoryData
    local_trajectory_data[trajectory_id] = {
        local_slam_data,

        // local frame 到 global frame间的坐标变换
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),

        // published_frame 到 tracking_frame 间的坐标变换
        sensor_bridge.tf_bridge().LookupToTracking(
            local_slam_data->time,
            trajectory_options_[trajectory_id].published_frame),

        trajectory_options_[trajectory_id]};
  } // end for
  return local_trajectory_data;
}
```

