---
title: 残差项构建-cartographer
date: 2021-12-13 18:38:29
tags: cartographer
---

之前介绍了四种构建位姿的方式：

1. 节点 通过 GetLocalToGlobalTransform * constant_data->local_pose 进行global下位姿的计算
2. 子图 通过对前一个子图到后一个子图的坐标变换进行累计, 得到子图在global坐标系下的位姿
3. 子图内约束 local坐标系系下, 子图原点指向节点间的坐标变换
4. 子图间约束 根据global坐标计算初值, 然后通过分支定界算法粗匹配与ceres的精匹配, 获取校准后的位姿, 最后计算local坐标系系下, 子图原点指向校准后的节点间的坐标变换

下面构建残差，通过构建2D优化问题

```c++
/**
 * @brief 搭建优化问题并进行求解
 * 
 * @param[in] constraints 所有的约束数据
 * @param[in] trajectories_state 轨迹的状态
 * @param[in] landmark_nodes landmark数据
 */
void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectories_state,
    const std::map<std::string, LandmarkNode>& landmark_nodes)
{}
```

# 调用处

于`pose_graph_2d.cc`文件中的函数`RunOptimization`

```c++
  // No other thread is accessing the optimization_problem_,
  // data_.constraints, data_.frozen_trajectories and data_.landmark_nodes
  // when executing the Solve. Solve is time consuming, so not taking the mutex
  // before Solve to avoid blocking foreground processing.
  // Solve 比较耗时, 所以在执行 Solve 之前不要加互斥锁, 以免阻塞其他的任务处理
  // landmark直接参与优化问题
  optimization_problem_->Solve(data_.constraints, GetTrajectoryStates(),
                               data_.landmark_nodes);
```



# Solve具体实现

他需要传入 约束，轨迹状态和landmark，注意landmark是存在PoseGraph2D中的，而不是保存在优化问题类中的

冻结的轨迹就是被不会被优化的了，第一个子图也不会被改变。

```c++
void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectories_state,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  // 记录下所有FROZEN状态的轨迹id
  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }
```

创建ceres 优化问题

```c++
  // 创建优化问题对象
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);
```

把需要优化的变量，即submap，node，landmark的数据放入以下变量中

注意landmark的数据结构是CeresPose， 主要是用于更新其姿态而建立

```c++
  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapSpec.
  // ceres需要double的指针, std::array能转成原始指针的形式
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;
```

## 将需要优化的子图位姿设置为优化参数

把子图的global_pose 放入C_submaps，这里插入的子图的位姿为（x, y, theta）

```c++
  // 将需要优化的子图位姿设置为优化参数
  for (const auto& submap_id_data : submap_data_) {
    // submap_id的轨迹 是否是 已经冻结的轨迹
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    // 将子图的global_pose放入C_submaps中
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
```

添加需要优化的参数

```c++
    // c++11: std::array::data() 返回指向数组对象中第一个元素的指针
    // Step: 添加需要优化的数据 这里显式添加参数块,会进行额外的参数块正确性检查
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
```

 Step: 如果是第一幅子图, 或者是已经冻结的轨迹中的子图, 不优化这个子图位姿

```c++
    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      // Step: 如果是第一幅子图, 或者是已经冻结的轨迹中的子图, 不优化这个子图位姿
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
```

## 将需要优化的节点位姿设置为优化参数

逻辑和需要优化的子图位姿一样

```c++
  // 将需要优化的节点位姿设置为优化参数  
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    // 将节点的global_pose_2d放入C_nodes中
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    // 第一个节点的位姿也是要优化的变量, 不是固定的
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
```



## 第一种残差

添加第一种残差块，参数

- cost_function: CreateAutoDiffSpaCostFunction(constraint.pose),
- 核函数：
- 优化变量：submap
- 优化变量：node

```c++
  // Step: 第一种残差 将节点与子图原点在global坐标系下的相对位姿 与 约束 的差值作为残差项
  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        // 根据SPA论文中的公式计算出的残差的CostFunction
        CreateAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        // 为闭环约束提供一个Huber的核函数,用于降低错误的闭环检测对最终的优化结果带来的负面影响
        constraint.tag == Constraint::INTER_SUBMAP // 核函数
            ? new ceres::HuberLoss(options_.huber_scale()) // param: huber_scale
            : nullptr,
        C_submaps.at(constraint.submap_id).data(), // 2个优化变量
        C_nodes.at(constraint.node_id).data());
  }
```

### 残差函数

```c++
// 创建AutoDiffSpaCostFunction
ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const PoseGraphInterface::Constraint::Pose& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}
```

返回的是SpaCostFunction2D的构造函数

### SpaCostFunction2D

构造函数存储了observed_relative_pose_ 的值，在` bool operator()`中计算了残差，并且给`e`赋值。

- observed_relative_pose_ 是constraint.pose。**是所有子图内和子图间的约束**

无论是子图间约束还是子图内约束，都是通过local坐标系进行计算的，如node在local坐标系下的位姿的逆乘以submap在local坐标系下的逆

```c++
class SpaCostFunction2D {
 public:
  explicit SpaCostFunction2D(
      const PoseGraphInterface::Constraint::Pose& observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    const std::array<T, 3> error =
        // 计算残差并乘以尺度
        ScaleError(ComputeUnscaledError(
                       transform::Project2D(observed_relative_pose_.zbar_ij),
                       start_pose, end_pose),
                   observed_relative_pose_.translation_weight,
                   observed_relative_pose_.rotation_weight);
    // c++11: std::copy 拷贝元素到e中
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  // 约束, 图结构的边
  const PoseGraphInterface::Constraint::Pose observed_relative_pose_;
};
```

### 计算残差

通过start和end来计算两个实际点的相对偏移，因为relative_pose为 end在start上的位姿

所以求他们相对偏差$h(c_s,c_e)$的时候应该求该偏差在start上的位姿,其中 $c_s,c_e$分别为start和end在global上的位姿。

偏差为
$$
h(c_s,c_e)=
\begin{cases}
R_s^T(t_e-t_s) \\
\theta_e-\theta_s
\end{cases}
$$
最后直接相减

```c++
 /* 
 * @param[in] relative_pose 
 * @param[in] start 
 * @param[in] end 
 * @return std::array<T, 3> 
 */
template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const transform::Rigid2d& relative_pose, const T* const start,
    const T* const end) {
  // 旋转矩阵R
  const T cos_theta_i = cos(start[2]);
  const T sin_theta_i = sin(start[2]);
  const T delta_x = end[0] - start[0]; // t2 -t1
  const T delta_y = end[1] - start[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y, // R.inverse * (t2 -t1)
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end[2] - start[2]};
  return {{T(relative_pose.translation().x()) - h[0],
           T(relative_pose.translation().y()) - h[1],
           common::NormalizeAngleDifference(
               T(relative_pose.rotation().angle()) - h[2])}};
}
```

将节点与子图原点在global坐标系下的相对位姿 与 约束 的差值作为残差项

- 第一种坐标变换: 节点与子图原点在global坐标系下的坐标变换，即通过start和end计算得到

- 第二种坐标变换: 子图内约束与子图间约束，relative_pose

