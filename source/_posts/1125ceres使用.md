---
title: ceres使用
date: 2021-11-25 18:38:29
tags: cartographer
---

cartographer的ceres扫描匹配使用在`cartographer/mapping/internal/2d/local_trajetory_builder.cc`中

# ceres 调用处

```c++
  // 使用ceres进行扫描匹配
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
```

如果没有进行暴力匹配搜索的话，pose_prediction和initial_ceres_pose 是一样的，后面的是点云，地图，以及两个输出



# ceres使用的头文件

位于文件`cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h`中，需要传入的是Options

```c++
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D& options);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const Grid2D& grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
  const proto::CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;
};
```

## 成员变量

- options_： protobuf形式的option
- ceres_solver_options_: ceres_solver形式的option

## 构造函数

```c++
CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}
```

对两个option进行赋值，线性分解的方法使用的是稠密的QR分解

## Match

### 定义变量

```c++
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
```

### 平移残差的逼近

```c++
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.translation_weight(), target_translation), // 平移的目标值, 没有使用校准后的平移
      nullptr /* loss function */, ceres_pose_estimate);      // 平移的初值
```

第一个参数是函数，需要定义优化的目标函数，第二个nullptr表示没有loss function，第三个是更新的变量，

查看目标函数，类的静态函数，返回一个类的实例

```c++
class TranslationDeltaCostFunctor2D {
 public:
  // 静态成员函数, 返回CostFunction
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d& target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals 需要更新的变量维度*/, 
                                           3 /* pose variables 实际变量的维度*/>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }
```

构造函数：

```c++
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}
```

查看一下（）的重载，即目标函数

```c++
  // 平移量残差的计算, (pose[0] - x_)的平方ceres会自动加上
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }
```

优化变量pose，两个维度，减去的x_ 和y_ 是target_translation的值，是**位姿估计器**的值。要把pose优化到位姿估计器上去

### 旋转残差

```c++
 // 旋转的残差, 固定了角度不变
  CHECK_GT(options_.rotation_weight(), 0.);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.rotation_weight(), ceres_pose_estimate[2]), // 角度的目标值
      nullptr /* loss function */, ceres_pose_estimate);       // 角度的初值
```

查看旋转的目标函数

```c++
class RotationDeltaCostFunctor2D {
 public:
  // 静态成员函数, 返回CostFunction
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }
```

这里要优化的变量是ceres_pose_estimate的第三个变量，即角度。所以residuals这里是1



构造函数，也是对角度进行了赋值

```c++
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}
```

旋转的目标函数，转换到位姿估计器上的角度，如有暴力匹配，则是**暴力匹配之后的角度**

```c++
  // 旋转量残差的计算
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }
```

**调参问题，如果位姿估计器不准确，可以把平移和旋转的逼近直接注释掉**，有可能叠图

### 地图残差的逼近

```c++
  // 地图部分的残差
  CHECK_GT(options_.occupied_space_weight(), 0.);
  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
    case GridType::TSDF:
      problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D&>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
  }
```

可以待补充，通过空闲的概率来获取插值后空闲的概率来表示残差项，free越小，概率才越大

# 总结

- 可能有问题的点
  平移和旋转的残差项是逼近于先验位姿的, 当先验位姿不准确时会产生问题
- 可能的改进建议
  先将地图的权重调大, 平移旋转的权重调小, 如 1000, 1, 1, 或者 100, 1, 1
  调参没有作用的时候可以将平移和旋转的残差项注释掉

如果都不行，可以把位姿估计器改成gmapping的或者是karto的试一下