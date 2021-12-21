---
title: ImuTracker-cartographer
date: 2021-11-15 18:38:29
tags: cartographer
---



ImuTracker 的主要作用是根据 IMU的角速度和线性加速度进行位姿的预测，IMU原本给出的位姿是通过积分得出的，不准确的

# 头文件

ImuTracker 的主要作用是根据 IMU的角速度和线性加速度进行位姿的预测，IMU原本给出的位姿是通过积分得出的，不准确的

```c++
/**
 * @brief 
 * ImuTracker 的主要作用是根据 IMU的角速度来预测姿态,
 * 并根据IMU的线加速度来确定重力的方向, 并使用重力的方向来对姿态进行校准
 */
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  // 获取上一次预测位姿的时间戳
  common::Time time() const { return time_; }

  // Query the current orientation estimate.
  // 查询当前估计出的姿态
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;
  common::Time time_;
  common::Time last_linear_acceleration_time_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d gravity_vector_;
  Eigen::Vector3d imu_angular_velocity_;
};
```

# 构造函数

```c++
/**
 * @brief Construct a new Imu Tracker:: Imu Tracker object
 * 
 * @param[in] imu_gravity_time_constant 这个值在2d与3d情况下都为10
 * @param[in] time 
 */
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()), // 初始方向角
      gravity_vector_(Eigen::Vector3d::UnitZ()),    // 重力方向初始化为[0,0,1]
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}
```

orientation_ 在最开始时，在欧拉角表示则为（0，0，0）的。

# AddImuAngularVelocityObservation

角速度的校准

```c++
// 更新角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}
```

# AddImuLinearAccelerationObservation

此函数最主要的作用是计算出当前加速度下的位姿orientation_ 。

在这期间，还做了以下操作

- 更新目前时刻的线性加速度（和上一帧的加权平均），gravity_vector_ 。gravity_vector_ 就是 imu_linear_acceleration

  ```c++
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);  
  gravity_vector_ =
        (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  ```

  **理解**：如果imu时间间隔很小，那么就直接约等于上一帧的gravity_vector_  ，**第一帧的时候，gravity_vector_ 直接赋值为imu_linear_acceleration。**

  时间差约大就越相信当前传入的值，时间约小，就越相信之前的值

- 预测目前时刻的位姿

  ```c++
    // Step: 5 使用这个旋转量来校准当前的姿态
    orientation_ = (orientation_ * rotation).normalized();
  ```



```c++
/**
 * @brief 更新线性加速度的值,并根据重力的方向对上一时刻的姿态进行校准
 * 
 * @param[in] imu_linear_acceleration imu的线加速度的大小
 */
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 指数滑动平均法 exponential moving average
 
  // Step: 1 求delta_t, delta_t初始时刻为infinity, 之后为time_-last_linear_acceleration_time_
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;

  // Step: 2 求alpha, alpha=1-e^(-delta_t/10)
  // delta_t越大, alpha越大
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);

  // Step: 3 将之前的线加速度与当前传入的线加速度进行融合, 这里采用指数滑动平均法

  // 指数来确定权重, 因为有噪声的存在, 时间差越大, 当前的线性加速度的权重越大
  // 这里的gravity_vector_改成线性加速度更清晰一些
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
      
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // Step: 4 求得 线性加速度的值 与 由上一时刻姿态求出的线性加速度 间的旋转量
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());

  // Step: 5 使用这个旋转量来校准当前的姿态
  orientation_ = (orientation_ * rotation).normalized();

  // note: glog CHECK_GT: 第一个参数要大于第二个参数
  // 如果线性加速度与姿态均计算完全正确,那这二者的乘积应该是 0 0 1
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}
```

根据现在的线性加速度和之前的线性加速度不一样，就会产生一个旋转，此旋转可以根据当前的线性加速度对之前的线性加速度做一个矫正，矫正量就是rotation，把这个rotation作用于之前的orientation_ 上就可以得到现在的 orientation_ 。

# Advance

此函数 预测出time时刻的姿态与重力方向

```c++
/**
 * @brief 预测出time时刻的姿态与重力方向
 * 
 * @param[in] time 要预测的时刻
 */
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  // 上一时刻的角速度乘以时间,得到当前时刻相对于上一时刻的预测的姿态变化量,再转换成四元数
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  // 使用上一时刻的姿态 orientation_ 乘以姿态变化量, 得到当前时刻的预测出的姿态
  orientation_ = (orientation_ * rotation).normalized();

  // 根据预测出的姿态变化量,预测旋转后的线性加速度的值
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  // 更新时间
  time_ = time;
}
```

