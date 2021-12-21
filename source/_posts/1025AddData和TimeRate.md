---
title: AddSensorData与RateTimer-cartographer
date: 2021-10-25 09:59:22
tags: cartographer
---

本节介绍的函数位于`cartographer/mapping/internal/collated_trajectory_builder.cc`文件中

# AddSensorData

AddSensorData 函数为CollatedTrajectoryBuilder的方法

此类继承于TrajectoryBuilderInterface接口，接口中的有多种AddSensorData函数，用于处理各个传感器数据，如

```c++
  // 处理雷达点云数据
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
  }
```

## Sensor类的sensor::MakeDispatchable

```c++
// c++11: template <typename DataType> 
// 函数模板的调用使用 实参推演 来进行
// 类模板 模板形参的类型必须在类名后的尖括号中明确指定, 不能使用实参推演 
// 在类外声明一个 函数模板, 使用 实参推演 的方式来使得 类模板可以自动适应不同的数据类型


// 根据传入的data的数据类型,自动推断DataType, 实现一个函数处理不同类型的传感器数据
template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}
```

此函数的作用是生称一个指向Dispatchable类的unique指针。

注意他是一个模板函数，**使用的时候不需要加上尖括号，能自动推断出模板的数据类型**，还是上面那个例子，则DataType为`sensor::TimedPointCloudData`类型

### 模板类 Dispatchable

此类是接口Data的是一个继承，主要的作用是把数据加入到trajectory_builder

```c++
template <typename DataType>
class Dispatchable : public Data {
 public:
  Dispatchable(const std::string &sensor_id, const DataType &data)
      : Data(sensor_id), data_(data) {}

  common::Time GetTime() const override { return data_.time; }

  // 调用传入的trajectory_builder的AddSensorData()
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *const trajectory_builder) override {
    trajectory_builder->AddSensorData(sensor_id_, data_);
  }

  const DataType &data() const { return data_; }

 private:
  const DataType data_;
};
```

# RateTimer

## RateTimer建立

在HandleCollatedSensorData中，建立了RateTimer。RateTimer存储了一些还需要使用的数据的时间

```c++
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  // 找不到就新建一个
  if (it == rate_timers_.end()) {
    // map::emplace()返回一个pair
    // emplace().first表示新插入元素或者原始位置的迭代器
    // emplace().second表示插入成功,只有在key在map中不存在时才插入成功
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, 
                 std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->GetTime());
```

rate_timers_ 是一个map，可以通过emplace的方式进行插入。emplace().first表示新插入元素或者原始位置的迭代器。

```c++
std::map<std::string, common::RateTimer<>> rate_timers_;
```

### RateTimer构造

注意，rate_timers_的值为common::RateTimer，common::RateTimer的构造函数为，表示需要保存的数据的持续时间

```c++
explicit RateTimer(const common::Duration window_duration)
      : window_duration_(window_duration) {}
```

它只接受一单个参数，common::Duration，他的构造为

```c++
std::forward_as_tuple(common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
```

## 数据更新

```c++
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  // 找不到就新建一个
  	...
  //数据更新，在now - kSensorDataRatesLoggingPeriodSeconds之外的时间就不要了
  it->second.Pulse(data->GetTime());
```

`it->second.Pulse(data->GetTime());`把当前传感器的数据时间和当前时间加入，并且剔除在当前传感器数据前面kSensorDataRatesLoggingPeriodSeconds 秒的时间

## Debug log

程序运行期间，只要当前的时间距离上一次的时间大于15s了，就打印一次log

```c++
if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }
// 也就是跑carto时候的消息：
  // [ INFO]: collated_trajectory_builder.cc:72] imu rate: 10.00 Hz 1.00e-01 s +/- 4.35e-05 s (pulsed at 100.44% real time)
  // [ INFO]: collated_trajectory_builder.cc:72] scan rate: 19.83 Hz 5.04e-02 s +/- 4.27e-05 s (pulsed at 99.82% real time)
```

### DebugString

```c++
  // Returns a debug string representation.
  std::string DebugString() const {
    if (events_.size() < 2) {
      return "unknown";
    }

    // c++11: std::fixed 与 std::setprecision(2) 一起使用, 表示输出2位小数点的数据

    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
        << DeltasDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)";
    return out.str();
  }
```



