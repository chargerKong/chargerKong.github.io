---
title: cartographer_ros之node_main.cc详解下
date: 2021-08-23 15:59:22
tags: cartographer
---

打开cartographer_ros下的node_main.cc

## main函数

首先看一下main函数

```
int main(int argc, char** argv) {

  // note: 初始化glog库
  google::InitGoogleLogging(argv[0]);
  
  // 使用gflags进行参数的初始化. 其中第三个参数为remove_flag
  // 如果为true, gflags会移除parse过的参数, 否则gflags就会保留这些参数, 但可能会对参数顺序进行调整.
  // 开始解析参数
  google::ParseCommandLineFlags(&argc, &argv, true);

  /**
   * @brief glog里提供的CHECK系列的宏, 检测某个表达式是否为真
   * 检测expression如果不为真, 则打印后面的description和栈上的信息
   * 然后退出程序, 出错后的处理过程和FATAL比较像.
   */
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  // ros节点的初始化
  ::ros::init(argc, argv, "cartographer_node");

  // 一般不需要在自己的代码中显式调用
  // 但是若想在创建任何NodeHandle实例之前启动ROS相关的线程, 网络等, 可以显式调用该函数.
  ::ros::start();

  // 使用ROS_INFO进行glog消息的输出
  cartographer_ros::ScopedRosLogSink ros_log_sink;

  // 开始运行cartographer_ros
  cartographer_ros::Run();

  // 结束ROS相关的线程, 网络等
  ::ros::shutdown();
}
```

如果不进行初始化，那么就无法使用glog库

```
  // note: 初始化glog库
  google::InitGoogleLogging(argv[0]);
```

这里开始解析参数，即就是DEFINE_bool这里的参数，前面加上FALGS的操作

```
  // 使用gflags进行参数的初始化. 其中第三个参数为remove_flag
  // 如果为true, gflags会移除parse过的参数, 否则gflags就会保留这些参数, 但可能会对参数顺序进行调整.
  // 开始解析参数
  google::ParseCommandLineFlags(&argc, &argv, true);
```

对ros结点进行初始化，并且命名为cartographer_node，定义自己的log系统

```
  // ros节点的初始化
  ::ros::init(argc, argv, "cartographer_node");

  // 一般不需要在自己的代码中显式调用
  // 但是若想在创建任何NodeHandle实例之前启动ROS相关的线程, 网络等, 可以显式调用该函数.
  ::ros::start();

  // 使用ROS_INFO进行glog消息的输出
  cartographer_ros::ScopedRosLogSink ros_log_sink;
```

最后开始运行cartographer_ros

```
  cartographer_ros::Run();
```

## Run函数

开启监听tf的独立线程，开启监听tf独立的线程

```
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  // 开启监听tf的独立线程，开启监听tf独立的线程
  tf2_ros::TransformListener tf(tf_buffer);
```

根据Lua配置文件中的内容, 为node_options, trajectory_options 赋值

```
std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
```

建立 map_builder类，map_build类是一个完整的slam, 包括前端和后端.auto的变量必须要赋值。

```
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
```

对Node类进行初始化，将ROS的topic传入SLAM, 也就是MapBuilder

```
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
```

加载pbstream

```
  // 如果加载了pbstream文件, 就执行这个函数
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }
```

最后结束的时候还会有一次全局优化

```
  // 当所有的轨迹结束时, 再执行一次全局优化
  node.RunFinalOptimization();
```

## std::tie

这个函数的作用就和Python的多个变量赋值一样，但是他只接受元祖作为赋值

```
std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
```

在这里，LoadOptions的返回值为std::tuple，注意return，是通过make_tuple生成的tuple

```
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename)
    {
    ...
    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
    }
```



