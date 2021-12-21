---
title: cartographer_ros之node_main.cc详解上
date: 2021-08-20 18:38:29
tags: cartographer
---



打开文件`node_main.cc`

# gflag工具 

```c
DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
```

在这里可以看见`DEFINE_string`和`DEFINE_bool`，他们是一个宏，被定义在gflags.h文件中，**注意：gflags是一套命令行参数解析工具**。其使用语法的参数为**命令行参数名**, **参数默认值,** 以及**参数的帮助信息**

`configuration_directory`和`configuration_basename`就是我们在启动cartographer_ros的时候通过launch文件传进来的参数

当我们需要使用参数的时候，只需要在前面加上`FLAGS_`前缀就行，比如`FLAGS_configuration_directory`



# main 函数

初始化glog库

glog库是一个比较强大的日志库。里面提供了很多的帮助做log的函数和宏

glog里提供的CHECK系列的宏, 检测某个表达式是否为真

   \* 检测expression如果不为真, 则打印后面的description和栈上的信息，然后退出程序, 出错后的处理过程和FATAL比较像.

```
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
```

## 基于glog创建自己的输出日志的方式

```c++
class ScopedRosLogSink : public ::google::LogSink {
 public:
  ScopedRosLogSink();
  ~ScopedRosLogSink() override;

  void send(::google::LogSeverity severity, const char* filename,
            const char* base_filename, int line, const struct std::tm* tm_time,
            const char* message, size_t message_len) override;

  void WaitTillSent() override;

 private:
  bool will_die_;
};
```

在这里定义了自定义的输出日志的方式: 使用ROS_INFO进行glog消息的输出，在这里主要定义了两个方法，一个是send，发送消息日志使用，另外一个是WaitTillSent

在构造函数ScopedRosLogSink()中，定义了`will_die_=false`, 调用AddLogSink(), 将ScopedRosLogSink类注册到glog中

 从而每一次打log的时候，就不需要glog本身自己的输出格式，而是会用send函数log消息具体内容的生成

```
ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
```

在这里重载了send方法，定义了自己消息格式`message_string`，并且使用ROS_INFO_STREAM进行消息输出

```c++
/**
 * @brief 重载了send()方法, 使用ROS_INFO进行glog消息的输出
 * 
 * @param[in] severity 消息级别
 * @param[in] filename 全路径文件名
 * @param[in] base_filename 文件名
 * @param[in] line 消息所在的文件行数
 * @param[in] tm_time 消息的时间
 * @param[in] message 消息数据本体
 * @param[in] message_len 消息长度
 */
void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, 
                            const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
  switch (severity) {
    case ::google::GLOG_INFO:
      ROS_INFO_STREAM(message_string);
      break;

    case ::google::GLOG_WARNING:
      ROS_WARN_STREAM(message_string);
      break;

    case ::google::GLOG_ERROR:
      ROS_ERROR_STREAM(message_string);
      break;

    case ::google::GLOG_FATAL:
      ROS_FATAL_STREAM(message_string);
      will_die_ = true;
      break;
  }
}
```

WaitTillSent()会在每次send后调用, 用于一些异步写的场景

```c++
void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
```

