---
title: ros2_c++之pub_sub
date: 2021-08-23 09:59:22
tags:

---

# 写一个publisher

```
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## 代码审查

`#include "rclcpp/rclcpp.hpp"`表示你可以使用ROS2系统中最常用的一些功能，`#include "std_msgs/msg/string.hpp"`则是用于发送消息所使用的ROS的String类型

同时，前面的头文件include也表示自己引入过的依赖，这在package和CMakeList里面也一定要添加。

下面一行表示创建了一个类MinimalPublisher，继承与Node。因此这里的每一个this都指代Node

```
class MinimalPublisher : public rclcpp::Node
```

public的构造函数，初始化count_为0，在构造函数里面，publisher的消息类型被初始化为String，topic的名字为“topic”，能接受的存储消息队列大小极限为10。下面timer被初始化，每一秒会调用timer_callback两次。

```
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
```

timer_callback函数是设置消息的地方，`RCLCPP_INFO`是可以确保消息会打印出来

```
private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
```

最后是main函数，是node实际执行的地方，rclcpp::init会初始化ROS2，rclcpp:spin是开始处理数据的地方，包括callbacks

```
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## 添加依赖至package.xml

首先，确保`<description>`, `<maintainer>` and `<license>` 这三个标签的内容以及填写完毕，然后在`ament_cmake` buildtool_depend依赖项的后面添加以下依赖

```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```



## CMakeLists.txt

现在打开CMakeLists.txt，在find_package(ament_cmake REQUIRED)的后面添加以下两句话

```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

之后，需要添加executable，命名为talker，这样我们就可以使用ros2 run来跑结点

```
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

最后添加以下install，这样才可以让ros2 run来找到可执行结点

```
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

现在可以清理以下CMakeLists，清理一些没有用的部分和注释，像下面所示

```
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

# 写一个subscriber

## 代码审查

```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

其实，subscriber和publisher基本一样，现在他的名字是minimal_subscriber，注意这里没有timer，因为订阅者只是反馈订阅到消息之后的动作

topic_callback接受消息，并且简单利用了RCLCPP_INFO打印了一下消息，在这里 main函数和package.xml和之前是一模一样的

## CMakeList.txt

重新打开CMakeLists, 添加一下订阅者的结点，并且安装到对应的目录下

```
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

# build and run

首先我们要确保自己已经安装了依赖

```
rosdep install -i --from-path src --rosdistro foxy -y
```

然后开始编译

```
colcon build
```

运行

```
ros2 run package_name node_name
```

