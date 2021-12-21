---
title: LaunchConfiguration-launch文件
date: 2021-10-13 09:59:22
tags: ros2
---

# 功能简介

LaunchConfiguration参数是ros2中的常用函数之一，他的主要作用可以通过命令行传递参数。语法如下

```
<name>:=<value>
```

name是launch文件中的参数，加在launch文件之后，比如

```
ros2 launch my_packages launch_test.launch.py test_name:=my_value1
```

其中，my_packages是包名，test_name在launch文件中定义为

```
test_name = LaunchConfiguration("test_name", default="my_value2")
```

如此，test_name即可接受到命令行的值`my_value1`



# 最小案例

## 配置launch文件

launch_test.launch.py文件，此文件

- test_name：从命令行接受test_name的值，如果没有，则默认取值为klq。
- 启动cpp_pubsub结点，并传入test_name参数

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    test_name = LaunchConfiguration("test_name", default="klq")

    return LaunchDescription([
            
        Node(package='cpp_pubsub',
        executable='talker',
        output="screen",
        parameters=[{'test_name': test_name}]),
        
    ])
```

## cpp文件接受参数

在结点中，定义test_name变量

```c++
  std::string test_name;
  void init_param()
  {
      this->declare_parameter("test_name");
      this->get_parameter_or<std::string>("test_name", test_name, "000");
      std::cout << test_name << std::endl;
  }
```

取名为"test_name"的参数，并且赋值给test_name变量，如果在ros2的参数系统中没有，则默认赋值为“000”

最后打印变量，进行查看



完整node为下

```c++
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
    
    init_param();
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  std::string test_name;
  void init_param()
  {
    this->declare_parameter("test_name");
    this->get_parameter_or<std::string>("test_name", test_name, "000");
     RCLCPP_INFO(this->get_logger(), "test_name: '%s'", test_name);
  }

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

# 多launch文件

second_launch.launch.py现在多创立一个launch文件，

```python
def generate_launch_description():
    test_name = LaunchConfiguration("test_name", default="mdzzgouying")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/launch_test.launch.py']),
            launch_arguments={'test_name': test_name}.items(),
        ),
    ])
```

​	此launch文件调用launch_test.launch.py，传入launch参数test_name，在launch_test.launch.py文件中收到此test_name，并传入cpp

使用举例

```
ros2 launch cpp_pubsub second_launch.launch.py test_name:=mdzz
```

