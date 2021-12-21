---
title: DeclareLaunchArgument-launch文件
date: 2021-10-13 10:59:22
tags: ros2
---

# 功能简介

DeclareLaunchArgument 即声明一下launch 文件的可以修改的参数

# 使用说明

一般情况下，DeclareLaunchArgument放在LaunchDescription之后

```python
return LaunchDescription([
        DeclareLaunchArgument(
            'test_name',
            default_value=test_name,
            description='Specifying whether or not to enable angle_compensate of scan data'),
		...,
    ])

```

DeclareLaunchArgument的参数列表为

- 参数名称
- 默认取值
- 参数的描述

# 使用举例

在任意一个自己的包下新建一个名为declare_test.launch的launch文件

```python
def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_name',
            default_value="test_name",
            description='Specifying whether or not to enable angle_compensate of scan data'),
    ])
```

编译之后，通过命令行查看launch的参数

```
ros2 launch cpp_pubsub declare_test.launch.py --show-args
```

将会看见

```
    'test_name':
        Specifying whether or not to enable angle_compensate of scan data
        (default: 'test_name')
```







