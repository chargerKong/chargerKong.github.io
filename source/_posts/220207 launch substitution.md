---
title: 在launch中使用substitutions
date: 2022-02-06 12:59:22
tags: [ros2, launch]
---

# 背景

Launch文件用于启动节点，服务以及执行进程。这一些行为会涉及到参数，不同的参数会有不同的行为。在描述可重用的启动文件时，可以在参数中使用替换以提供更大的灵活性。Substitutions是仅在执行启动描述期间计算的变量，可用于获取特定信息，如启动配置、环境变量，或用于计算任意Python表达式。

# 前提准备

你需要创建一个类型为`ament_cmake`的`launch_tutorial`包

# 使用substitutions

## 1. Parent launch 文件

首先我们会创建一个launch文件，他会通过适当的参数来调用其他文件，下面我们在launch文件夹下创建example_main.launch.py

```python
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'launch',
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

在example_main.launch.py文件中，`FindPackageShare` substitution 用于寻找launch_tutorial包的share文件夹，`PathJoinSubstitution`则用于组合目录

```python
PathJoinSubstitution([
    FindPackageShare('launch_tutorial'),
    'launch',
    'example_substitutions.launch.py'
])
```

 `launch_arguments` 字典被传入到`IncludeLaunchDescription`中， `TextSubstitution` substitution 用于定义new_background_r 的值

```python
launch_arguments={
    'turtlesim_ns': 'turtlesim2',
    'use_provided_red': 'True',
    'new_background_r': TextSubstitution(text=str(colors['background_r']))
}.items()
```

## 2 Substitutions example launch file

在同一个文件下创建文件 `example_substitutions.launch.py`

```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

在文件example_substitutions.launch.py中，定义了`turtlesim_ns`, `use_provided_red`, 和 `new_background_r` 的launch configurations 。它们用于在上述变量中存储launch参数的值，并将它们传递给所需的action。这些LaunchConfiguration substitutions允许我们在launch的任何部分来获得launch参数的值。

DeclareLaunchArgument用于定义可以从上面的启动文件或控制台传递的启动参数。

```python
turtlesim_ns = LaunchConfiguration('turtlesim_ns')
use_provided_red = LaunchConfiguration('use_provided_red')
new_background_r = LaunchConfiguration('new_background_r')

turtlesim_ns_launch_arg = DeclareLaunchArgument(
    'turtlesim_ns',
    default_value='turtlesim1'
)
use_provided_red_launch_arg = DeclareLaunchArgument(
    'use_provided_red',
    default_value='False'
)
new_background_r_launch_arg = DeclareLaunchArgument(
    'new_background_r',
    default_value='200'
)
```

节点的turtlesim_node的`namespace`被指定为`turtlesim_ns`，此substitutions之前已定义过

```python
turtlesim_node = Node(
    package='turtlesim',
    namespace=turtlesim_ns,
    executable='turtlesim_node',
    name='sim'
)
```

# Launching example

```
ros2 launch launch_tutorial example_main.launch.py
```

# 修改launch参数

在修改之前，我们需要知道可以修改什么参数

```
ros2 launch launch_tutorial example_substitutions.launch.py --show-args
```

这可以返回有什么参数，并且有什么默认值

```
Arguments (pass arguments as '<name>:=<value>'):

    'turtlesim_ns':
        no description given
        (default: 'turtlesim1')

    'use_provided_red':
        no description given
        (default: 'False')

    'new_background_r':
        no description given
        (default: '200')
```

现在我们可以修改参数

```
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

