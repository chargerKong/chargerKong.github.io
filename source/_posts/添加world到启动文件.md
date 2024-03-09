---
title: 添加world到启动文件
date: 2021-04-24 10:23:20
tags: robot_simulation
---

# 添加world文件与model文件

打开robot_desciption文件夹

```
cd ~/dev_es/src/robot_description
```

新建两个文件夹

```
mkdir model
mkdir world
```

在world文件夹里添加warehouse.world文件

在model文件夹里添加small_warehouse文件夹

## 修改setup.py

添加data_files中的内容

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ...
        ('share/' + package_name, glob('world/*')),
        ('share/' + package_name + "/model/small_warehouse", glob('model/small_warehouse/*')),
    ],
```

data_files为一个列表，每一个元素为一个元祖，元祖里的内容分别为

- 编译后文件所在的位置，根目录为`~/dev_ws/install/robot_description`
- 编译文件的原位置，根目录为`~/dev_ws/src/robot_description`

在此，我们把整个model文件夹的内容（包括model文件夹）加入了编译后的share文件夹里面

## 配置world文件

打开包`robot_description`中launch文件夹中的launch文件复制粘贴为gazebo_lab_world.launch.py

打开文件，在最后的LaunchDescription中添加world参数

```python
import os
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

...

def generate_launch_description():
	...
    pkg_dir = get_package_share_directory('robot_description')
    world = os.path.join(pkg_dir, 'warehouse.world')
    ...
    return LaunchDescription([

        DeclareLaunchArgument(
            "world", default_value=world,
            description="world description file name"
        ),
        gazebo,
        robot_pub,
        spawn_entity,
    ])
```

`DeclareLaunchArgument`所配置的参数可以跨文件传递，可以通过LaunchConfiguration(name)来获取在这里设置的值。



#### 添加环境变量

因为我们的world文件需要加载我们的model文件，但是gazebo却不知道model在哪，因此需要添加我们的model文件夹的路径，打开gazebo_lab_world.launch.py

```python
...

def generate_launch_description():
   	...
    pkg_dir = get_package_share_directory('robot_description')
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, "model")
    ...
    
```

## 编译运行

```
cd ~/dev_ws
colcon build --packages-select robot_description --symlink-install
```

记得source一下bash

```
ros2 launch robot_description gazebo_lab_world.launch.py
```

![](添加world到启动文件/world.png)



# launch文件分析（gazebo）

### generate_launch_description

 launch文件启动会从这个函数开始运行

返回一个`LaunchDescription`，整个函数可以接收一个列表，列表内的元素可以为

- DeclareLaunchArgument(name, default_value="",description="")
- OpaqueFunction
- 一个Node



只要是这里定义过的, 再一次调用此函数，就需要

```
LaunchConfiguration('name')
```

才能获得一个`launch.launch_context.LaunchContext object`

通过

```python
def test(context, *args, **kwargs):
    print(LaunchConfiguration('test').perform(context))
    
LaunchDescription([
	DeclareLaunchArgument(
        "test1", default_value="it's a test",
        description="test value"
    )
    OpaqueFunction(function=test),
])
```

可以获得值



## gazebo launch分析

在安装的`gazebo_ros`包中，有三个重要的`launch`文件，可以通过如下命令查看

```
ls ~/dev_ws/src/gazebo_ros_pkgs/gazebo_ros/launch
```

分别是

```
gazebo.launch.py  gzclient.launch.py  gzserver.launch.py spawn_entity_demo.launch.py
```

### gazebo.launch.py查看

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server'))
        ),  

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),  
    ])
```

#### gzserver

函数直接返回了其他launch文件，一个是gzserver

```python
IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server'))
        ),
```

`IncludeLaunchDescription`用于包含另外一个launch文件，当我们再一次访问它的时候，就可以获得launch文件里面的内容。`PythonLaunchDescriptionSource`指明用于启动Python的launch文件。它的构造函数为

```python
class PythonLaunchDescriptionSource(LaunchDescriptionSource):
    """Encapsulation of a Python launch file, which can be loaded during launch."""

    def __init__(
        self,
        launch_file_path: SomeSubstitutionsType,
    ) -> None:
```

`launch_file_path`的类型为`SomeSubstitutionsType`，下面给出`SomeSubstitutionsType`的定义

```python
SomeSubstitutionsType = Union[
    Text,
    Substitution,
    Iterable[Union[Text, Substitution]],
]
```

其中`Union`类型：`Union[X, Y]`意味着要么是X要么是Y

因此，`SomeSubstitutionsType`的类型可以为下面三者其一

- 一个Text（就是str）
- 一个Substitution的实例
- 可迭代的一个对象，其中对象必须是一个Text或者Substitution

在这里是第三种，传入的是一个列表，第一个ThisLaunchFileDir()，此类继承于Substitution，第二个是一个Text.

下面的语句表示，如果有配置了参数server，则启动此launch文件

```
condition=IfCondition(LaunchConfiguration('server'))
```

#### gzclient

```
IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),  
```

这一段的内容和启动gzclient的内容一模一样

下面打开gzserver.launch.py一看究竟



### gzserver.launch.py查看

```python
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

from scripts import GazeboRosPaths


def generate_launch_description():
    cmd = [ 
        'gzserver',
        # Pass through arguments to gzserver
        LaunchConfiguration('world'), ' ',
        _boolean_command('version'), ' ',
        _boolean_command('verbose'), ' ',
        _boolean_command('lockstep'), ' ',
        _boolean_command('help'), ' ',
        _boolean_command('pause'), ' ',
        _arg_command('physics'), ' ', LaunchConfiguration('physics'), ' ',
        _arg_command('play'), ' ', LaunchConfiguration('play'), ' ',
        _boolean_command('record'), ' ',
        _arg_command('record_encoding'), ' ', LaunchConfiguration('record_encoding'), ' ',
        _arg_command('record_path'), ' ', LaunchConfiguration('record_path'), ' ',
        _arg_command('record_period'), ' ', LaunchConfiguration('record_period'), ' ',
        _arg_command('record_filter'), ' ', LaunchConfiguration('record_filter'), ' ',
        _arg_command('seed'), ' ', LaunchConfiguration('seed'), ' ',
        _arg_command('iters'), ' ', LaunchConfiguration('iters'), ' ',
        _boolean_command('minimal_comms'),
        _plugin_command('init'), ' ',
        _plugin_command('factory'), ' ',
        _plugin_command('force_system'), ' ',
        # Wait for (https://github.com/ros-simulation/gazebo_ros_pkgs/pull/941)
        # _plugin_command('force_system'), ' ',
        _arg_command('profile'), ' ', LaunchConfiguration('profile'),
        LaunchConfiguration('extra_gazebo_args'),
    ]   
                                                                                                                                                    
    model, plugin, media = GazeboRosPaths.get_paths()
    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }

    prefix = PythonExpression([
        '"gdb -ex run --args" if "true" == "',
        LaunchConfiguration('gdb'),
        '" else "valgrind" if "true" == "',
        LaunchConfiguration('valgrind'),
        '" else ""'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='',
            description='Specify world file name'
        ),
        DeclareLaunchArgument(
            'version', default_value='false',
            description='Set "true" to output version information.'
        ),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Set "true" to increase messages written to terminal.'
        ),
        DeclareLaunchArgument(
            'lockstep', default_value='false',
            description='Set "true" to respect update rates'
        ),
        DeclareLaunchArgument(
            'help', default_value='false',
            description='Set "true" to produce gzserver help message.'
        ),
        DeclareLaunchArgument(
            'pause', default_value='false',
            description='Set "true" to start the server in a paused state.'
        ),
        DeclareLaunchArgument(
            'physics', default_value='',
            description='Specify a physics engine (ode|bullet|dart|simbody).'
        ),
        DeclareLaunchArgument(
            'play', default_value='',
            description='Play the specified log file.'
        ),                                                                                                                                          
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Set "true" to record state data.'
        ),
        DeclareLaunchArgument(
            'record_encoding', default_value='',
            description='Specify compression encoding format for log data (zlib|bz2|txt).'
        ),
        DeclareLaunchArgument(
            'record_path', default_value='',
            description='Absolute path in which to store state data.'
        ),
        DeclareLaunchArgument(
            'record_period', default_value='',
            description='Specify recording period (seconds).'
        ),
        DeclareLaunchArgument(
            'record_filter', default_value='',
            description='Specify recording filter (supports wildcard and regular expression).'
        ),
        DeclareLaunchArgument(
            'seed', default_value='', description='Start with a given a random number seed.'
        ),
        DeclareLaunchArgument(
            'iters', default_value='', description='Specify number of iterations to simulate.'
        ),
        DeclareLaunchArgument(
            'minimal_comms', default_value='false',
            description='Set "true" to reduce TCP/IP traffic output.'
        ),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='Specify physics preset profile name from the options in the world file.'
        ),
        DeclareLaunchArgument(
            'extra_gazebo_args', default_value='',
            description='Extra arguments to be passed to Gazebo'
        ),

        # Specific to gazebo_ros
        DeclareLaunchArgument(
            'gdb', default_value='false',
            description='Set "true" to run gzserver with gdb'
        ),
        DeclareLaunchArgument(
            'valgrind', default_value='false',
            description='Set "true" to run gzserver with valgrind'
        ),
        DeclareLaunchArgument(
            'init', default_value='true',
            description='Set "false" not to load "libgazebo_ros_init.so"'
        ),
        DeclareLaunchArgument(
            'factory', default_value='true',
            description='Set "false" not to load "libgazebo_ros_factory.so"'
        ),
        DeclareLaunchArgument(
            'force_system', default_value='true',
            description='Set "false" not to load "libgazebo_ros_force_system.so"'
        ),
        DeclareLaunchArgument(
            'server_required', default_value='false',
            description='Set "true" to shut down launch script when server is terminated'
        ),
        ExecuteProcess(
            cmd=cmd,
            output='screen',
            additional_env=env,
            shell=True,
            prefix=prefix,
            on_exit=Shutdown(),
            condition=IfCondition(LaunchConfiguration('server_required')),
        ),

        # Execute node with default on_exit if the node is not required
        ExecuteProcess(
            cmd=cmd,
            output='screen',
            additional_env=env,
            shell=True,
            prefix=prefix,
            condition=UnlessCondition(LaunchConfiguration('server_required')),
        ),
    ])
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


# Add string commands if not empty
def _arg_command(arg):
    cmd = ['"--', arg, '" if "" != "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


# Add gazebo_ros plugins if true
def _plugin_command(arg):
    cmd = ['"-s libgazebo_ros_', arg, '.so" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
```

此文件较长

在`generate_launch_description`函数的开头，定义了cmd命令，即模拟终端的命令。它等价于

```
gzserver xxx.world --version ...
```

`LaunchConfiguration("world")`表示获取相关变量

```python
    model, plugin, media = GazeboRosPaths.get_paths()
    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }
    prefix = PythonExpression([
        '"gdb -ex run --args" if "true" == "',
        LaunchConfiguration('gdb'),
        '" else "valgrind" if "true" == "',
        LaunchConfiguration('valgrind'),
        '" else ""'
    ])
```

这一段表示设置`GAZEBO`的相关文件路径的环境变量，以及定义prefix

最后是设置相关的变量，我们看到第一个就是world参数，默认值为空。我们也可以通过其他文件调用此文件来设置world参数

最后执行了之前所定义的cmd语句

### gzclient.launch.py查看

内容结构和gzclient.launch.py一模一样