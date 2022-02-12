---
title: gazebo添加使用world
date: 2021-12-02 18:38:29
tags: [gazebo, ros2]
---

# 如何在终端使用gazebo的world文件

打开终端，到相应的world目录下，直接运行

```
gazebo your_world_file.model
```

**有的时候会加载很久：**一般情况下是此world里面需要的model文件，没有指定到gazebo_model_path中

打开相应的world文件，搜索`model://`，在我的文件中，有一项内容是

```xml
    <model name="turtlebot3_world">
      <static>1</static>
      <include>
        <uri>model://turtlebot3_world</uri>
      </include>
    </model>
```

我需要确定的是turtlebot3_world已经在Gazebo_model_path 之下

全局搜索

```
sudo find / -name turtlebot3_world
```

找到相应的目录，将其添加

```
export GAZEBO_MODEL_PATH=path/to/your/model
```

