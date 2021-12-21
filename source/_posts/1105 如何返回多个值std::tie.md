---
title: std::tie 如何返回多个值
date: 2021-11-05 18:38:29
tags: c++
---

假设有两个变量，需要赋值

```
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
```

我们可以一次性通过std::tie 进行赋值

```c++
std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
```

其中LoadOption的定义和返回值应该为

```c++
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    ...
    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
```

