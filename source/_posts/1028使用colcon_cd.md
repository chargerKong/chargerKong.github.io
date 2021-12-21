---
title: 使用colcon_cd
date: 2021-10-28 09:59:22
tags: ros2
---

```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
```

```
echo "export _colcon_cd_root=~/dev_ws" >> ~/.bashrc
```

以上的内容即把dev_ws作为根目录，之后的colcon_cd都从他的src下面目录寻找

