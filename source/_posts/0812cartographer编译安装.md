---
title: 编译Cartographer的几种方法
date: 2021-08-12 18:38:29
tags: 
---

# 编译Cartographer的几种方法

## 第一：

可以先安装Cartographer到电脑上，先参考Cartographer的官方安装参考

然后下载Cartographer_ros作为一个软件包catkin_make即可。这种安装方式可以直接修改源码无需再编译了



# 第二：

可以把两个代码放到同一个src下面，然后catkin_make_isolated --install --use-ninja。就是官方给出的安装方法

这个安装方法每一次修改源码都需要再一次编译

