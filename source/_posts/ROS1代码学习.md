---
title: ros1代码学习
date: 2021-05-20 10:11:26
tags:
---

参考：[此处博客](https://blog.csdn.net/sru_alo/article/details/102893536?utm_medium=distribute.pc_relevant.none-task-blog-baidujs_baidulandingword-0&spm=1001.2101.3001.4242)

ROS Time表示的是ROS网络中的时间。

ROS网络中的时间是指，如果当时在非仿真环境里运行，那它就是当前的时间。但是假设去回放当时的情况，那就需要把当时的时间录下来。以控制为例，很多的数据处理需要知道当时某一个时刻发生了什么。Wall Time可以理解为墙上时间，墙上挂着的时间没有人改变的了，永远在往前走；**ROS Time可以被人为修改，你可以暂停它，可以加速，可以减速，但是Wall Time不可以。**

在开启一个Node之前，当把use_sim_time设置为true时，这个节点会从clock Topic获得时间。所以操作这个clock的发布者，可以实现一个让Node中得到ROS Time暂停、加速、减速的效果。同时下面这些方面都是跟Node透明的，所以非常适合离线的调试方式。当把ROSbag记下来以后重新play出来时，加两个横杠，--clock，它就会发布出这个消息。

## 1.1 获得当前时间

```
ros::Time::now();
ros::Time begin = ros::Time::now();
```