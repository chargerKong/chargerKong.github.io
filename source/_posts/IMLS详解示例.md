---
title: IMLS-ICP
date: 2021-07-01 18:38:29
tags:
---

## IMLS-ICP用途

IMLS-ICP用于SLAM的前端匹配，通过处理采集到的雷达数据来估计相机的移动位姿。

## 示例

下面我们用如下两帧的2D雷达数据做一个示例讲解，假设我们采集到的2D雷达数据内容如下所示





3. 完全卸载NAVIDIA驱动
如果想卸载NAVIDIA驱动，使用附加驱动的方式只能切换驱动，但卸载不了驱动，只能通过命令的方式卸载：

sudo apt-get --purge remove nvidia*
sudo apt-get --purge remove "*nvidia*"
sudo apt-get --purge remove "*cublas*" "cuda*"
sudo apt autoremove
1
2
3
4
4. 其它命令
# 查看NAVIDIA的型号
thanlon@thanlon:~$ lspci |grep -i nvidia
02:00.0 3D controller: NVIDIA Corporation GP108M [GeForce MX150] (rev a1)
# 查看NVIDIA驱动版本
sudo dpkg --list | grep nvidia-*
# 检查适合系统的NAVIDIA版本
thanlon@thanlon:~$ nvidia-detector 
nvidia-driver-440

