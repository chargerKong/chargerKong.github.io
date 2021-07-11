---
title: GPU在docker中使用
date: 2021-07-02 16:24:35
tags: GPU, docker
---

首先自己的机器上需要有GPU，在附加驱动中可以查看

## 安装显卡驱动

### 附加驱动安装NAVIDIA驱动

点开Ubuntu中自带的Software & update （软件和更新），选择下面的additional Drivers（附加驱动），即可查看自己的显卡型号

![](/home/kong/repo/blog/blog/source/_posts/GPU在docker中使用/2021-07-02 11-34-52屏幕截图.png)

这里可以看见，我的电脑上有一块GeForce MX250的显卡。现在默认使用的是系统自带的，我可以选择上面其他几个NVIDIA驱动。我这里选择的是nvidia-driver-460，两个460都可以，具体对应自己的GPU是多少可以去[NVIDIA官网](https://www.nvidia.com/Download/index.aspx#)查看最合适的驱动。

首先选好对应自己的显卡和操作系统，我这里是GeForce MX250和Linux，点击确定会有推荐的驱动

![](/home/kong/repo/blog/blog/source/_posts/GPU在docker中使用/2021-07-02 14-26-11屏幕截图.png)

相应上面推荐的是460.84

![](/home/kong/repo/blog/blog/source/_posts/GPU在docker中使用/2021-07-02 14-28-42屏幕截图.png)



### 使用命令安装NAVIDIA驱动

```
sudo apt install nvidia-driver-460
```

安装完成后需要重新启动系统才能生效，重启系统后通过 **`nvidia-smi`** 命令可以查看有没有应用在使用NAVIDIA显卡驱动，如果存在这样的应用则表示安装成功：

```
kong@kong-KLV-WX9:~/Templates$ nvidia-smi 
Fri Jul  2 14:31:35 2021       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 460.84       Driver Version: 460.84       CUDA Version: 11.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  GeForce MX250       Off  | 00000000:01:00.0 Off |                  N/A |
| N/A   59C    P0    N/A /  N/A |   1355MiB /  2002MiB |      6%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|    0   N/A  N/A       961      G   /usr/lib/xorg/Xorg                714MiB |
|    0   N/A  N/A      1919      G   /usr/bin/gnome-shell               93MiB |
|    0   N/A  N/A      7925      G   ...AAAAAAAAA= --shared-files       82MiB |
|    0   N/A  N/A     29518      G   gzclient                          252MiB |
+-----------------------------------------------------------------------------+
```



## docker使用GPU总结

（注：下面只讨论docker19使用gpu，低于19的请移步其他博客，或更新19后再参考本文）

### 背景及基本介绍

  现由于项目要使用GPU，所以需要docker支持GPU，在docker19以前都需要单独下载nvidia-docker1或nvidia-docker2来启动容器，自从升级了docker19后跑需要gpu的docker只需要加个参数–gpus all 即可(表示使用所有的gpu，如果要使用2个gpu：–gpus 2，也可直接指定哪几个卡：–gpus ‘“device=1,2”’，后面有详细介绍)。
  接着需要安装nvidia驱动，这需要根据自己显卡安装相应的驱动，网上有很多类似教程，此处不再介绍，推荐一个链接：讲的很详细
  不要以为这样就可以安心的使用gpu了，你的镜像还必须要有cuda才行，这也很简单，去dockerhub上找到和自己tensorflow相对应的cuda版本的镜像，再基于该镜像生成自己的镜像就可以轻松使用gpu了。这里需要额外多说一句，如果你的docker 本身就基于了某个镜像（例如基于本公司仓库的镜像），docker是不允许from两个镜像的，要想实现基于两个或多个，只能基于其中一个，其他的镜像通过各镜像的Dockerfile拼到新的Dockerfile上，但更多的镜像是没有Dockerfile的，可以通过docker history查看某镜像的生成过程，其实就是Dockerfile，nvidia/cuda官网本身就有Dockerfile，也可直接参考。

### 安装toolkit

关于配置docker19使用gpu，其实只用装官方提供的toolkit即可，把github上的搬下来：

```
# Add the package repositories
$ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
$ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
$ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

$ sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
$ sudo systemctl restart docker
```

重启！！！！！！！！

### 测试安装是否成功 

 经过以上大部分linux系统的docker toolkit应该都能安装成功，如不能安装成功，可参考github[官网](https://github.com/NVIDIA/nvidia-docker)，查看是否安装成功：
(1) 查看–gpus 参数是否安装成功：

```
$ docker run --help | grep -i gpus
      --gpus gpu-request               GPU devices to add to the container ('all' to pass all GPUs)
```

(2) 运行nvidia官网提供的镜像，并输入nvidia-smi命令，查看nvidia界面是否能够启动：

```
docker run --gpus all nvidia/cuda:9.0-base nvidia-smi
```

**运行gpu的容器**

```
# 使用所有GPU
$ docker run --gpus all nvidia/cuda:9.0-base nvidia-smi

# 使用两个GPU
$ docker run --gpus 2 nvidia/cuda:9.0-base nvidia-smi

# 指定GPU运行
$ docker run --gpus '"device=1,2"' nvidia/cuda:9.0-base nvidia-smi
$ docker run --gpus '"device=UUID-ABCDEF,1"' nvidia/cuda:9.0-base nvidia-smi

```



## 下载docker 镜像

### 镜像介绍

```
figoowen2003/ros1-gazebo-desktop-vnc:noetic
```

此镜像为一个包含了vnc可视化的镜像，里面基础包含了ROS1，以及Gazebo。



### 镜像使用

```
sudo docker run  -p 6080:80  -v /dev/shm:/dev/shm --gpus all figoowen2003/ros1-gazebo-desktop-vnc:noetic
```

然后打开浏览器输入

```
127.0.0.1:6080
```

即可利用vnc进入docker。



### 测试GPU是否能使用

打开内置的火狐浏览器，然后打开终端，输入`nvidia-smi`，可以看见，火狐浏览器已经运行在GPU里面

![](/home/kong/repo/blog/blog/source/_posts/GPU在docker中使用/2021-07-02 15-15-24屏幕截图.png)



参考：

1. https://blog.csdn.net/weixin_43975924/article/details/104046790

2. https://blog.csdn.net/Thanlon/article/details/106125738