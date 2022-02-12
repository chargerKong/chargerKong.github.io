---
title: Groot安装
date: 2021-12-03 18:38:29
tags: [gazebo, ros2]
---

# 安装Groot

![](2021-12-03 18-35-42屏幕截图.png)

![](1203 安装Groot/2021-12-03 18-35-42屏幕截图.png)

# 安装BehaviorTree.CPP

```
 sudo apt-get install libzmq3-dev libboost-dev
 git clone https://github.com/BehaviorTree/BehaviorTree.CPP
 cd BehaviorTree.CPP;mkdir build;cd build;
 cmake ..
 make
 sudo make install
```

# 安装Groot

```
sudo apt install qtbase5-dev libqt5svg5-dev
git clone https://github.com/BehaviorTree/Groot
cd Groot;mkdir build;cd build;
cmake ..
make 
sudo make install
```

# 运行Groot

```
Groot
```

如果报错

```
so: cannot open shared object file: No such file or directory
```

添加so的链接路径

```
sudo vi /etc/ld.so.conf
```

添加一行如下，如下所示

```
  1 include /etc/ld.so.conf.d/*.conf
  2 /usr/local/lib
```

保存退出之后，运行ldconfig，再一次运行

```
Groot
```

