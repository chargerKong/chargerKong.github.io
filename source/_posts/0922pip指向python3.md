---
title: pip指向Python3问题
date: 2021-09-18 15:59:23
tags: Python
---

当下载Python包的时候，无论是pip还是pip3，他的版本都是Python3的

```
pip -V
pip3 -V
```

**一，改一下软链接**

改一下目录为`/usr/bin`下的pip，将其指向`/usr/local/bin`下的pip，如果`/usr/local/bin`下的pip正确的画

```
ln -s /usr/local/bin/pip /usr/bin/pip
```



**二，解决方法: 重新下载pip**

打开此网页https://bootstrap.pypa.io/pip/2.7/get-pip.py，复制下载到本地

```
sudo python get-pip.py  # 运行安装脚本
sudo apt-get install python3-pip
```

此时发现使用

```
pip2 -V
```

则为Python2的pip