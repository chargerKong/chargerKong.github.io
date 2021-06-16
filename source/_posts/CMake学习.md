---
title: CMake实践
date: 2021-05-05 14:30:29
tags: 
---

## Project（name）

声明一个project的名字，这个指令隐式的定义了两个 cmake 变量，`${PROJECT_BINARY_DIR}` 以及`${PROJECT_SOURCE_DIR}`



## add_subdirectory

```
----CMakeLists.txt
----src
	|
	----main.cpp
	----CMakeLists.txt
```

在外层的CMakeLists.txt

```
  1 project(hello)                                                                                                  
  2 cmake_minimum_required(VERSION 3.16)
  3 add_subdirectory(src bin)
```

这里告诉cmake，有一个子目录

在目录中的CMakeLIsts.txt

```
  1 add_executable(slam main.cpp)                                                                                   
```



和src同级的目录下建立一个目录build。进入

```
cmake ..
make
```

可以看见编译之后会**产生一个新的文件加，名字为bin**，**并且可执行文件slam在bin文件下**。在add_subdirectory中倘若不指定第二个参数，则:默认编译生成 的目录名字为src。



## 换个地方保存目标二进制可执行文件

我们可以通过 SET 指令重新定义 EXECUTABLE_OUTPUT_PATH 和 LIBRARY_OUTPUT_PATH 变量来指定最终的目标二进制的位置(指最终生成的 slam 或者最终的共享库,不包含编译生成的中间文件)

```
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
```

​	上面两个指令分别定义了:
可执行二进制的输出路径为 build/bin 和库的输出路径为 build/lib

问题是,我应该把这两条指令写在工程的 CMakeLists.txt 还是 src 目录下的CMakeLists.txt,把握一个简单的原则,在哪里 ADD_EXECUTABLE 或 ADD_LIBRARY,
如果需要改变目标存放路径,就在哪里加入上述的定义，因此是在SRC目录下的CMakeLists下添加。



## 安装

安装的需要有两种,一种是从代码编译后直接 make install 安装,一种是打包时的指定目录安装

可以通过命令行

```
sudo make install
```

将其安装到`/usr/bin`中

稍微复杂一些的就需要用到`CMAKE_INSTALL_PREFIX。`一个常见的用法

```
cmake -DCMAKE_INSTALL_PREFIX=/usr
```

#### 目标文件的安装

在CMakeLists里， 填写规则为

```
INSTALL(TARGETS targets...
[[ARCHIVE|LIBRARY|RUNTIME]
	[DESTINATION <dir>]
	[PERMISSIONS permissions...]
	[CONFIGURATIONS
[Debug|Release|...]]
	[COMPONENT <component>]
	[OPTIONAL]
	] [...])
```

参数中的 TARGETS 后面跟的就是我们通过 ADD_EXECUTABLE 或者 ADD_LIBRARY 定义的目标文件, 可能是可执行二进制、动态库、静态库

目标类型为三种

- ARCHIVE 特指静态库
- LIBRARY 特指动态库
- UNTIME特指可执行目标二进制。

DESTINATION 定义了安装的路径,如果路径以/开头,那么指的是绝对路径,这时候CMAKE_INSTALL_PREFIX 其实就无效了。如果你希望使用 CMAKE_INSTALL_PREFIX 来定义安装路径,就要写成相对路径,即不要以/开头,那么安装后的路径就是${CMAKE_INSTALL_PREFIX}/<DESTINATION 定义的路径>



举个简单的例子:

```
INSTALL(TARGETS myrun mylib mystaticlib
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION libstatic
)
```

上面的例子会将:

- 可执行二进制 myrun 安装到`${CMAKE_INSTALL_PREFIX}/bin` 
- 动态库 mylib 安装到`${CMAKE_INSTALL_PREFIX}/lib` 目录
- 静态库 mystaticlib 安装到`${CMAKE_INSTALL_PREFIX}/libstatic `目录



#### 普通文件的安装

```
INSTALL(FILES files... DESTINATION <dir>
    [PERMISSIONS permissions...]
    [CONFIGURATIONS [Debug|Release|...]]
    [COMPONENT <component>]
    [RENAME <name>] [OPTIONAL])
```

可用于安装一般文件,并可以指定访问权限,文件名是此指令所在路径下的相对路径。如果默认不定义权限 PERMISSIONS,安装后的权限为644

#### 非目标文件的可执行程序安装(比如脚本之类)

```
INSTALL(PROGRAMS files... DESTINATION <dir>
    [PERMISSIONS permissions...]
    [CONFIGURATIONS [Debug|Release|...]]
    [COMPONENT <component>]
    [RENAME <name>] [OPTIONAL])
```

默认权限是755

#### 目录的安装

```
INSTALL(DIRECTORY dirs... DESTINATION <dir>
    [FILE_PERMISSIONS permissions...]
    [DIRECTORY_PERMISSIONS permissions...]
    [USE_SOURCE_PERMISSIONS]
    [CONFIGURATIONS [Debug|Release|...]]
    [COMPONENT <component>]
    [[PATTERN <pattern> | REGEX <regex>]
    [EXCLUDE] [PERMISSIONS permissions...]] [...])
```

DIRECTORY 后面连接的是所在 Source 目录的相对路径,但务必注意: abc/ 会在目标丼下再建立一个abc文件夹，但是abc就不会在建立

PATTERN 用于使用正则表达式进行过滤

PERMISSIONS 用于指定 PATTERN 过滤后的文件权限



###  安装举例

#### 安装README

在src同级目录下，添加文件COPYRIGHT和README，打开主工程文件 CMakelists.txt，加入以下指令

```
INSTALL(FILES COPYRIGHT README DESTINATION share/doc/cmake/t2)
```

这句话的意识为：安装COPYRIGHT 和README 两个文件到 <CMAKE_INSTALL_PREFIX>/share/doc/cmake/t2目录之下

前缀目录需要在cmake的时候指定，例如

```
cmake -DCMAKE_INSTALL_PREFIX=/home/kong
```

最后sudo make install 即可



#### 安装runhello.sh

```
INSTALL(PROGRAMS runhello.sh DESTINATION bin)
```



#### 安装 doc 中的 hello.txt

首先在主工程中添加add_subdirectory(doc)

然后打开doc文件夹，新建CMakeLIsts.txt

键入

```
INSTALL(FILES hello.txt DESTINATION share)
```



另外一种方式可以直接安装文件夹

```
INSTALL(DIRECTORY doc/ DESTINATION share/doc/cmake/t2)
```



### 默认安装路径

如果没有指定安装路径，那么默认安装的路径是`/usr/local`

## 静态库和动态库构建

```
 ----CMakeLists.txt
 ----lib
      |    
 	----hello.h
 	----hello.cpp
 	----CMakeLists.txt
```

主工程下的CMakeLists.txt

```
PROJECT(HELLOLIB)
ADD_SUBDIRECTORY(lib)
```

lib目录下的CMakeLists.txt

```
SET(LIBHELLO_SRC hello.cpp)
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})
```

编译之后会在build文件夹下，产生libhello.so文件

### ADD_LIBRARY

```
ADD_LIBRARY(libname [SHARED|STATIC|MODULE] [EXCLUDE_FROM_ALL] source1 source2 ... sourceN)
```

你不需要写全 libhello.so,只需要填写 hello 即可,cmake 系统会自动为你生成libhello.X

类型有三种:
SHARED,动态库
STATIC,静态库
MODULE,在使用 dyld 的系统有效,如果不支持 dyld,则被当作 SHARED 对待。

EXCLUDE_FROM_ALL 参数的意思是这个库不会被默认构建,除非有其他的组件依赖或者手工构建



**注意！**

如果想要名字相同的和静态库和动态库，则不能只通过add_library来实现，因为target的名字，libname是唯一的

此时需要引入其他的一个指令：

```
SET_TARGET_PROPERTIES(target1 target2 ...
    PROPERTIES prop1 value1
    prop2 value2 ...)
```

修改lib下的CMakeLists.txt，修改了静态文件的名字，并且通过SET_TARGET_PROPERTIES来修改名字

```
SET(LIBHELLO_SRC hello.cpp)                                                                                                                         
ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})
SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME "hello")
```



### 动态库版本

一般的动态库是跟着版本号的

```
libhello.so.1.2
libhello.so ->libhello.so.1
libhello.so.1->libhello.so.1.2
```

为了实现动态库版本号,我们仍然需要使用 SET_TARGET_PROPERTIES 指令。

```
SET_TARGET_PROPERTIES(hello PROPERTIES VERSION 1.2 SOVERSION 1)
```

添加到lib/CmakeLists.txt中，重新编译可以看见

```
在 build/lib 目录会生成:
libhello.so.1.2
libhello.so.1->libhello.so.1.2
libhello.so ->libhello.so.1
```

### 安装共享库和头文件

我们需要把.a、.so文件和h文件安装到系统目录才可以让别人使用

```
INSTALL(TARGETS hello hello_static
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)
```

注意,静态库要使用 ARCHIVE 关键字



以及h文件

```
INSTALL(FILES hello.h DESTINATION include/hello)
```

### 使用共享库

#### 找到头文件 INCLUDE_DIRECTORIES

完整语法

```
INCLUDE_DIRECTORIES([AFTER|BEFORE] [SYSTEM] dir1 dir2 ...)
```

这条指令可以用来向工程添加多个特定的头文件搜索路径,路径之间用空格分割,如果路径中包含了空格,可以使用双引号将它括起来

默认的行为是追加到当前的头文件搜索路径的后面,你可以通过两种方式来进行控制搜索路径添加的方式:

通过配置

```
AFTER|BEFORE
```

在src/CMakeLists.txt文件中添加头文件导入目录

```
INCLUDE_DIRECTORIES(/usr/include/hello)
```

#### 为target添加共享库

LINK_DIRECTORIES(directory1 directory2 ...)

整个可以添加非标准的共享库文件夹





现在只是找到了头文件，却不知道真正的函数定义在哪

我们现在需要完成的任务是将目标文件链接到 libhello,这里我们需要引入两个新的指令

`TARGET_LINK_LIBRARIES(main hello)`
也可以写成
`TARGET_LINK_LIBRARIES(main libhello.so)`