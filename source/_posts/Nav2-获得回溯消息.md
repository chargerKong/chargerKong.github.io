---
title: Navigation2-获得回溯消息
date: 2021-04-29 16:24:35
tags: navigation2
---

GDB是Unix系统上最流行的C ++调试器。 它可用于确定崩溃的原因并跟踪线程。 它也可以用于在代码中添加断点，以检查内存中的值，即软件中的特定点。

对于所有使用C/ c++的软件开发人员来说，使用GDB是一项关键技能。许多ide都内置了某种调试器或分析器，但是有了ROS，就很少有ide可供选择了。因此，理解如何使用这些可用的原始工具，而不是依赖IDE来提供它们，这一点很重要。此外，理解这些工具是C/ c++开发的一项基本技能，如果你不再能够访问IDE，或者正在通过远程资产的ssh会话进行动态开发，那么无法使用IDE调试将会是一个比较麻烦的问题了。

幸运的是在掌握了基础知识之后，使用GDB会非常简单。第一步是将`-g`添加到要分析/调试的ROS包的编译器标志中。 此标志生成GDB和valgrind可以读取的调试符号，以告诉您项目中特定的代码行失败以及为什么，如果未设置此标志，则仍然可以获取回溯记录，但不会提供失败的行号。 确保在调试完成之后后删除此标志，因为这会降低运行时的性能。

将以下行添加到项目的CMakeLists.txt中即可解决问题。 如果您的项目已经具有add_compile_options（），则可以简单地在其中添加-g。 然后只需使用此软件包`colcon build --packages-select <package-name>`重建您的工作区。 编译可能需要比通常更长的时间。

```
add_compile_options(-g)
```

现在可以调试代码了!如果这是一个非ros项目，此时您可能会执行如下操作。在这里，我们启动了一个GDB会话，并告诉我们的程序立即运行。一旦程序崩溃，它将返回一个由(gdb)表示的gdb会话提示。在这个提示符下，您可以访问您感兴趣的信息。然而，由于这是一个包含大量节点配置和其他事情的ROS项目，对于初学者或不喜欢大量命令行工作和不理解文件系统的人来说，这不是一个很好的选择。

```
gdb ex run --args /path/to/exe/program
```

下面是三种可以运行基于ROS2系统的方式

## From a Node

与我们的非ros示例一样，我们需要在启动我们的ROS2节点之前设置一个GDB会话。虽然我们可以通过命令行通过对ROS 2文件系统的一些了解来设置它，但是我们可以使用Open Robotics为我们提供的launch `-prefix`选项。

`--prefix`将在我们的ros2命令之前执行一些代码，允许我们插入一些信息。如果您试图执行`gdb ex run --args ros2 run`，作为我们在初步示例中的模拟，您会发现它无法找到`ros2`命令。如果你再聪明一点，你会发现试图source你的工作空间也会因为类似的原因而失败。

我们可以使用`--prefix`代替查找可执行文件的安装路径并将其全部输入出来。这允许我们使用您习惯的相同的ros2运行语法，而不必担心一些GDB细节。

```
ros2 run --prefix 'gdb -ex run --args' <pkg> <node> --all-other-launch arguments
```

与前面一样，这个prefix将启动一个GDB会话，并使用所有附加命令行参数运行您请求的节点。您现在应该已经运行了您的节点，并且应该正在进行一些调试打印。

一旦服务器崩溃，您将看到如下提示。此时，您可以获得一个回溯。

```
(gdb)
```

在这个会话中，输入`backtrace`他会返回错误的回溯，

```
(gdb) backtrace
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
#1  0x00007ffff79cc859 in __GI_abort () at abort.c:79
#2  0x00007ffff7c52951 in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#3  0x00007ffff7c5e47c in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#4  0x00007ffff7c5e4e7 in std::terminate() () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#5  0x00007ffff7c5e799 in __cxa_throw () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#6  0x00007ffff7c553eb in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#7  0x000055555555936c in std::vector<int, std::allocator<int> >::_M_range_check (
    this=0x5555555cfdb0, __n=100) at /usr/include/c++/9/bits/stl_vector.h:1070
#8  0x0000555555558e1d in std::vector<int, std::allocator<int> >::at (this=0x5555555cfdb0,
    __n=100) at /usr/include/c++/9/bits/stl_vector.h:1091
#9  0x000055555555828b in GDBTester::VectorCrash (this=0x5555555cfb40)
    at /home/steve/Documents/nav2_ws/src/gdb_test_pkg/src/gdb_test_node.cpp:44
#10 0x0000555555559cfc in main (argc=1, argv=0x7fffffffc108)
    at /home/steve/Documents/nav2_ws/src/gdb_test_pkg/src/main.cpp:25
```

此回溯应该从下往上读

- 正在main函数中，25行调用了函数VectorCrash
- 在VectorCrash中，44行，在输入100的时候Vector的at方法崩溃
- ...

这些回溯需要花费一些时间来习惯阅读，但是通常，从底部开始，然后沿着堆栈向上，直到您看到它崩溃的那一行为止。 然后，您可以推断出其崩溃的原因。 完成GDB的操作后，键入quit，它将退出会话并杀死仍在运行的所有进程。 它可能会询问您是否要结束一些线程，输入yes



## From a Launch File

与我们的非ros示例一样，我们需要在启动我们的ROS2启动文件之前设置一个GDB会话。虽然我们可以通过命令行进行设置，但是我们可以使用与在ros2运行节点示例中相同的机制，现在使用一个启动文件。

在launch文件中，找到你想要debugging的节点。在此，我们假设你的launch文件只包含了一个节点，在launch_ros包中使用的Node函数将接受一个字段`prefix`，该`prefix`包含一个前缀参数列表。我们只需要在例子中一处就可以在这里插入GDB片段，即使用`xterm`。`xterm`将弹出一个新的终端窗口来显示并与GDB交互。我们这样做是因为在启动文件上处理stdin的问题. 下面是一个debugging一个SLAM工具箱的例子

与以前一样，这个prefix将启动一个GDB会话，现在是在`xterm`中，并运行您请求的启动文件，其中定义了所有附加的启动参数。

一旦服务器崩溃，您将在`xterm`会话中看到如下提示。此时，您可以获得一个回溯。

```
(gdb)
```

和上面的一样，只要输入backtrace就会有一个回溯的信息

这些回溯需要花费一些时间来习惯阅读，但是通常，从底部开始，然后沿着堆栈向上，直到您看到它崩溃的那一行为止。 然后，您可以推断出其崩溃的原因。 完成GDB的操作后，键入quit，它将退出会话并杀死仍在运行的所有进程。 它可能会询问您是否要结束一些线程，输入yes



## From Nav2 Bringup

使用多个节点的启动文件略有不同，因此您可以与您的GDB会话交互，而不会被同一终端中的其他日志所困扰。出于这个原因，在处理更大的启动文件时，最好选择您感兴趣的特定服务器并单独启动它。

因此，对于这种情况，当您看到想要调查的崩溃时，最好将此服务器与其他服务器分开。

如果你的服务器正在从一个嵌套的启动文件(例如，一个包含的启动文件的启动文件)启动，你可能要做以下事情:

- 注释掉父启动文件中的启动文件内容
- 用调试符号的`-g`标志重新编译感兴趣的包
- 在终端中启动父启动文件
- 按照“从启动文件”中的说明在另一个终端中启动服务器的启动文件。

或者，如果你的服务器直接在这些文件中启动(例如，你看到一个Node, LifecycleNode，或在一个ComponentContainer内)，你将需要将它与其他分开:

- 注释掉父启动文件中的启动文件内容
- 用调试符号的`-g`标志重新编译感兴趣的包
- 在终端中启动父启动文件
- 按照“从启动文件”中的说明在另一个终端中启动服务器的启动文件。

一旦出现程序崩溃也会出现如下情况

```
(gdb)
```

和上面的一样，只要输入backtrace就会有一个回溯的信息

这些回溯需要花费一些时间来习惯阅读，但是通常，从底部开始，然后沿着堆栈向上，直到您看到它崩溃的那一行为止。 然后，您可以推断出其崩溃的原因。 完成GDB的操作后，键入quit，它将退出会话并杀死仍在运行的所有进程。 它可能会询问您是否要结束一些线程，输入yes