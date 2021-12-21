---
title: node lifecycle design
date: 2021-11-22 18:38:29
tags: ros2
---

节点生命周期的管理允许对ROS系统的状态进行更大的控制。它将允许roslaunch在允许任何组件开始执行其行为之前确保所有组件已经被正确实例化。它还允许在线重启或替换节点。

本文档最重要的概念是，管理节点表示一个已知的接口，根据一个已知的生命周期状态机执行，否则可以视为一个黑盒。这允许节点开发人员自由决定如何提供管理生命周期功能，同时还确保为管理节点而创建的任何工具都可以与任何兼容的节点一起工作。

![The proposed node life cycle state machine](http://design.ros2.org/img/node_lifecycle/life_cycle_sm.png)

有四个主要的状态：

- `Unconfigured`
- `Inactive`
- `Active`
- `Finalized`

想要转换出主要的状态，需要外部的监控进程，除了在Active状态触发错误之外。

还有6个转换状态，它们是请求转换期间的中间状态。

- `Configuring`
- `CleaningUp`
- `ShuttingDown`
- `Activating`
- `Deactivating`
- `ErrorProcessing`

在转换状态中，将执行逻辑以确定转换是否成功。成功或失败应通过生命周期管理界面传达给生命周期管理软件。

有7种过渡暴露在监督过程中，它们是:

- `create`
- `configure`
- `cleanup`
- `activate`
- `deactivate`
- `shutdown`
- `destroy`

### Primary State: Unconfigured

生命周期状态，表示结点才刚刚被实例化。他也有可能是一个被返回的结点，如果出错。**这个状态应该是不存储其他状态？**

### Primary State: Inactive

这个状态下的结点表示，该节点还没有做任何的处理

这个状态下，允许重新进行配置

在这种状态下，节点将不会收到任何用于读取topic、执行数据处理、响应功能性服务请求等的执行时间。也不会读取任何已经传送过来的数据

### Primary State: Active

在这个生命周期节点中，这个是一个主要的状态，在这种状态下，节点执行任何处理、响应服务请求、读取和处理数据、生成输出等。

### Primary State: Finalized

最终状态是节点在被销毁之前立即结束的状态。这种状态总是终结的，从这里唯一的过渡就是被摧毁。

这种状态的存在是为了支持调试和自省。一个失败的节点将对系统自省保持可见，并且可能通过调试工具而不是直接销毁来实现自省。如果一个节点在重新生成循环中启动，或者已知循环的原因，则监视流程将具有自动销毁并重新创建该节点的策略。

### Transition State: Configuring

在此状态下，会调用`onConfigure`，加载配置文件，节点的配置通常会涉及在节点的生命周期内必须执行一次的那些任务，例如获取永久内存缓冲区和设置不会更改的主题发布/订阅。

节点使用它来设置它在其整个生命周期中必须持有的任何资源（无论它是活动的还是非活动的）。 作为示例，此类资源可以包括主题发布和订阅、持续持有的内存以及初始化配置参数。

### Transition State: CleaningUp

在这个状态下，`onCleanup`回调函数会被调用，会清理掉所有的状态，如果发送错误会转换到`ErrorProcessing`.

**记住**，所有的转换状态如果失败了都会到 `ErrorProcessing`.

### Transition State: ErrorProcessing

这个过渡状态是可以清除任何错误的地方。 可以从将执行用户代码的任何状态进入此状态。 如果错误处理成功完成，则节点可以返回到未配置状态，如果无法进行完全清理，则它必须失败，节点将转换为已完成以准备销毁。

