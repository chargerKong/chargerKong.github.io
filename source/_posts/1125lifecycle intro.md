---
title: node lifecycle intro
date: 2021-11-25 18:38:29
tags: ros2
---

https://index.ros.org/p/lifecycle/

Introduction
------------

ROS2 introduces the concept of managed nodes, also called ``LifecycleNode``
In the following tutorial, we explain the purpose of these nodes, what makes them different from regular nodes and how they comply to a lifecycle management.
Managed nodes are scoped within a state machine of a finite amount of states.

管理节点的状态被限制于状态机的有限个状态中

These states can be changed by invoking a transition id which indicates the succeeding consecutive state.
The state machine is implemented as described at the `ROS2 design page <http://design.ros2.org/articles/node_lifecycle.html>`.

Our implementation differentiates between ``Primary States`` and ``Transition States``.
Primary States are supposed to be steady states in which any node can do the respected task.
On the other hand, Transition States are meant as temporary intermediate states attached to a transition.
The result of these intermediate states are used to indicate whether a transition between two primary states is considered successful or not.

Thus, any managed node can be in one of the following states:

基本状态是稳定的状态，在这种状态下，任何节点都可以完成相应的任务。另一方面，过渡状态意味着附加到转换的临时中间状态。这些中间状态的结果用于表明两个主要状态之间的转换是否被认为成功。因此，任何管理节点都可能处于以下状态之一:

Primary States (steady states):


* unconfigured
* inactive
* active
* shutdown

Transition States (intermediate states):


* configuring
* activating
* deactivating
* cleaningup
* shuttingdown

The possible transitions to invoke are:


* configure
* activate
* deactivate
* cleanup
* shutdown

For a more verbose explanation on the applied state machine, we refer to the design page which provides an in-detail explanation about each state and transition.

The demo
--------

### What's happening

The demo is split into 3 different separate applications.


* lifecycle_talker
* lifecycle_listener
* lifecycle_service_client

The ``lifecycle_talker`` represents a managed node and publishes according to which state the node is in.
We split the tasks of the talker node into separate pieces and execute them as followed.

lifecycle_talker表示一个管理节点，并根据该节点所处的状态进行数据发布。我们将talker节点的任务分割成几个单独的部分，并按如下方式执行它们。

- configuring: We initialize our publisher and timer
- activate: We activate the publisher and timer in order to enable a publishing
-  deactivate: We stop the publisher and timer
-  cleanup: We destroy the publisher and timer

The principle is implemented in this demo as the typical talker/listener demo.
However, imaging a real scenario with attached hardware which may have a rather long booting phase, i.e. a laser or camera.

想象一下，在真实的情况下，硬件可能有一段相当长时间的启动阶段，例如雷达或者相机

One could image bringing up the device driver in the configuring state, start and stop only the publishing of the device's data and only in the cleanup/shutdown phase actually shutdown the device.

你可以在配置状态下启动设备驱动程序，启动和停止设备数据的发布，只有在清理/关闭阶段才会真正关闭设备。

The ``lifecycle_listener`` is a simple listener which shows the characteristics of the lifecycle talker.
The talker enables the message publishing only in the active state and thus making the listener receiving only messages when the talker is in an active state.

The ``lifecycle_service_client`` is a script calling different transitions on the ``lifecycle_talker``.
This is meant as the external user controlling the lifecycle of nodes.

外部用户可以控制节点的生命周期

Run the demo
------------

In order to run this demo, we open three terminals and source our ROS2 environment variables either from the binary distributions or the workspace we compiled from source.

| lifecycle_talker                                             | lifecycle_listener                                           | lifecycle_service_client                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ``$ ros2 run lifecycle lifecycle_talker``                    | ``$ ros2 run lifecycle lifecycle_listener``                  | ``$ ros2 run lifecycle lifecycle_service_client``            |
| ![asciicast](https://camo.githubusercontent.com/6af40d87a90146d4ef95dadbaf346b7aafab0fd885ea6c5c42cfa33e4230ba4c/68747470733a2f2f61736369696e656d612e6f72672f612f3234393034392e706e67) | ![asciicast](https://camo.githubusercontent.com/e39630aeaf1a23341e89846256563cb02d42d50741ccfea51613cf660633998b/68747470733a2f2f61736369696e656d612e6f72672f612f3234393035302e706e67) | ![asciicast](https://camo.githubusercontent.com/e39630aeaf1a23341e89846256563cb02d42d50741ccfea51613cf660633998b/68747470733a2f2f61736369696e656d612e6f72672f612f3234393035302e706e67) |


Alternatively, these three programs can be run together in the same terminal using the launch file (as of ROS 2 Bouncy):

```
ros2 launch lifecycle lifecycle_demo.launch.py
```

If we look at the output of the ``lifecycle_talker``\ , we notice that nothing seems to happen.
And this does make sense, since every node starts as ``unconfigured``.
The lifecycle_talker is not configured yet and in our example, no publishers and timers are created yet.
The same behavior can be seen for the ``lifecycle_listener``\ , which is less surprising given that no publishers are available at this moment.
The interesting part starts with the third terminal.
In there we launch our ``lifecycle_service_client`` which is responsible for changing the states of the ``lifecycle_talker``.

第一个和第二个窗口什么都没有发生，都处于unconfigured状态，直到第三个窗口启动lifecycle_service_client，他改变了lifecycle_talker的状态

### Triggering transition 1 (configure)

```
   [lc_client] Transition 1 successfully triggered.
   [lc_client] Node lc_talker has current state inactive.
```

makes the lifecycle talker change its state to inactive.
**Inactive means that all publishers and timers are created and configured.**



However, the node is still not active.
Therefore no messages are getting published.

```
   [lc_talker] on_configure() is called.
   Lifecycle publisher is currently inactive. Messages are not published
```

The lifecycle listener on the same time receives a notification as it listens to every state change notification of the lifecycle talker.
In fact, the listener receives two consecutive notifications.

实际上，listener接收两个连续的通知。

One for changing from the primary state "unconfigured" to "configuring".
Because the configuring step was successful within the lifecycle talker, a second notification from "configuring" to "inactive".

```
   [lc_listener] notify callback: Transition from state unconfigured to configuring
   [lc_listener] notify callback: Transition from state configuring to inactive
```





### Triggering transition 2 (activate)
```
 [lc_client] Transition 2 successfully triggered.
 [lc_client] Node lc_talker has current state active.
```

makes the lifecycle talker change its state to active.
That means all **publishers and timers are now activated** and therefore the messages are now getting published.

```
   [lc_talker] on_activate() is called.
   [lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #11]
   [lc_talker] Lifecycle publisher is active. Publishing: [Lifecycle HelloWorld #12]
```

The lifecycle listener receives the same set of notifications as before.
Lifecycle talker changed its state from inactive to active.

```
   [lc_listener]: notify callback: Transition from state inactive to activating
   [lc_listener]: notify callback: Transition from state activating to active
```

The difference to the transition event before is that our listener now also receives the actual published data.

```
   [lc_listener] data_callback: Lifecycle HelloWorld #11
   [lc_listener] data_callback: Lifecycle HelloWorld #12
```

Please note that the index of the published message is already at 11.
The purpose of this demo is to show that even though we call ``publish`` at every state of the lifecycle talker, only when the state in active, the messages are actually published.

这个demo的目的是，即使我们在talker生命周期的每个状态都调用 publish ，也只有当状态处于活动状态时，消息才会被实际发布。

As for the beta1, all other messages are getting ignored.
This behavior may change in future versions in order to provide better error handling.

For the rest of the demo, you will see similar output as we deactivate and activate the lifecycle talker and finally shut it down.

The demo code
-------------

### lifecycle_talker, lifecycle_listener and lifecycle_service_client

If we have a look at the code, there is one significant change for the lifecycle talker compared to a regular talker.
**Our node does not inherit from the regular ``rclcpp::node::Node`` but from ``rclcpp_lifecycle::LifecycleNode``.**

生命周期结点继承的不是rclcpp::node::Node，而是rclcpp_lifecycle::LifecycleNode

```
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
```

Every child of LifecycleNodes have a set of callbacks provided.
These callbacks go along with the applied state machine attached to it.
These callbacks are:

他的每一个子类都有一系列的callback函数

```c++
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_configure(const rclcpp_lifecycle::State & previous_state)

   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_activate(const rclcpp_lifecycle::State & previous_state)

   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_deactivate(const rclcpp_lifecycle::State & previous_state)

   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_cleanup(const rclcpp_lifecycle::State & previous_state)

   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_shutdown(const rclcpp_lifecycle::State & previous_state)
```

In the following we assume that we are inside the namespace ``rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface`` to shorten the name of the return type.
All these callbacks have a positive default return value (\ ``return CallbackReturn::SUCCESS``\ ).

所有的callback函数都有一个默认的返回值 ``return CallbackReturn::SUCCESS``\

This allows a lifecycle node to change its state even though no explicit callback function was overwritten.

这样即便是没有显示的覆盖。也可以让生命周期结点正常改变他的状态

There is one other callback function for error handling.
Whenever a state transition throws an uncaught exception, we call ``on_error``.

当任何一个状态转换发生错误的时候，就会调用on_error回调函数


* ``CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state)``

This gives room for executing a custom error handling.
Only (!) in the case that this function returns ``CallbackReturn::SUCCESS``\ , the state machine transitions to the state ``unconfigured``.
By default, the ``on_error`` returns ``CallbackReturn::FAILURE`` and the state machine transitions into ``finalized``.

这为执行自定义错误处理提供了空间。只有在该函数返回CallbackReturn::SUCCESS的情况下，状态机转换为unconfigured的状态。默认情况下，on_error返回CallbackReturn::FAILURE，状态机转换为finalize。



At the same time, every lifecycle node has by default 5 different communication interfaces.

同时，每个生命周期节点默认有5个不同的通信接口。


* Publisher ``<node_name>__transition_event`` : publishes in case a transition is happening. 

* 当转换发生的时候调用

  

This allows users to get notified of transition events within the network.

* Service ``<node_name>__get_state``\ : query about the current state of the node.
* 查询node现在的状态，返回一个primary或者transition的状态

Return either a primary or transition state.

* Service ``<node_name>__change_state``\ : triggers a transition for the current node.
* 出发一个节点的转换

This service call takes a transition id.
Only in the case, that this transition ID is a valid transition of the current state, the transition is fulfilled.
All other cases are getting ignored. ？？

* Service ``<node_name>__get_available_states``\ : This is meant to be an introspection tool.

It returns a list of all possible states this node can be.

* Service ``<node_name>__get_available_transitions``\ : Same as above, meant to an introspection tool.

It returns a list of all possible transitions this node can execute.

### ros2 lifecycle command line interface
The ``lifecycle_service_client`` application is a fixed order script for this demo purpose only.
It explains the use and the API calls made for this lifecycle implementation, but may be inconvenient to use otherwise.
For this reason we implemented a command line tool which lets you dynamically change states or various nodes.

它解释了这个生命周期实现的使用和API调用，但如果不这样使用可能会不方便。出于这个原因，我们实现了一个命令行工具，它可以让你动态地改变状态或各种节点。

In the case you want to get the current state of the ``lc_talker`` node, you would call:

```
   $ ros2 lifecycle get /lc_talker
   unconfigured [1]
```

The next step would be to execute a state change:

```
   $ ros2 lifecycle set /lc_talker configure
   Transitioning successful
```

In order to see what states are currently available:

```
   $ ros2 lifecycle list lc_talker

   - configure [1]
     Start: unconfigured
     Goal: configuring
   - shutdown [5]
     Start: unconfigured
     Goal: shuttingdown
```

In this case we see that currently, the available transitions are ``configure`` and ``shutdown``.
The complete state machine can be viewed with the following command, which can be helpful for debugging or visualization purposes:

可以使用以下命令查看完整的状态机，这对调试或可视化很有帮助:

```
$ ros2 lifecycle list lc_talker -a

   - configure [1]
     Start: unconfigured
     Goal: configuring
   - transition_success [10]
     Start: configuring
     Goal: inactive
   - transition_failure [11]
     Start: configuring
     Goal: unconfigured
   - transition_error [12]
     Start: configuring
     Goal: errorprocessing

   [...]

   - transition_error [62]
     Start: errorprocessing
     Goal: finalized


```

   All of the above commands are nothing else than calling the lifecycle node's services.
With that being said, we can also call these services directly with the ros2 command line interface:

```
 $ ros2 service call /lc_talker/get_state lifecycle_msgs/GetState
   requester: making request: lifecycle_msgs.srv.GetState_Request()
   
   response:
   lifecycle_msgs.srv.GetState_Response(current_state=lifecycle_msgs.msg.State(id=1, label='unconfigured'))
```



In order to trigger a transition, we call the ``change_state`` service

```
  $ ros2 service call /lc_talker/change_state lifecycle_msgs/ChangeState "{transition: {id: 2}}"
   requester: making request: lifecycle_msgs.srv.ChangeState_Request(transition=lifecycle_msgs.msg.Transition(id=2, label=''))
   
 response:
   lifecycle_msgs.srv.ChangeState_Response(success=True)

```

   

It is slightly less convenient, because you have to know the IDs which correspond to each transition.
You can find them though in the lifecycle_msgs package.

它稍微有些不方便，因为您必须知道与每个转换对应的id。您可以在lifecycle_msgs包中找到它们。

    $ ros2 interface show lifecycle_msgs/msg/Transition

Outlook
-------

The above description points to the current state of the development as for beta1.
The future todo list for this topic comprises:


* Python lifecycle nodes
* Lifecycle manager: A global node, handling and dispatching trigger requests for multiple nodes.
* LifeyclceSubscriber/LifecycleWalltimer/... add more lifecycle controlled entities.