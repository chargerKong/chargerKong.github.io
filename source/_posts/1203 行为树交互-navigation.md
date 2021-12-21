---
title: Groot-与行为树交互
date: 2021-12-03 18:38:29
tags: navigation
---

Groot是BehaviorTree.CPP库的配套应用程序，用于创建、编辑和可视化行为树。行为树被深入集成到Nav2中，作为在复杂的导航和自治堆栈中编排任务服务器逻辑的主要方法。行为树，简称BTs，由许多完成不同任务和控制逻辑流的节点组成，类似于有限状态机，但以树的结构构成。这些节点的类型有: Action, Condition, Control, 或者 Decorator，在 [Navigation Concepts](https://navigation.ros.org/concepts/index.html#concepts) 和 [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).中有更详细的描述。

[Writing a New Behavior Tree Plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html#writing-new-nbt-plugin)提供了一个编写良好的例子，如果创建新的BT节点，如何创建一个简单的Action节点。本教程将专注于启动Groot，可视化一个行为树，并且自定义行为树，假设有一个BT节点库。幸运的是，Nav2提供了大量的BT节点供你开箱即用，enumerated in [Navigation Plugins](https://navigation.ros.org/plugins/index.html#plugins).

 BehaviorTree.CPP的配置文件为XML文件。这用于在运行时从适当库的中动态加载BT节点插件。详细XML格式 [in detail here](https://www.behaviortree.dev/xml_format/). 。因此，Groot需要有一个它可以访问的节点列表，以及关于它们的重要元数据，比如它们的类型和端口(或参数)。在本教程的后面，我们将其称为节点的“pallet”。

在上面的视频中，你可以看到Groot与RVIz并排，以及一个100%配备了SIEMENS支持ros硬件的测试平台。Groot不仅在机器人操作时显示当前的行为树。注意:在ROS 2 Humble之前，Nav2支持在执行过程中实时监视Groot行为树。由于BT.CPP /Groot对飞行中改变行为树的bug支持，这一功能被移除

# 行为树可视化

1. 打开Groot，如图1
2. 选择*Load palette from*
3. 打开“/path/to/navigation2/nav2_behavior_tree/nav2_tree_nodes.xml”文件，导入所有用于导航的自定义行为树节点。注意这个时候，左边的palette会多一些内容，如图2
4. 选择Load tree
5. 选择 navigate_w_replanning_and_recovery.xml，如图3

![../../_images/groot_bt_editor.png](https://navigation.ros.org/_images/groot_bt_editor.png)

![../../_images/groot_with_nav2_custom_nodes.png](https://navigation.ros.org/_images/groot_with_nav2_custom_nodes.png)

![../../_images/bt_w_replanning_and_recovery.png](https://navigation.ros.org/_images/bt_w_replanning_and_recovery.png)

# 行为树编辑

现在你已经在编辑模式下在Groot中打开了一个Nav2 BT，你应该能够使用GUI简单地修改它。从如图4所示的屏幕开始，您可以从侧面板拉入新的节点，将它们添加到工作区中。然后，您可以在节点的输入和输出端口之间使用“拖放”动作连接节点，以将新节点组装到树中。

如果您选择一个给定的节点，您可以更改关于它的元数据，比如它的名称或可参数化端口的值。当您完成修改后，只需保存新的配置文件，并在下一次使用您的机器人!

# 添加一个已定义的节点

行为树中的每个节点都拥有一个专门的函数。有时，在设计过程中创建新节点并将它们添加到您的托盘中是很有用的——可能是在实现本身存在之前。

这有助于设计者从树本身的更高层次逻辑中抽象出节点的实现细节，以及它们希望如何与给定节点交互(例如类型、端口等)。在Groot中，您可以创建新的自定义节点添加到您的树中，并将这些新节点导出到您的pallet中。实现节点本身需要与Groot分开完成，这在编写一个新的行为树插件中进行了描述。

实现节点本身需要与Groot分开完成，这在[Writing a New Behavior Tree Plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html#writing-new-nbt-plugin).中进行了描述。



点击新建一个自定义节点

![../../_images/groot_create_custom_node.png](https://navigation.ros.org/_images/groot_create_custom_node.png)



在编辑模式下，应该出来如下内容，定义节点的名字，类型以及相关的参数,

![../../_images/groot_interactive_node_creation.png](https://navigation.ros.org/_images/groot_interactive_node_creation.png)

点击完成后，在palette中会多一项

![../../_images/groot_export_new_node.png](https://navigation.ros.org/_images/groot_export_new_node.png)

建议将新创建的节点导出保存，以防Groot崩溃。点击上图中绿色的框，对树的model进行导出

