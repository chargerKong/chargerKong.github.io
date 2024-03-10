---
title: Docker简单使用
date: 2021-12-02 14:30:29
tags: gazebo
---

# gazebo教程补充

http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b3

在建立简单模型一节，建立joint的时候，有一个对齐概念

1. child to parent

   把child对齐到parent的x，y，z轴。即child的x轴的最小值对齐到parent x轴的最小值，以此对应

   下面三张图分别是x轴（正方体为parent）对齐的min， center 和max

   

![](/home/kong/Pictures/left.png)![center](/home/kong/Pictures/center.png)![max](/home/kong/Pictures/max.png)



注意最后仿真的时候，车头位置在哪。。。把立方体放在车尾可动不了

# Extrude SVG files

 http://gazebosim.org/tutorials?cat=model_editor_top&tut=extrude_svg

这一节中，描述了如何通过利用 Inkscape 绘制平面图，然后再gazebo里面导入平面图创建三维模型

Inkscape绘图保存为SVG文件，gazebo支持支持2D多边形线绘制模型，他可以从SVG里面提取出多边形从而形成三维模型

# Building Editor

http://gazebosim.org/tutorials?cat=build_world&tut=building_editor

几个注意：

- building无法再被编辑

- 目前还不知道编辑自定义的墙面纹理
- 目前所有的地板都是长方形的

# 记录和回放功能



# Velodyne建立

尾缀为.world文件结构：

```
<sdf version=1.5>
	<world name="default">
		<model name="world">
			<link name="base"> <!--下底座-->
				<self_collide>1</self_collide>
				...
				...
			</link>
			
			
			
			<link name="top"> <!--上面旋转部分-->
				<self_collide>1</self_collide>
				...
				...
			</link>
			
			
			
			<joint type="revolue" name="joint">
				...
			</joint>
			
			
			
		</model>
	</world>
</sdf>
```

注意千万别打错了，<collision>打错的话，直接link就往下掉。

上面的物体不可以太重！！！！否则就会发生偏移啥的，如果没有joint都直接掉下来了



## 坑一：step1

在每一个link里面需要添加一个`<self_collide>1</self_collide>`。这样上面的下面的底座才不会重叠。

输入

```
gazebo velodyne.world -u
```

打开这world



## 坑二: step3

施加力的时候，按照官网的示意图去做，会让雷达越转越快

换一种方法，选择model，点击右键添加力

![force](/home/kong/Pictures/force.png)





# Improve Appearance

Gazebo can only use STL, OBJ or Collada files

 Collada files就是 .dae文件。

用`freecad`工具把`step`文件改为`dae`文件，用`blender`修改单位，gazebo要求的是米

mesh 是可以替代原来的cylinder

texture是用于描绘mesh表面的



# 添加传感器噪声

## 坑一

Ctrl + t 打开topic， 打开`…/top/sensor/scan`后，需要放大放大再放大！！！！不然看不见

# 插件

http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin

现在有六种插件

- world

- model

- sensor

- system

- visual

- GUI

  system插件是通过命令行进行控制的

写插件代码 的思路

1. 写好代码：从gazebo的类里面继承，CMakeLists写好

2. 编译

3. 把编译好的so文件，放入gazebo的插件path。

   `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo/plugin/build`

4. 运行插件。`gzserver hello.world --verbose`

运行插件的时候，会报一个wrn

```
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 100.67.32.17
[Msg] Loading world file [/home/kong/gazebo/plugin/hello.world]
^Chello world[Wrn] [Publisher.cc:136] Queue limit reached for topic /gazebo/default/physics/contacts, deleting message. This warning is printed only once.
```

https://github.com/schmerl/LLStaging/issues/52

据说不影响结果，可以被忽略。

## model 插件

插件允许完全访问模型及其底层元素(链接、关节、碰撞对象)的物理属性。下面的插件将应用线性速度到它的父模型。

```
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
```

添加CMakeList.txt

```
add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})
```

写model_push.world

```
<?xml version="1.0"?> 
<sdf version="1.4">
  <world name="default">

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>

      <plugin name="model_push" filename="libmodel_push.so"/>
    </model>        
  </world>
</sdf>
```

把build的目录添加到GAZEBO_PLUGIN_PATH

```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo/plugin/build

```

## world插件

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/gazebo/plugin/models







# Control plugin

## step5 

启动自己的插件之前，即

```
gazebo --verbose ../velodyne.world
```

需要把编译生成的`libvelodyne_plugin.so`文件的文件目录路径导入到`GAZEBO_PLUGIN_PATH`

```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo/velodyne_plugin/build
```



## 修改model配置的方法

注意插件是放入world文件里面

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A testing model that includes the Velodyne sensor model -->
    <model name="my_velodyne">
      <include>
        <uri>model://velodyne_hdl32</uri>
      </include>

      <!-- Attach the plugin to this model -->
      <plugin name="velodyne_control" filename="libvelodyne_plugin.so"/>
    </model>

  </world>
</sdf>
```

插件放入需要控制的model里面，和include同级

若需要控制model，则可以：

- 直接修改plugin

- 直接修改xml文件，**不需要重新进行编译**

  ```xml
  <plugin name="velodyne_control" filename="libvelodyne_plugin.so">
    <velocity>25</velocity>
  </plugin>
  ```

  然后去cc文件进行数据读取

  ```c++
  // Default to zero velocity
  double velocity = 0;
  
  // Check that the velocity element exists, then read the value
  if (_sdf->HasElement("velocity"))
    velocity = _sdf->Get<double>("velocity");
  
  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
  this->model->GetJointController()->SetVelocityTarget(
      this->joint->GetScopedName(), velocity);
  ```

- 创建API，两种方式。可以动态调整我们这速度的问题。

  - topic，插件接收topic的值
  - 函数，我们需要从当前的插件中继承一个新的插件。子插件将由Gazebo而不是我们当前的插件来实例化，并通过调用我们的函数来控制速度。这种类型的方法在将Gazebo连接到ROS时最常用



