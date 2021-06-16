---
title: IMLS-ICP
date: 2021-06-09 18:38:29
tags:
---

## 查看Path数据

```
ros2 interface show nav_msgs/msg/Path
```

返回

```
# An array of poses that represents a Path for a robot to follow.

# Indicates the frame_id of the path.
std_msgs/Header header

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses
```



### 查看header的数据结构

```
ros2 interface show std_msgs/msg/Header
```

返回

```
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id
```



## Python的点云显示

```python
import numpy as np
import open3d as o3d

pc_view = o3d.geometry.PointCloud()
pc_view.points = o3d.utility.Vector3dVector(np.array([[1,2,3]]))
o3d.visualization.draw_geometries([pc_view])
```

