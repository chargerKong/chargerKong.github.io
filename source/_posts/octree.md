---
title: octree
date: 2021-03-07 16:24:35
---

## 1. 八叉树性质

八叉树应用于三维数据的存储和显示，有以下特点

- 每一个节点Octant都有八个子节点
- KD树中，我们不确定其它区域是否有更近的点，因此还需要回去查看。但是八叉树不需要，因此八叉树

![img](fig1.jpg)

## 2.1 Octant 构建

以二维的为例，作为最大的区域，进行分割一分为四, ,,。其中一个没有点，不做标记。

![img](fig2.jpg)

然后对小区域再一次进行分割, 形成第三层小区域

![img](fig3.jpg)

然后形成第四层小区域，在此处，我们叶子节点数量`leaf_size=1`

![img](fig4.jpg)

这就形成了根据点的密集程度来划分点的网格。

1.2 八叉树构建

下面设计一下Octant需要存储的信息

```
11 class Octant:
12     def __init__(self, children, center, extent, point_indices, is_leaf):
13         """
14         children: 长度为8的列表
15         center: Octant的中心点
16         extent: Octant的半径
17         point_indices: 数据的序号
18         is_leaf: 是否为叶子节点                                                                                                          
19         """
20         self.children = children
21         self.center = center
22         self.extent = extent
23         self.point_indices = point_indices
24         self.is_leaf = is_leafpoint_indices
```

## 2.2 八叉树构建

构建八叉树的过程其实就是建立各个Octant关系的一个过程。

我们创建一个函数，传入一个root, 即便是None，只要有点，就返回一个是Octant的root。`octree_recursive_build` , 这里添加一个参数`min_extent`，这是为了防止两个完全重合的点，无论怎么切割都无法分出一个Octant。

```
63 def octree_recursive_build(root, db, center, extent, point_indices, leaf_size, min_extent):
64     if len(point_indices) == 0:
65         return None
66 octree_recursive_build
67     if root is None:
68         root = Oct
68         root = Octant([None for i in range(8)], center, extent, point_indices, is_leaf=True)
69 
70     # determine whether to split this octant
71     if len(point_indices) <= leaf_size or extent <= min_extent:
72         root.is_leaf = True
73     else:
```

代码64行，如果没有点了，直接返回None

代码67行，如果有点，这个点还是None，那么就对这个点创建一个Octant。我们每一次只要传入一个None就行，因为会自动创建出一个Octant

代码70行，如果点的数量小于叶子节点的最大数量或者半径太小。则判定此为一个叶子节点

octree_recursive_build当以上条件均不满足，我们开始建立此Octant 和其它子Octant的关系

关于子Octant的信息，有：center,extent,point_indices

1. point_indiand len(root.point_indices) > 0:ces: 确定现在这些点属于哪个小的Octant。我们可以先建立一个`child_list=[[] for _ in range(8)]`，然后往这八个区域里面添加point的序号。

   ```
   child_list=[[] for _ in range(8)]
   for point_idx in point_indices:
       point = db[point_idx]
       # 判断下标为point_idx的点应该要去哪个子区域
       # 通过判断x, y, z和center的相对位置来判定去哪个子区域
       zone = 0
       if point[0] > center[0]:
           zone = zone | 1
       if point[1] > center[1]:
           zone = zone | 2
       if point[2] > center[2]:
           zone = zone | 4
       # 把相应的点放入相应的区域
       child_list[zone].append(point_idx)    
   ```

2. extent: 1小的Octant为原来Octant的一半

   ```
   extent = extent / 2
   ```

3. center: 有八个小Octant，根据不同的小Octant计算每一个的center

   计算方式：

   new_center = center 0.5 * extent

   ```
   factor = [-0.5, 0.5]
   for i in range(8):
       center_new_x = center[0] + factor[(i & 1) > 0] * extent
       center_new_y = center[1] + factor[(i & 2) > 0] * extent
       center_new_z = center[2] + factor[(i & 4) > 0] * extent
       cen_new = np.asarray([center_new_x, center_new_y, center_new_z])
       child_extent = extent / 2
       root.children[i] = octree_recursive_build(root.children[i],
                                               											db,
                                               											cen_new,
                                               											child_extent,
                                               											children_list[i],
                                               											leaf_size,
                                               											min_extent)
       
       
   return root
   ```

## 3. Knn搜索

假设有query点。首先快速定位到query点所在的区域。

```
def knn(root, query):
    if root.is_leaf():
        pass
    else:
        zone = 0
        if query[0] > root.center[0]:
            zone = zone | 1
        if query[1] > root.center[1]:
            zone = zone | 2
        if query[2] > root.center[2]:
            zone = zone | 4
        knn(root.chilren[zone], query)
```

如果到了叶子节点，计算每一个点到query的距离

```
242     if root.is_leaf and len(root.point_indices) > 0:
243         # compare the contents of a leaf
244         leaf_points = db[root.point_indices, :]
245         diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
246         for i in range(diff.shape[0]):
247             result_set.add_point(diff[i], root.point_indices[i])
248         # check whether we can stop search now
249         return inside(query, result_set.worstDist(), root)
```

整个KNN搜索思路

首先快速定位到该点（query）所在区域，遍历区域内所有的点，记录下最远的距离。遍历结束判断该点所在的最远区域的球是否再该区域的正方体内，是，则停止搜索其它区域。

否则，开始寻找上层区域的其它的孩子节点（区域），跳过上一段去过的区域和没有与球重叠的区域。加入有重叠区域的点，更新最远距离。结束后，判断本区域是否完全包含query所在的球，是就 结束。

```
238 def octree_knn_search(root: Octant, db: np.ndarray, result_set: KNNResultSet, query: np.ndarray):
239     if root is None:
240         return False
241 
242     if root.is_leaf and len(root.point_indices) > 0:
243         # compare the contents of a leaf
244         leaf_points = db[root.point_indices, :]
245         diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
246         for i in range(diff.shape[0]):
247             result_set.add_point(diff[i], root.point_indices[i])
248         # check whether we can stop search now
249         return inside(query, result_set.worstDist(), root)
250 
251     # 作业7
252     # 屏蔽开始
253     # 前去相应的区域
254     zone = 0
255     if query[0] > root.center[0]:
256         zone = zone | 1
257     if query[1] > root.center[1]:
258         zone = zone | 2
259     if query[2] > root.center[2]:
260         zone = zone | 4
261     
262     if octree_knn_search(root.children[zone], db, result_set, query):
263         return True
264 
265     # 检查别的子节点
266     for idx, child in enumerate(root.children):
267         # zone 是自己的所在的区域，直接跳过。
268         if idx == zone or child is None:
269             continue
270         # 跳过没有重叠的区域                                                                                                                               
271         if False == overlaps(query, result_set.worstDist(), child):
272             continue
273         if octree_knn_search(root.children[idx], db, result_set, query):
274             return True
275     
276     return inside(query, result_set.worstDist(), root)
```

### 3.1 碰撞检测

1. case 1: 没有碰撞

![img](fig5.jpg)

假设query点到八叉树某区域正方体的中心点的距离是，**其实只要任何一个轴的距离大于max_dis = extent + radius**（extent为八叉树某区域的正方体边长的一半，radius为以query点为圆心，radius为半径的球的半径）

若三个维度的值都小于等于max_dis, 则说明球不会距离正方体太远，投影到二维平面上，应该包含如下这些情况

![图6](fig7.jpg)图6

这里三个球都是满足, 但是也有两种不同的情况，可以看见依然是有情况是没有碰撞的，最上面的球和最下面的球都是没有碰撞的。

下面先处理上图中画圈圈的区域，因为在这种区域内，一旦控制了只要任何一个轴的距离小于等于max_dis，则球体和正方体一定会有碰撞。

```
max_dis = root.extent + radius
if np.any(np.fabs(query - root.extent)) > max_dis:
    return False
```

1. case 2: 非四个角有碰撞

![img](fig6.jpg)

需要控制球在图6画上黑色圈圈的范围内，则需要

首先看图6的正上方点，就是再z轴上的投影。query点的必须要再center点的的一半边长的范围内即并且上面整合一下其它两个轴同理。

图8是图6的三维形式

![图8](fig8.jpg)图8

如果需要控制query点在再六个面投射出去对应的空间内，则需要下面三个条件任意两个

```
query_offset_abs = np.fabs(query - root.center)
# query_offset_abs 就是 (\Delta x ,\Delta y, \Delta z)
# np.sum(query_offset_abs  < extent)  的结果类似于是 [0, 0, 1]， 只要有两个及以上就行
if np.sum(query_offset_abs  < extent) >= 2:
    return True
```

1. case 3

除去上述两种情况，剩下的情况，也会有碰撞以及没有碰撞的情况，如下图

![图9](fig9.jpg)图9

这里的两个圆，圆心都已经在四个投射面的外面了，但是依然会有碰撞和没有碰撞的情况。下面这个是临界点

![图10](fig10.jpg)图10

`query - root.center`可以表示为由向量和向量向量组成的向量( 这里E 画的不太标准）， 如果我们不计算球是否再正方体内，则可以减去半径，

则可以理解为由向量和向量组成的。

如果向量的长度小于半径，则可以判定他和立方体是有交集的。

```
query_offset_abs = np.fabs(query - root.center)
x_diff = query_offset_abs[0] - root.center
y_diff = query_offset_abs[1] - root.center
z_diff = query_offset_abs[2] - root.center
return x_diff^2 + y_diff^2 + z_diff^2 < radius * radius
```

![img](fig11.jpg)

但是在三维空间中，多了一个z轴，是可以在z轴上进行上下平移的，图10仅为投影下的情况，下面看图11，球所在的高度没有高于正方体，那么z轴其实是不需要了，按照投影来看，其实就是图9。因此在当发现z_diff的值是负数的时候，可以将其设置为0。就退化到图9的情形，也可以正确的判定。

同理也可以应对贴着x轴和y轴的情况。

代码修改为

```
query_offset_abs = np.fabs(query - root.center)
x_diff = max(query_offset_abs[0] - root.center, 0)
y_diff = max(query_offset_abs[1] - root.center, 0)
z_diff = max(query_offset_abs[2] - root.center, 0)
return x_diff^2 + y_diff^2 + z_diff^2 < radius * radius
```

最后给出一个判断有没有重叠部分的函数

```
135 def overlaps(query: np.ndarray, radius: float, octant:Octant):
136     """
137     Determines if the query ball overlaps with the octant
138     :param query:
139     :param radius:
140     :param octant:
141     :return:
142     """
143     query_offset = query - octant.center
144     query_offset_abs = np.fabs(query_offset)
145 
146     # completely outside, since query is outside the relevant area
147     max_dist = radius + octant.extent
148     if np.any(query_offset_abs > max_dist):
149         return False
150 
151     # if pass the above check, consider the case that the ball is contacting the face of the octant
152     if np.sum((query_offset_abs < octant.extent).astype(np.int)) >= 2:
153         return True
154 
155     # conside the case that the ball is contacting the edge or corner of the octant
156     # since the case of the ball center (query) inside octant has been considered,
157     # we only consider the ball center (query) outside octant
158     x_diff = max(query_offset_abs[0] - octant.extent, 0)
159     y_diff = max(query_offset_abs[1] - octant.extent, 0)
160     z_diff = max(query_offset_abs[2] - octant.extent, 0)
161 
162     return x_diff * x_diff + y_diff * y_diff + z_diff * z_diff < radius * radius
```

