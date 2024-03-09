---
title: KDtree
date: 2021-03-06 11:01:52
tags: [math,机器学习]
---

本文主要讲解利用KD树建立，以及搜索

## 1. 暴力解法

假设对数据的第一个点寻找与他最近的几个点

```
# db_np 为Nx3的三维数据，需要找的数据是第一行的数据, 比如
# db_np = array([[ 5.2897942e+01,  2.2989739e-02,  1.9979945e+00],
#       [ 5.3750526e+01,  1.9291429e-01,  2.0269539e+00],
#       [ 5.3803116e+01,  3.6183926e-01,  2.0289137e+00],
#       ...,
#       [ 3.8401384e+00, -1.4381756e+00, -1.7735560e+00],
#       [ 3.8257158e+00, -1.4192016e+00, -1.7645701e+00],
#       [ 4.0923753e+00, -1.5071962e+00, -1.8955611e+00]], dtype=float32)
#  则我们需要求和[0.52897, 0.022, 1.9979]最近的几个点
```

对每一个点进行距离的计算

```
query = db_np[0,:]
dist = np.linalg.norm(db_np - query, axis=1)
# 这样dist = array([ 0.        ,  0.86983556,  0.96701366, ..., 49.224262 , 49.23739   , 48.98453   ], dtype=float32)
```

接下来对距离进行排序

```
nn_idx = np.argsort(dist)
# 按照距离进行排序，出来的是原来数组的下标array([    0,  3943,  5884, ..., 20570, 22575, 18609])
# 按照距离给出的序号，对原来的db_np进行排序
min_dist_sort = db_np[nn_idx]
# 此时min_dist_sort = array([[ 5.2897942e+01,  2.2989739e-02,  1.9979945e+00],
#       [ 5.2810284e+01, -1.1394917e-01,  1.6600274e+00],
#       [ 5.2790421e+01, -1.6892470e-01,  1.4110407e+00],
#       ...,
#       [-7.7472404e+01, -7.5055069e-01, -1.8607584e+00],
#       [-7.7778915e+01, -4.4104338e-01, -2.3320513e+00],
#       [-7.8087395e+01, -4.3613955e-01, -1.4752271e+00]], dtype=float32)
# 已经是排好序的点
```

以上暴力搜索的时间为（在我的机器上）：

## 2. KD（K-Dimensions）树

它是按照不同的维度给数据进行划分区域，找点的时候需要按照区域来寻找，下面介绍一颗二维数据树的使用

### 2.1 如何构建一颗KD树

假设我们有这么多的点，其中，

![img](sfig1.jpg)

#### 2.1.1 构建Node数据结构

他们组成一个大区域，我们需要一些数据来存储这个区域。

- 这个区域会沿着哪个轴进行划分, 用一个数字保存。0表示第一个轴，1表示第二个轴... 这是因为在python中储存的点形式为，就是第一个轴上的值为3。0就表示为x轴。
- 那个轴上的划分线的值
- 左子树
- 右子树
- 这个区域内包含哪些点

因此考虑一个新的数据结构来进行存储

```
class Node:
	def __init__(self, axis, value, left, right, point_indices):
    	self.axis = axis
        self.value = value
        self.left = left
        self.right = right
        self.point_indices = point_indices
```

对于这个区域，我们对轴进行分割，其值记为

![img](fig2.jpg)

那么，在这里代表的是在轴上切割，value取值为在x轴上的值，point_indices表示对应原数据的下标，记录是什么点在这个区域内。left就是左边的框的一个Node

#### 2.2.2 建立一棵树

目标

- 输入：一堆维度一样的数据
- 输出：一颗KD树

假设已经设置好了需要被构建为KD的数据db_np.

```
import numpy as np
db_np = np.asarray([[1,8],[3,7],[5,7],[1.3,4],[3.5,3.8],[7,4],[3,2.5],[6,3],[4.5,1]])
```

想通过一句话构建出KD树，（ leaf_size指切割最后叶子节点的点的数目，比如leaf_size=2，则当某区域的点小于等于2时，就不再分割。）

```
root = kdtree_construction(db_np, leaf_size=leaf_size)
```

构建一个区域需要之前介绍的这些参数，因此还需要嵌套一层函数比较合适

```
def kdtree_construction(db_np, leaf_size):
    N, dim = db_np.shape[0], db_np.shape[1]

    # build kd_tree recursively
    root = None
    root = kdtree_recursive_build(root,
                                  db_np,
                                  np.arange(N),
                                  axis=0,
                                  leaf_size=leaf_size)
    return root
```

这里`np.arang(N)`表示序号，root表示最大的那个区域。

需要再`kdtree_recursive_build`构建每一个Node, 在这里，默认从x轴开始，因此输入0.

在`kdtree_recursive_build`中，首先要定一下切割的位置，我们可以把所有点的x轴拿出来，然后进行排序，找到中间的点

在上面的例子中，相当于排序`[1, 3, 5, 1.3, 3.5 ,7, 6, 4.5]`,

这里可以定义一个函数进行排序

输入：点的序号和对应轴的坐标

输出：根据轴的坐标排好序号的下标和值

```
def sort_key_by_vale(point_indices, value):                                                                                                                              
    assert point_indices.shape == value.shape
    assert len(point_indices.shape) == 1
    sorted_idx = np.argsort(value)
    point_indices_sorted = point_indices[sorted_idx]
    value_sorted = value[sorted_idx]
    return point_indices_sorted, value_sorted
```

然后找寻中间的点，左边的放入left节点，右边的放入right节点。

```
# 放入当前区域的点的序号和点的值
point_indices_sorted = sort_key_by_vale(point_indices, db[point_indices])
# 找寻中间的点
mid_indices = point_indices_sorted.shape[0] // 2
# 序号左边的点放入左边节点
left_indices = point_indices_sorted[:mid_indices]
# 序号右边的点放入右边界点
right_indices = point_indices_sorted[mid_indices:]
```

找到分割线，不要和点重合，找到左边的点最右边的点和右边最左边的点求平均

```
# 把左边序号最后一个和右边序号第一个的坐标求中间值
root.value = (db[mid_indices - 1, axis] + db[mid_indices, axis]) * 0.5
```

定义左右边节点, 并且更改切割的轴，我们定义了一个axis_round_robin函数来改变需要切换的轴，简单来说就是x轴、y轴、z轴、x轴、y轴如此循环下去

```
# 给左边节点赋值
root.left = kdtree_recursive_build(root.left,
                            		db, 
                                   left_indices,
                                   axis_round_robin(axis, dim=db.shape[1]),                                                                            
                                   leaf_size)
# 给右边节点赋值
root.right = kdtree_recursive_build(root.right,
                                    db, 
                                    right_indices,
                                    axis_round_robin(axis, dim=db.shape[1]),
                                    leaf_size)
```

在此之前，也需要对root进行定义

```
if root is None:
        root = Node(axis, None, None, None, point_indices)
```

以上对root的切割和左右区域的操作都需要一个条件, 即剩下的点的数量大于叶子节点数。

以下是全部的代码

```
def kdtree_recursive_build(root, db, point_indices, axis, leaf_size):
    if root is None:
        root = Node(axis, None, None, None, point_indices)

    # determine whether to split into left and right
    if len(point_indices) > leaf_size:
        # --- get the split position ---
        point_indices_sorted, _ = sort_key_by_vale(point_indices, db[point_indices, axis])  # M
        # 当这个节点还不是叶子节点的时候，需要定义左子树和右子树
        # 首先求出中间节点的序号
        mid_indices = point_indices_sorted.shape[0] // 2
        # 序号左边的点放入左边节点
        left_indices = point_indices_sorted[:mid_indices]
        # 序号右边的点放入右边界点
        right_indices = point_indices_sorted[mid_indices:]
        # 把左边序号最后一个和右边序号第一个的坐标求中间值
        root.value = (db[mid_indices - 1, axis] + db[mid_indices, axis]) * 0.5
        # 给左边节点赋值
        root.left = kdtree_recursive_build(root.left,
                                           db, 
                                           left_indices,
                                           axis_round_robin(axis, dim=db.shape[1]),
                                           leaf_size)
        # 给右边节点赋值
        root.right = kdtree_recursive_build(root.right,
                                           db, 
                                           right_indices,
                                           axis_round_robin(axis, dim=db.shape[1]),
                                           leaf_size)
    return root
```

### 2.2 找寻一个点的附近点，K-NN

我们通过对比被寻找点与KD树再每一个维度上的切割的值，快速找到相应的区域。

假设已经对数据点分好区域，很明显这里的`leaf_size=1`。我们的目标是找到和红点最近的K个点，先看如何找到最近的一个点。

![img](fig3.jpg)

右图是从上往下看。

第一个代表根节点，就是最大的区域，代表x轴的分割点

第二行代表分割完的节点，左边的区域分割轴为y轴，y值为, 右边的区域分割轴为y轴，y值是

...

按照区域的划分，先和x轴的进行对比，我们有，因此选择root节点的左边节点，然后对比和，，因此选择上边的区域，这个区域分割点是, 依然是y轴，继续往上找，发现已经是一个叶子节点了，可以直接计算叶子节点里的点距离红点的距离。

上面的过程可以通过类似于下面的代码实现, 这一段代码就表示着区域的寻找

```
def kdtree_knn_search(root: Node, db: np.ndarray, result_set: KNNResultSet, query: np.ndarray):
    # 判断query点和分割点的距离
    if query[root.axis] < root.value:
        kdtree_knn_search(root.left, db, result_set, query)
    else:
        kdtree_knn_search(root.right, db, result_set, query)
```

一直寻找到叶子节点为止(需要对Node节点添加一个是不是叶子节点的方法)，然后计算每一个点到`query`点的距离

添加方法is_leaf() (通过判定这个区域是否有切割先来判定是不是叶子节点)

```
class Node:
	def __init__(self, axis, value, left, right, point_indices):
		self.axis = axis
        self.value = value
        self.left = left
        self.right = right
        self.point_indices = point_indices

    def is_leaf(self):                                                                                                            
        if self.value is None:
            return True
        else:
            return False
```

计算叶子节点中的点到query点的距离

```
def kdtree_knn_search(root: Node, db: np.ndarray, result_set: KNNResultSet, query: np.ndarray):
     if root.is_leaf():
        # compare the contents of a leaf
        leaf_points = db[root.point_indices, :]
        diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
        for i in range(diff.shape[0]):
            result_set.add_point(diff[i], root.point_indices[i])
            return False
    # 判断query点和分割点的距离
    if query[root.axis] < root.value:
        kdtree_knn_search(root.left, db, result_set, query)
    else:
        kdtree_knn_search(root.right, db, result_set, query)
    return False
```

在这里，我们做了一个保存结果的集合`result_set`, 保证可以留下距离最小的K个数。但是这个只是计算了和`query`点在同一个区域内的点，其它区域内的点有可能比这些点更近。

![图4](fig4.jpg)

**但是需要注意**，虽然红点`query`点在左上角的区域内，但是在同一个区域内的却不是最近的！！而是在红色所在区域之外的点

我们在添加点到结果集`result_set`的时候可以保存下这些点的最远距离`worst_distance`，当我们回溯的时候，再去计算每一个区域距离`query`的距离，如果区域距离`query`点小于`worst_distance`，则再去那个区域寻找有没有比较近的点。

代码可以修改为

```
151 def kdtree_knn_search(root: Node, db: np.ndarray, result_set: KNNResultSet, query: np.ndarray):
152     if root is None:
153         return False
154 
155     if root.is_leaf():
156         # compare the contents of a leaf
157         leaf_points = db[root.point_indices, :]                                                                                          
158         diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
159         for i in range(diff.shape[0]):
160             result_set.add_point(diff[i], root.point_indices[i])
161         return False
162 
163     # 作业2
164     # 提示：仍通过递归的方式实现搜索
165     # 对比一下query的值
166     if query[root.axis] < root.value:
167         kdtree_knn_search(root.left, db, result_set, query)
168         if math.fabs(root.value - query[root.axis]) < result_set.worstDist():
169             kdtree_knn_search(root.right, db, result_set, query)
170     else:
171         kdtree_knn_search(root.right, db, result_set, query)
172         if math.fabs(root.value - query[root.axis]) < result_set.worstDist():
173             kdtree_knn_search(root.left, db, result_set, query)
174 
175     return False
```

假设现在图4中g所在的区域找完了，返回了167行，这个时候就需要去求一下切割线距离`query`点的距离，小于g到`query`的距离，即168行的判定，如果为true，则进入切割线的另外一边，进入169行。

重新进入`kdtree_knn_search`函数，区域为如下灰色部分，

![img](fig5.jpg)

然后一样的判断不是叶子节点，判断切割的x轴的值小于query的x轴`if query[root.axis] < root.value:`，进入灰色区域右边171行

![img](fig6.jpg)

向结果集添加点, 可以通过我们的结果集来计算到底要不要加。如果要加的话，要更新`worst_ditance`。加完后171行结束，判断到query点的距离是否小于`worst_ditance`, 即172行。。。如此回到跟根节点，计算结束

下面看一下对于结果集的定义, 简单来说，找了一个`KNNResultSet.dist_index_list`来存储结果，里面的每一个元素都是一个`DistIndex`对象，来保存着距离和点的序号

每一次add的操作，就会去更新这个列表，如果加入的点的距离小于第K个点（就是最后一个点）的距离，则更新列表，并且重新排序即可。

```
 6 class DistIndex:
 7     def __init__(self, distance, index):
 8         self.distance = distance
 9         self.index = index
10 
11     def __lt__(self, other):
12         return self.distance < other.distance
13 
14
15 class KNNResultSet:                                                                                                                      
16     def __init__(self, capacity):
17         self.capacity = capacity
18         self.count = 0
19         self.worst_dist = 1e10
20         self.dist_index_list = []
21         for i in range(capacity):
22             self.dist_index_list.append(DistIndex(self.worst_dist, 0))
23 
24         self.comparison_counter = 0
25
26     def size(self):
27         return self.count
28 
29     def full(self):
30         return self.count == self.capacity
31 
32     def worstDist(self):
33         return self.worst_dist
34 
35     def add_point(self, dist, index):
36         self.comparison_counter += 1
37         if dist > self.worst_dist:
38             return
39 
40         if self.count < self.capacity:
41             self.count += 1
42 
43         i = self.count - 1
44         while i > 0:
45             if self.dist_index_list[i-1].distance > dist:
46                 self.dist_index_list[i] = copy.deepcopy(self.dist_index_list[i-1])
47                 i -= 1
48             else:
49                 break
50 
51         self.dist_index_list[i].distance = dist
52         self.dist_index_list[i].index = index
53         self.worst_dist = self.dist_index_list[self.capacity-1].distance
```

## 3. 运行代码

最后给一个可以运行的代码

```
216     db_size = 64
217     dim = 3
218     leaf_size = 4
219     k = 2
220     
221     db_np = np.random.rand(db_size, dim)
223     db_np = np.concatenate((db_np, np.array([[0.1,0.2,0.3],[0.1,0.2,0.3]])), axis=0)
224     root = kdtree_construction(db_np, leaf_size=leaf_size)
225 

232     query = np.asarray([0.1, 0.2, 0.3])
233     result_set = KNNResultSet(capacity=k)
234     kdtree_knn_search(root, db_np, result_set, query)
```