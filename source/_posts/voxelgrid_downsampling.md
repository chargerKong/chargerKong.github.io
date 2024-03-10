---
title: Voxel_Grid_Downsampling
date: 2021-03-03 16:24:35
tags: 3D
---

点云往往比较密集，往往不需要这么密集的点，我们也可以看出这个点云是什么东西，或者能得到和密集点云一样的效果产出。下面两张图：第一张是原始的点云10000个点，第二张是下采样之后的点云1412个点。

![原始点云](fig.jpg)原始点云

![下采样点云](fig1.jpg)下采样点云

其实我们也可以通过下采样之后的点云看出，这是一架飞机。

下面介绍如何实现

1. 首先计算出飞机所在的三维空间，即
2. 给出方格的大小
3. 计算在每一个轴上能有多少方格。全部向上取整
4. 计算每一个点在各个方向上应该属于第几个方格



 全部向下取整

1. 把每一个点所在的网格位置对应到一个数上，相当于对网格做标号网格标号如下所示

   ![img](fig2.jpg)

2. 假设, 这代表的第一个点再第12个方格，第二个点再第10个方格。。。

   我们把原来的点按照这个小到大进行排序，，那么前面的几个点就会都在第一个方格，比如如下

   ，我们从所有的方格中只取一个，有两种方法

   - 随机选取，快，但是不准，如图

     ![方格随机取点下采样](fig1.jpg)方格随机取点下采样

   - 求平均的，慢但是精确一些

   ![方格求平均取点下采样](fig3.jpg)方格求平均取点下采样

```
import open3d as o3d 
import os
import numpy as np
from pyntcloud import PyntCloud
import random

def voxel_filter(point_cloud, leaf_size, method="rand"):
    """
    # 功能：对点云进行voxel滤波
    # 输入：
    #     point_cloud：输入点云
    #     leaf_size: voxel尺寸
    :param point_cloud:
    :param leaf_size:
    :return:
    """
    filtered_points = []

    xyz_max = np.max(point_cloud, axis=0)
    xyz_min = np.min(point_cloud, axis=0)
    # 计算格子的维度
    D_xyz = np.floor((xyz_max - xyz_min) / leaf_size) + 1
    # 计算每一个点的序号，x方向，y方向和z方向
    h_xyz = (point_cloud - xyz_min) // leaf_size
    # 计算每一个点应该在第几个格子，格子的序号从x轴的开始，满了就往y轴方向填，再往z轴方向填
    # idx_grid = [12345, 21, 0, 1...] 这代表第一个点应该再标号为12345的格子上
    idx_grid = h_xyz['x'] + h_xyz['y'] * D_xyz['x'] + h_xyz['z'] * D_xyz['x'] * D_xyz['y']
    # 按照格子的标号对原来的点进行排序，可以从中随机选取一个作为格子的点
    idx_grid = idx_grid.to_numpy()
    idx_i_grid = [(i, idx) for i, idx in enumerate(idx_grid)]
    idx_i_grid.sort(key=lambda x: x[1])
    # 下面这个排序之后大约就是[0,0,3,3,10,10,10...]，代表第一个和第二个点都在第一个格子里，
    # 第三个和第四个点都在第三个格子里，
    sort_point = point_cloud.to_numpy()[[i for i, j in idx_i_grid]]
    # 再每一个格子里面，随机选择一个点
    begin, end = 0, 0
    for end in range(1, len(idx_grid)):
        if idx_i_grid[end][1] != idx_i_grid[begin][1]:
            if method == "rand":
            ## random
                candid = get_point_random(sort_point, begin, end)
            elif method == "centroid":
            ## centroid
                candid = get_point_centroid(sort_point, begin, end)

            filtered_points.append(candid)
            begin = end

    # 把点云格式改成array，并对外返回
    filtered_points = np.array(filtered_points, dtype=np.float64)
    return filtered_points


def get_point_random(data, begin, end):
    idx = np.random.choice([i for i in range(begin, end + 1)])
    return data[idx]


def get_point_centroid(data, begin, end):
    sorted_list = data[begin: end + 1]
    return np.mean(sorted_list, axis=0)


def main():
    cat_index = 2  # 物体编号，范围是0-39，即对应数据集中40个物体
    root_dir = '/home/kong/下载/shenlan/3D数据集/modelnet40_normal_resampled'  # 数据集路径
    cat = os.listdir(root_dir)
    filename = os.path.join(root_dir, cat[cat_index], cat[cat_index] + '_0001.txt')  # 默认使用第一个点云

    # 加载原始点云
    point_cloud_pynt = PyntCloud.from_file(filename,
                                           sep=",",
                                           names=["x", "y", "z", "nx", "ny", "nz"])
    # 从点云中获取点，只对点进行处理
    points = point_cloud_pynt.points.iloc[:, :3]  # get x,y,z cols
    print('total points number is:', points.shape[0])
    point_cloud_pynt = PyntCloud(points)

    # 转成open3d能识别的格式
    point_cloud_o3d = point_cloud_pynt.to_instance("open3d", mesh=False)
    o3d.visualization.draw_geometries([point_cloud_o3d]) # 显示原始点云

    # 调用voxel滤波函数，实现滤波
    filtered_cloud = voxel_filter(point_cloud_pynt.points, 0.05, method="centroid")
    point_cloud_o3d.points = o3d.utility.Vector3dVector(filtered_cloud)
    print("after filter: ", filtered_cloud.shape[0])
    # 显示滤波后的点云
    o3d.visualization.draw_geometries([point_cloud_o3d])

if __name__ == '__main__':
    main()
```