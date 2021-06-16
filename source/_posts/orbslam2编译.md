---
title: 修改orbslam2 OpenCV版本
date: 2021-05-30 16:24:35
---

## 修改CMakeLists

修改主工程下的CMakeLists.txt以及Thirdparty/g2o/CMakeLists.txt

```
 31 find_package(OpenCV)
```

把后面的3.0.0和QUIET去掉

## 修改cv.h 头文件

打开文件include/ORBextractor.h

把头文件

```
#include <opencv/cv.h>
```

改为

```
#include <opencv2/opencv.hpp>
```

## 添加namespace

打开include/PnPsolver.h

添加

```
using namespace cv;
```



##  error "Allocator::value_type must be same type as value_type"

修改文件`/include/LoopClosing.h`

把文件

```
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```

改为

```
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;
```



## usleep 报错

所有usleep报错的地方在文件头部添加

```
#include <unistd.h>
```



## CV_LOAD_IMAGE_UNCHANGED 报错

在相应报错的文件中，把`CV_LOAD_IMAGE_UNCHANGED` 改为 `cv::IMREAD_UNCHANGED`



## error: ‘GRAY2BGR’ was not declared in this scope

相应文件中的，GRAY2BGR改为`cv::COLOR_GRAY2BGR`

