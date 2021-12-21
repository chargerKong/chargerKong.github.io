---
title: ceres介绍

date: 2021-12-02 18:38:29
tags: ceres
---

# ceres 是干什么的

**非线性优化**，求函数的最小值，一般都是求 $\sum（*）^2$

比如最小二乘迭代求解



# 举例

前两个例子来自于 https://blog.csdn.net/cqrtxwd/article/details/78956227

## 例一

求
$$
\min_x(10-x)^2
$$
当给一个初值，比如$x_0=2$，最后可以迭代出结果，x=10;



需要构建的代价函数（cost function，然后添加到problem里面，这里添加了一次

```c++
problem.AddResidualBlock(cost_function, NULL, &x);
```

很明显，这里x就是需要求的变量



**全部代码**

```c++
#include<iostream>
#include<ceres/ceres.h>

using namespace std;
using namespace ceres;

//第一部分：构建代价函数，重载（）符号，仿函数的小技巧
struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};

//主函数
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // 寻优参数x的初始值，为5
  double initial_x = 5.0;
  double x = initial_x;

  // 第二部分：构建寻优问题
Problem problem;
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor); //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度。
  problem.AddResidualBlock(cost_function, NULL, &x); //向问题中添加误差项，本问题比较简单，添加一个就行。

  //第三部分： 配置并运行求解器
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
  options.minimizer_progress_to_stdout = true;//输出到cout
  Solver::Summary summary;//优化信息
  Solve(options, &problem, &summary);//求解!!!

  std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
//最终结果
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
————————————————
版权声明：本文为CSDN博主「福尔摩睿」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/cqrtxwd/article/details/78956227
```

CMakeLists.txt

```
//CMakeLists.txt：
cmake_minimum_required(VERSION 2.8)
project(ceres)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(use_ceres use_ceres.cpp)
target_link_libraries(use_ceres ${CERES_LIBRARIES})
————————————————
版权声明：本文为CSDN博主「福尔摩睿」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/cqrtxwd/article/details/78956227
```

## 例二

拟合一个非线性函数曲线
$$
y=e^{ax^2+bx+c}
$$


假设现在有1000个点$(x_i,y_i)$，那么误差函数，或者代价函数（cost function）为
$$
\min_{a,b,c}\sum_{i=1}^{1000}||y_i-y(x_i)||^2
$$
这里和上面不同的地方，Problem需要添加1000次残差块

```c++
  ceres::Problem problem;
  for(int i=0;i<1000;i++)
  {
//自动求导法，输出维度1，输入维度3,
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
        new CURVE_FITTING_COST(x_data[i],y_data[i])
      ),
      nullptr,
      abc
    );
  }
————————————————
版权声明：本文为CSDN博主「福尔摩睿」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/cqrtxwd/article/details/78956227
```

这里是直接把$x_i，y_i$的值通过构造函数传进去了，

```c++
struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x,double y):_x(x),_y(y){}
  template <typename T>
  bool operator()(const T* const abc,T* residual)const
  {
    residual[0]=_y-ceres::exp(abc[0]*_x*_x+abc[1]*_x+abc[2]);
    return true;
  }
  const double _x,_y;
};
————————————————
版权声明：本文为CSDN博主「福尔摩睿」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/cqrtxwd/article/details/78956227
```



全部代码

```c++
#include<iostream>
#include<opencv2/core/core.hpp>
#include<ceres/ceres.h>
using namespace std;
using namespace cv;

//构建代价函数结构体，abc为待优化参数，residual为残差。
struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x,double y):_x(x),_y(y){}
  template <typename T>
  bool operator()(const T* const abc,T* residual)const
  {
    residual[0]=_y-ceres::exp(abc[0]*_x*_x+abc[1]*_x+abc[2]);
    return true;
  }
  const double _x,_y;
};

//主函数
int main()
{
  //参数初始化设置，abc初始化为0，白噪声方差为1（使用OpenCV的随机数产生器）。
  double a=3,b=2,c=1;
  double w=1;
  RNG rng;
  double abc[3]={0,0,0};

//生成待拟合曲线的数据散点，储存在Vector里，x_data，y_data。
  vector<double> x_data,y_data;
  for(int i=0;i<1000;i++)
  {
    double x=i/1000.0;
    x_data.push_back(x);
    y_data.push_back(exp(a*x*x+b*x+c)+rng.gaussian(w));
  }

//反复使用AddResidualBlock方法（逐个散点，反复1000次）
//将每个点的残差累计求和构建最小二乘优化式
//不使用核函数，待优化参数是abc
  ceres::Problem problem;
  for(int i=0;i<1000;i++)
  {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
        new CURVE_FITTING_COST(x_data[i],y_data[i])
      ),
      nullptr,
      abc
    );
  }

//配置求解器并求解，输出结果
  ceres::Solver::Options options;
  options.linear_solver_type=ceres::DENSE_QR;
  options.minimizer_progress_to_stdout=true;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  cout<<"a= "<<abc[0]<<endl;
  cout<<"b= "<<abc[1]<<endl;
  cout<<"c= "<<abc[2]<<endl;
return 0;
}
}
————————————————
版权声明：本文为CSDN博主「福尔摩睿」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/cqrtxwd/article/details/78956227
```

# 核函数

求解优化问题中（比如拟合曲线），数据中往往会有离群点、错误值什么的，最终得到的寻优结果很容易受到影响，此时就可以使用一些损失核函数来对离群点的影响加以消除。要使用核函数，只需要把上述代码中的NULL或nullptr换成损失核函数结构体的实例。
Ceres库中提供的核函数主要有：TrivialLoss 、HuberLoss、 SoftLOneLoss 、 CauchyLoss。
比如此时要使用CauchyLoss，只需要将nullptr换成new CauchyLoss(0.5)就行（0.5为参数）。
下面两图别为Ceres官网上的例程的结果，可以明显看出使用损失核函数之后的曲线收离群点的影响更小。

![不使用鲁棒核函数的拟合](https://img-blog.csdn.net/20180102215946340?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvY3FydHh3ZA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

![使用鲁棒核函数的拟合](https://img-blog.csdn.net/20180102220033689?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvY3FydHh3ZA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)