---
title: ceres介绍
date: 2021-12-02 18:38:29
tags: ceres
---

# ceres 是干什么的

**非线性优化**，求函数的最小值，一般都是求 $\sum（*）^2$

比如最小二乘迭代求解

# ceres 怎么用

对于一个优化问题，通常是求能使得一些平方和达到最小值的参数为多少，利用ceres求解一个问题，通常分为以下几步

## 构建问题器对象

```c++
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

Problem problem;
```

## 构建参数块（可选），告诉ceres参数如何更新

```c++
  // 传入的是x的地址，x维度是2，第三个参数表示x的增加方式
  problem.AddParameterBlock(x, 2,
   (new ceres::AutoDiffLocalParameterization
    <TestLocalParamterization, 2, 2>())); // 2 表示真正的维度， 第二个2表示使用的维度
```

TestLocalParamterization的声明和定义如下, 必须要返回true, 

结构体中必须重载()运算符，第一个参数为变量，第二个参数为delta，第三个参数为更新结果

```c++
struct TestLocalParamterization {
  template <typename T>
  bool operator() (const T* x, const T* delta, T* x_plus_data) const {
    x_plus_data[0] = x[0] + delta[0];
    x_plus_data[1] = x[1] + delta[1];
    return true;
  }
};
```

## 构建残差块

AddResidualBlock 的参数，按照顺序

- 代价函数：定义残差的位置
- 核函数：暂时不用，使用Null。
- 变量

AutoDiffCostFunction 的参数模板参数第一个为输出的维度，第二个输入的维度

```c++
 CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 2, 2>(new CostFunctor);
problem.AddResidualBlock(cost_function, NULL, x);
```

要添加参加块，首先是要声明定义好残差块

```c++
struct CostFunctor {
  template <typename T> 
  bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    residual[1] = 10.0 - x[1]; // 优化两个值
    // residual[1] = T(0.0);      // 如果不优化第二个值，则这么写
    return true;
  }
};
```

## 计算优化器

```c++
  // Run the solver!
  Solver::Options options;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
```

此时传入的x 已经被优化完毕

# 举例

前两个例子来自于 https://blog.csdn.net/cqrtxwd/article/details/78956227

## 例一

求
$$
\min_x(10-x)^2
$$
当给一个初值，比如$x_0=2$，最后可以迭代出结果，x=10;

### ceres 使用

例一的问题输入其实就俩

- 定义残差函数
- 定义初始值

输出就是迭代结果，10。下面重点关注如何定义这两部

#### 定义优化函数

最关心的一步为`residual[0] = T(10.0) - x[0];` ceres会自动加上平方

```c++
//第一部分：构建代价函数，重载（）符号，仿函数的小技巧
struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};
```

#### 定义初始

```c++
  // 寻优参数x的初始值，为5
  double initial_x = 5.0;
```

#### 代码模板

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

  // 第二部分：构建寻优问题，就是上面定义的CostFunctor,
  // 输入到模板中
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

需要构建的代价函数（cost function，然后添加到problem里面，这里添加了一次

```c++
problem.AddResidualBlock(cost_function, NULL, &x);
```

很明显，这里x就是需要求的变量

#### CMakeLists.txt

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

$$
\min_{x,y}||(10,10)-(x,y)||^2
$$

例二和例一的不同在于，自变量变成了两维的，这里还添加了如何增加参数块，更新的方法

### 代码模板

```c++
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <iostream>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
  template <typename T> 
  bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    residual[1] = 10.0 - x[1]; // 优化两个值
    // residual[1] = T(0.0);      // 不优化第二个值 
    return true;
  }
};

struct TestLocalParamterization {
  template <typename T>
  bool operator() (const T* x, const T* delta, T* x_plus_data) const {
    x_plus_data[0] = x[0] + delta[0];
    x_plus_data[1] = x[1] + delta[1];
    return true;
  }
};


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x[] = {0.5, 2.0};
  // const double initial_x[] = x;
  for (auto &i: x) {
    std::cout << i << " ";
  }
  std::cout << "\n" << std::endl;
  // Build the problem.
  Problem problem;

  // 传入的是x的地址，维度是2，第三个参数表示x的增加方式
  problem.AddParameterBlock(x, 2,
   (new ceres::AutoDiffLocalParameterization
    <TestLocalParamterization, 2, 2>())); // 2 表示真正的维度， 第二个2表示使用的维度

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 2, 2>(new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, x);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  for (auto &i: x) {
    std::cout << i << " ";
  }
  std::cout << "\n" << std::endl;

  return 0;
}
```





## 例三

拟合一个非线性函数曲线
$$
y=e^{ax^2+bx+c}
$$


假设现在有1000个点$(x_i,y_i)$，那么误差函数，或者代价函数（cost function）为
$$
\min_{a,b,c}\sum_{i=1}^{1000}||y_i-y(x_i)||^2
$$
该案例和上面不同的地点

- 待求变量：从例一的一维x变成了例二的三维[a,b,c]
- 求和的项：从原来的一项，变成现在的1000项

### 定义优化函数

这里，我们定义好$y_i-y(x_i)$，以便后续计算1000次，不同的$x_i，y_i$只需要通过构造函数传入就行

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

### 把函数放入问题中problem.AddResidualBlock

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

### 代码模板

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