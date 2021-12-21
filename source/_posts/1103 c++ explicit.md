---
title: explicit 使用
date: 2021-11-03 18:38:29
tags: c++
---

在构造函数前面加上explicit，可以防止隐式调用带来的非预期的类型转换
下面的例子介绍了什么是隐式调用

```c++
#include <iostream>
using namespace std;

// 在构造函数前面加上explicit，可以防止隐式调用带来的非预期的类型转换
// 下面的例子介绍了什么是隐式调用
class Point {
public:
    int x, y;
    Point(int x = 0, int y = 0)
        : x(x), y(y) {
            std::cout << "have been called" << std::endl;
        }
};

void displayPoint(const Point& p) 
{
    cout << "(" << p.x << "," 
         << p.y << ")" << endl;
}

int main()
{
    displayPoint(1); //这里可以运行成功，间接运行了Point p = 1，输出(1,0)。即隐式调用。
    Point p = 1;

    // 下面是显式调用
    Point q(1, 0);
}
```

