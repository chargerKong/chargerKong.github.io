---
title: loops and 条件语句
date: 2022-04-01 18:38:29
tags: c++
---

# 三目运算符转换

```
factor = isPositive ? 1 : -1;
```

等价于

```
if (isPositive)
	factor = 1
else
	factor = -1
```

等价于

```
factor = isPositive * 2 - 1
```

最后这个是最快的，没有跳转只有计算



# while 

```c++
size_t num = 10
while (num >= 0) {
	cout << num << endl
	num--
}
```

会发现他会不停的打印，因为size_t的定义本身就是大于等于0，0减去1会得到最大的那个正整数



# 赋值

```c++
int m = (b = 8)
```

b = 8 也是一个表达式，他返回值为8，所以m = 8

