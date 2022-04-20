---
title: absl::optional 使用
date: 2022-03-03 18:38:29
tags: c++
---

absl::optional 是一个可有可无的数据结构，

## 声明

下面这一行表示声明了一个模板类型为double的变量

```c++
absl::optional<double> last_thread_cpu_time_seconds_;
```



## 使用

### has_value()

```c++
if (last_thread_cpu_time_seconds_.has_value()) {
	...
}
```



### 赋值

直接赋值double类型的变量

```
last_thread_cpu_time_seconds_ = 3.0
```



### 取值value()

```c++
double new_value = last_thread_cpu_time_seconds_.value()
```

