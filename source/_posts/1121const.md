---
title: const c++
date: 2021-11-20 18:38:29
tags: c++
---

```c++
#include <iostream>
int main()
{
	// const int a 与 int const a  是一样的，都表示 a 不可以被改变
	const int buffersize = 512;
	// buffersize = 10; 错误
	int a = 1;
	int b = 2;
	//==============const int* p ======================
	// 指向的内容是一个常量
	const int* p = &a;
	// *p = 2; 错误，无法通过p改变其 所指向的内容
	std::cout << "now p point to a: " << *p << std::endl;
	p = &b; // 但是可以改变p的指向
	std::cout << "now p point to b: " << *p << std::endl;

	//==============int const* p ======================
	// 和
	int const* p1 = &a;
	// *p1 = 2; 错误，无法通过p改变其 所指向的内容
	std::cout << "now p1 point to a: " << *p1 << std::endl;
	p1 = &b; // 但是可以改变p的指向
	std::cout << "now p point to b: " << *p1 << std::endl;

	//==============int* const p ======================
	// p2本身是一个常量，p2的地址不可以改变，指向的内容可以变化
	int* const p2 = &a;
	*p2 = b;
	// p2 = &a; 报错，不可以改变地址

	/*
	总结： 看*后边的内容
	(const int) *p 表示p是一个指针，*p的内容不可以变
	int *(const p) 表示const p 是一个指针，指针本身不可以变化
	*/

	return 0;
}

```

