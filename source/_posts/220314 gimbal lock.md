---
title: 最好的gimbal lock讲解
date: 2022-03-14 18:38:29
tags: math
---

https://krasjet.github.io/quaternion/bonus_gimbal_lock.pdf

gimbal lock 万向锁：他锁的是三维旋转的其中一个维度。如果某一个维度的旋转恰好和其他维度重合了，那么其中一个维度的旋转就失效了

如，旋转顺序是xyz轴，绕着x轴任意旋转后，绕着y轴顺时针旋转90度，自身的则z轴会和x轴重复，也就是说最后 𝑧 轴的旋转与 𝑥 轴的旋转 其实操纵的是同一个轴。

