---
title: typing
date: 2021-04-07 16:24:35
tags: Python
---

```
from typing import Union
```

`typing.Union`

联合类型；`Union[X, Y]` 的意思是，非 X 即 Y。

可用 `Union[int, str]` 等形式定义联合类型。 具体如下：

- 参数必须是某种类型，且至少有一个。

- 联合类型之联合类型会被展平，例如：

  ```
  Union[Union[int, str], float] == Union[int, str, float]
  ```

- 单参数之联合类型就是该参数自身，例如：

  ```
  Union[int] == int  # The constructor actually returns int
  ```

- 冗余的参数会被跳过，例如：

  ```
  Union[int, str, int] == Union[int, str]
  ```

- 比较联合类型，不涉及参数顺序，例如：

  ```
  Union[int, str] == Union[str, int]
  ```

- 联合类型不能作为子类，也不能实例化。

- 不支持 `Union[X][Y]` 这种写法。

- `Optional[X]` 是 `Union[X, None]` 的缩写。

