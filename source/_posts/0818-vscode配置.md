---
title: vscode配置-cartographer
date: 2021-09-05 15:59:22
tags: cartographer
---

## VScode简单配置

快捷键编译：Ctrl+shift+b

编译配置: tasks.json

```
{
    // See https://go.microsoft.com/fwlink/?LinkId=733558 
    // for the documentation about the tasks.json format
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make",
            "type": "shell",
            "command": "catkin_make_isolated --install --use-ninja", // catkin_make
            "args": [],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "presentation": {
                "reveal": "always"
            },
            "problemMatcher": "$msCompile"
        },
    ]
}
```

这里的command就是编译cartographer的命令



## Cartographer之CMakeLIsts

读代码，最好从CMakeLists.txt开始

