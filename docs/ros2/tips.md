---
title: ROS 2 小技巧
description: 一些好用的指令或小技巧
keywords:
  - ROS 2
---

## Topic Remapping

有時候我們會希望把某些程式內的 node 或 topic 改個名稱，但又不希望還要重新編譯，這時候可以使用 remapping 的技巧。
這邊是[官方相關的教學](https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html#name-remapping)

```bash
# 更改 topic 的名稱
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=my_topic
# 更改 node 的名稱
ros2 run demo_nodes_cpp talker --ros-args -r __node:=my_talker
# 增加 namespace
ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/demo
```
