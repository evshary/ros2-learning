---
title: Autoware API
description: 控制 Autoware 的 API
keywords:
  - Autoware
  - 自駕車
---

Autoware 本身有提供一些 API 給外部使用者使用，這些 API 基本上都是 ROS 2 topic 的形式。
相關資料可以參考[官方文件](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/#architecture)

這邊會列上一些常用的 API：

* 控制車子的操作模式 (Operation Mode)

```bash
# 讓車子停下
ros2 service call /api/operation_mode/change_to_stop autoware_adapi_v1_msgs/srv/ChangeOperationMode
# 讓車子回復到導航模式
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode
```

* 追蹤車子當下位置

```bash
# 會提供當下經緯度以及 map 上的位置
ros2 topic echo /api/vehicle/kinematics
```

* 路線相關

```bash
# 取得當下的路線，如果為空，代表沒有設定任何路線
ros2 topic echo /api/routing/route
```

* rviz 相關

```bash
# 取得 rviz 給的初始化位置
ros2 topic echo /initialpose
```
