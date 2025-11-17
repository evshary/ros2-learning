---
title: ROS 2 好用工具
description: 介紹推薦的 ROS 2 工具，包含 PlotJuggler
keywords:
  - ROS 2
  - 機器人
  - 工具
---

## PlotJuggler

[PlotJuggler](https://plotjuggler.io/) 可以用 GUI 的方式顯示 topic 數值隨時間的變化，他是開源的工具，程式碼可以在 [GitHub](https://github.com/facontidavide/PlotJuggler) 找到。
如果有比較不同數值(只能是數值)之間隨時間的變化趨勢，可以用這個工具來觀察。

* 使用方式

```bash
# 安裝
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
# 使用
ros2 run plotjuggler plotjuggler
```

在使用上基本上就是 UI 的拖拉而已，可以建議大家跑個 turtlesim 來觀察。
不過有個小地方要特別注意，如果你想要在 X 軸和 Y 軸各放上不同 topic 數值，記得拖拉時按著右鍵，這樣才能在不同軸放不同數值。

## domain_bridge

我們知道 ROS 支援 ROS_DOMAIN_ID 來避免不同環境的 ROS 流量互相影響。
然而有時候會希望可以某些在不同 Domain ID 的 topic 可以溝通，這時候就可以使用 [domain_bridge](https://github.com/ros2/domain_bridge/tree/main)。
細節設計可以參考[官方文件](https://github.com/ros2/domain_bridge/blob/main/doc/design.md)，
你也可以到 `/opt/ros/$ROS_DISTRO/share/domain_bridge/examples/` 參考範例設定檔。

* 安裝

```bash
sudo apt install ros-$ROS_DISTRO-domain-bridge
```

* 創一個簡單設定檔 `example_bridge_config.yaml`

```yaml
name: my_bridge
from_domain: 2
to_domain: 3
topics:
  chatter:
    type: std_msgs/msg/String
```

* 使用

```bash
ROS_DOMAIN_ID=2 ros2 run demo_nodes_cpp talker
ROS_DOMAIN_ID=3 ros2 run demo_nodes_cpp listener
ros2 run domain_bridge domain_bridge example_bridge_config.yaml
```
