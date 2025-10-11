---
title: ROS 2 錄製與重放
description: ROS 2 如何紀錄以及重放封包
keywords:
  - ROS 2
  - 機器人
---

在機器人開發中，錄製封包的功能可以在很多方面幫助我們。
舉例來說，如果某個問題的重現很困難，我們可以透過錄製當下發生的封包來更容易復現。
又或者是說，我們可以錄製實際機器人或是自駕車的感測器資訊，像是相機或是光達，然後讓開發者們可以在沒有實際硬體的情況下開發相對應的模組。

在網路世界中，我們是使用 Wireshark 來錄製封包，但是在機器人領域中，最小的封包單位是一個個 ROS Message，因此需要有其他更適合的工具，也就是這邊要談到的 ROS bag。

## 使用方式

ROS bag 是 ROS CLI 指令下的其中一個部份，我們可以用 `ros2 bag --help` 來看到更多指令的用法。

* 錄製封包

```bash
# 錄製所有 topic 的封包
ros2 bag record --all-topics
# 錄製特定 topic 的封包，例如 mytopic1 和 mytopic2
ros2 bag record --topics mytopic1 mytopic2
```

* 查看封包資訊：錄完後可以看到類似 `rosbag2_2025_10_11-10_38_16` 的資料夾，可以查看其內容

```bash
$ ros2 bag info rosbag2_2025_10_11-10_38_16
Files:             rosbag2_2025_10_11-10_38_16_0.mcap
Bag size:          5.3 KiB
Storage id:        mcap
ROS Distro:        rolling
Duration:          2.999745724s
Start:             Oct 11 2025 10:38:17.945157770 (1760150297.945157770)
End:               Oct 11 2025 10:38:20.944903494 (1760150300.944903494)
Messages:          4
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 4 | Serialization Format: cdr
Services:          0
Service information: 
Actions:           0
Action information: 
```

* 重放封包：會重放當初錄製的內容

```bash
ros2 bag play rosbag2_2025_10_11-10_38_16
```

## ROS bag 格式

那麼實際上 ROS bag 是用什麼格式錄製呢？
最早是使用 SQLite3 的方式，但在 [ROS 2 Iron 以後](https://discourse.openrobotics.org/t/psa-default-ros-2-bag-storage-format-is-changing-to-mcap-in-iron/28489)，預設的格式改為功能更為完整的 MCAP 格式。
MCAP 最大的好處在於有較佳的 throughput、能自行包含 metadata 來描述自我資訊、提供更多的功能(如更好的 compression 等等)。

## 參考資料

* ROSCon talk: MCAP: A Next-Generation File Format for ROS Recording
    * [slides](http://download.ros.org/downloads/roscon/2022/MCAP%20A%20Next-Generation%20File%20Format%20for%20ROS%20Recording.pdf)
    * [video](https://vimeo.com/showcase/9954564?video=767149733)
