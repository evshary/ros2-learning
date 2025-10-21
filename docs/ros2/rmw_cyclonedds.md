---
title: rmw_cyclonedds 介紹
description: 如何在 ROS 2 中使用 rmw_cyclonedds
keywords:
  - DDS
  - CycloneDDS
  - ROS 2
  - 機器人
  - middleware
---

目前在 ROS 2 中，DDS 的通訊方式主要有兩種實作：FastDDS 和 CycloneDDS。
官方預設是使用 FastDDS，但是我們也可以輕易地切換到 CycloneDDS。

## 安裝與使用

* 先安裝 `rmw_cyclonedds`，也會順便安裝 CycloneDDS 本身

```bash
# 依據自己調整 ROS 的版本，例如 rolling 就是 ros-rolling-cyclonedds-cpp
sudo apt install ros-${ROS_DISTRO}-cyclonedds-cpp
```

* 設定 `RMW_IMPLEMENTATION` 使用 CycloneDDS

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

* 接下來呼叫 ROS 2 時，都會用 `rmw_cyclonedds` 了

!!! warning
    若是你發現依然是使用其他 DDS 時，有可能是 ros2 daemon 仍然在背景運行的問題。
    你可以直接將其關閉，在重新執行 ROS 2 指令即可。
    關閉方法：`ros2 daemon stop`

## 共享記憶體 (Shared Memory)

CycloneDDS 預設是使用 [iceoryx](https://projects.eclipse.org/projects/technology.iceoryx) 來達成共享記憶體傳輸。
從 ROS 2 Humble 之後，這個功能就已經內建於 rmw_cyclonedds 了，但值得注意的是目前只有 Linux 才支援。

* 建立 `cyclonedds.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
    <Domain id="any">
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>info</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
```

* 指定設定檔

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

* 啟動 RouDi，預設 ROS 2 環境應該已經有安裝了
    * 如果沒有啟動會出現這個錯誤 `[Warning]: RouDi not found - waiting`

```bash
iox-roudi
```

* 預設 RouDi 會根據 `/opt/ros/${ROS_DISTRO}/etc/roudi_config_example.toml` 來預先保留記憶體，但如果用超過的話就會出現錯誤。我們可以另外產生一份適合自己的設定檔。

```bash
# 假設 config 名稱為 roudi_config.toml
iox-roudi -c roudi_config.toml
```

相關細節可以參考[官方教學](https://github.com/ros2/rmw_cyclonedds/blob/rolling/shared_memory_support.md)
