---
title: Linux 通訊機制
description: 簡介 Linux process 間以及 userspace 與 kernel 間的通訊機制
keywords:
  - Linux
  - IPC
  - inter-process communication
  - kernel
  - userspace
---

Linux 系統中的通訊不只發生在 process 之間，也包含 userspace application 與 kernel 之間的資料交換。
兩者面對的問題與適合的介面並不相同：

* Process 間通訊（Inter-Process Communication，IPC）讓各自擁有獨立記憶體空間的 process 交換資料、傳遞事件或協調工作。
* Userspace 與 kernel 通訊則讓 application 取得系統狀態、控制 device driver，或接收 kernel 產生的事件。

選擇通訊機制時，需要考量通訊對象、資料量、訊息邊界、效能、同步方式、權限，以及介面是否需要維持穩定的 ABI。

## Process 間通訊

IPC 機制在通訊範圍、資料模型與使用複雜度上各有差異。

Pipe/FIFO 適合簡單資料流，Socket 適合通用雙向通訊，Message Queue 適合獨立訊息，Shared Memory 適合大量低延遲資料，D-Bus 適合高階系統服務，而 Netlink 主要用於 userspace 與 kernel 溝通。

| IPC 機制 | 通訊範圍／方式 | 資料與通訊模型 | 優點 | 缺點 | 簡單範例 |
| --- | --- | --- | --- | --- | --- |
| **Pipe** | 本機；通常是有親緣關係的 process | 單向 byte stream | 簡單、輕量；適合串接程式 | 通常只能單向；沒有訊息邊界；通常需要 parent-child 關係 | `ps aux \| grep nginx` |
| **FIFO**<br>(Named Pipe) | 本機；透過 filesystem path 連接 | 單向 byte stream | 無親緣關係也能使用；操作方式類似檔案 | 雙向需要兩個 FIFO；不適合複雜的多 client 架構；需管理 path | `mkfifo /tmp/my_fifo` |
| **Socket** | Unix socket 用於本機；TCP/UDP 可跨主機 | 通常雙向；支援 stream、datagram 等模式 | 彈性高；支援多 client；可用於本機或網路 | API 與連線管理較複雜；stream 模式需自行處理訊息邊界 | Docker 使用 `/var/run/docker.sock` |
| **Message Queue** | 本機；透過 queue name 或 ID | Producer-consumer；保留訊息邊界 | 天然的訊息佇列；可阻塞等待；部分實作支援訊息優先權 | 訊息大小和 queue 容量有限；不適合傳送大量資料 | `mq_send()` / `mq_receive()` 傳送工作指令 |
| **Shared Memory**<br>(SHM) | 本機；多個 process 映射相同 memory pages | 直接共享自訂資料結構 | Throughput 高、延遲低；適合大量資料；減少資料複製 | 必須自行處理同步、資料格式和生命週期；容易發生 race condition | `shm_open()` + `mmap()` 共享影像 buffer |
| **D-Bus** | 本機；應用程式透過 system/session bus 找到 service | 高階 message；支援 method、reply、signal | 支援 service discovery、權限與結構化介面；適合系統服務 | 額外抽象與處理成本；不適合高頻率或大型資料 | 呼叫 NetworkManager 查詢網路狀態 |

## Userspace 與 Kernel 通訊

Userspace application 無法直接存取 kernel memory 或硬體，必須透過 system call 和 kernel 提供的介面進行操作。
簡單的狀態與屬性通常透過虛擬檔案系統呈現；device driver 常使用 device file；需要結構化雙向通訊或非同步事件時，則可使用 Netlink。

| 介面／機制 | 主要用途 | 通訊方式 | 優點 | 缺點 | 簡單範例 |
| --- | --- | --- | --- | --- | --- |
| **`read()` / `write()` / `ioctl()`** | Userspace 操作 device driver | 透過 `/dev` device file；`read/write` 傳資料，`ioctl` 傳控制命令 | 標準且直接；適合 driver 資料與控制操作；可支援阻塞 I/O | 需要設計 userspace/kernel ABI；`ioctl` 過多時介面較難維護 | 對 `/dev/video0` 讀取影像，或用 `ioctl()` 設定裝置 |
| **procfs** | 提供 process 與系統執行狀態 | 透過 `/proc` 虛擬檔案，以 `read/write` 存取 | 容易用 shell 和一般工具查看；適合文字資訊 | 不適合高頻或大量資料；部分格式主要供人閱讀 | `cat /proc/meminfo` |
| **sysfs** | 顯示與設定 device、driver、bus 和 kernel object 屬性 | 透過 `/sys` 虛擬檔案，通常一個檔案代表一個屬性 | 結構清楚；適合裝置屬性與簡單設定；容易 script 化 | 不適合複雜 command 或大型資料；通常限制為簡單文字值 | `cat /sys/class/net/eth0/operstate` |
| **debugfs** | Kernel 和 driver 開發除錯 | 透過 `/sys/kernel/debug` 暴露自訂資訊或控制項目 | 彈性高；容易加入診斷與測試介面 | 通常沒有穩定 ABI 保證；不適合作為正式產品介面 | 查看 `/sys/kernel/debug/tracing/` |
| **Netlink** | Userspace 與 kernel subsystem 交換結構化命令與事件 | 使用 `AF_NETLINK` socket；支援 request-response 和 multicast | 雙向、結構化、可擴充；支援 kernel 主動發送事件 | Message protocol 較複雜；開發成本高於虛擬檔案 | `ip addr` 透過 Netlink 查詢網路介面 |
| **uevent + udev** | 處理 device 新增、移除與狀態改變 | Kernel 發送 uevent；userspace 的 `systemd-udevd` 接收並套用規則 | 適合 device lifecycle；可自動建立裝置節點、權限及 symlink | 用途集中在 device model；udev 規則複雜時較難除錯 | 插入 USB 後建立 `/dev/sdb` |
