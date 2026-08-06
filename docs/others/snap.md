---
title: snap 機制
description: 簡介 Ubuntu snap 封裝系統
keywords:
  - Linux
  - Ubuntu
---

Snap 是 Canonical 推出的 Linux 軟體封裝、安裝與更新系統。
每個 .snap 套件包含應用程式及大部分相依檔案，並透過 sandbox 限制程式能存取的系統資源。

常用到的工具有

* snap: 使用者操作的 CLI
* snapd: 負責安裝、更新、掛載及安全政策的 daemon
* Snap Store: 軟體套件來源
* Snapcraft: 開發者製作 Snap 的工具

## 與其他工具的差異

| 工具 | 主要定位 | 相依套件 | Sandbox |
| - | - | - | ------- |
| deb／apt | Ubuntu 原生系統套件 | 通常共用系統函式庫 | 預設沒有 |
| snap | 跨發行版的應用程式、CLI、服務及 IoT 套件 | 內含或使用 base snap | 有 |
| flatpak | 跨發行版的桌面應用程式 | 使用共用 runtime | 有 |
| AppImage | 下載後直接執行的單一檔案 | 通常包含在檔案內 | 預設沒有 |
| homebrew | 安裝 CLI、開發工具及函式庫 | 由 Homebrew 管理 | 沒有 |

簡單定位

* 系統、driver、底層函式庫：APT
* 桌面應用程式：snap 或 flatpak
* 單檔直接執行：AppImage
* CLI／開發工具：apt、snap 或 homebrew

## Snap 格式

`.snap` 是一個唯讀、壓縮的 SquashFS 檔案系統映像。

內容大致包括：

* 執行檔
* 函式庫
* 資源及設定檔
* meta/snap.yaml
    * Snap 名稱及版本
    * 執行入口
    * Base snap
    * Apps 和 services
    * Plugs 和 slots
    * Confinement 模式
        * strict: 正常執行 sandbox 限制
        * devmode: 記錄違規，主要供開發除錯
        * classic: 接近傳統應用程式，能廣泛存取主機

安裝後通常掛載於 `/snap/<name>/<revision>/`

資料部份則分成兩個

* 使用者資料: `~/snap/<name>/`
* 系統或 service 資料: `/var/snap/<name>/`

## 常用

基本術語

* App: Snap 提供的程式或 service
* Base: 提供基本執行環境，例如 core24
* Interface: 一類系統資源及其安全政策，例如 home、network
* Plug: 資源需求端
* Slot: 資源提供端
* Connection: plug 與 slot 的實際連接
* Confinement: snap 的隔離等級
* Version: 應用程式自己的版本
* Revision: Snap Store 對每次上傳產生的編號
* Channel: 更新通道，例如 stable、beta、edge

最常用指令

* `snap find <name>`: 在 Snap Store 搜尋套件
* `sudo snap install <name>`: 從 Snap Store 下載並安裝套件
* `snap list`: 列出目前已安裝的 Snap
* `sudo snap refresh`: 更新所有已安裝的 Snap
* `sudo snap remove <name>`: 移除指定 Snap；預設可能保留資料 snapshot
* `snap connections <name>`: 查看指定 Snap 的 plugs、slots 及目前連接狀態

## 啟動 snap 流程

* `snapd`: 主要負責事前的安裝更新
* 使用者執行程式
* `/snap/bin/<app>`
* `snap run`: 讀取 metadata，準備啟動
* `snap-confine`: 建立 sandbox、mount namespace，套用安全政策
* `snap-exec`: 找到實際 command，最後執行應用程式
* 真正的應用程式

## 確保安全性的方法

* SquashFS: 應用程式本體唯讀、不可修改
* Mount namespace: 控制程式看到的檔案系統
* AppArmor: 限制可存取的檔案、socket、D-Bus 等資源
* Seccomp: 限制可使用的 system calls
* Capabilities: 限制 process 的系統特權
* cgroup／udev: 管理 service 及硬體裝置存取
* UNIX permissions: UID、GID、檔案權限仍然有效
* Interfaces: 以 plug／slot 開放必要資源
