---
title: Snap 概念
description: 介紹 Ubuntu snap 的格式、啟動流程、confinement、interface 與 channel
keywords:
  - Linux
  - Ubuntu
  - Snap
  - Confinement
  - Interface
---

Snap 是 Canonical 推出的 Linux 軟體封裝、安裝與更新系統。
每個 .snap 套件包含應用程式及大部分相依檔案，並透過 sandbox 限制程式能存取的系統資源。

常用到的工具有

- snap: 使用者操作的 CLI
- snapd: 負責安裝、更新、掛載及安全政策的 daemon
- Snap Store: 軟體套件來源
- Snapcraft: 開發者製作 Snap 的工具

本系列文章分成

- 本文：概念與機制
- [Snap 基本操作](usage.md)：安裝、更新、權限、資料路徑與疑難排解
- [Snapcraft 打包](snapcraft.md)：撰寫 `snapcraft.yaml` 並建置 snap
- [上架 Snap Store](store.md)：註冊名稱、上傳與發佈到 channel

## 與其他工具的差異

| 工具 | 主要定位 | 相依套件 | Sandbox |
| - | - | - | ------- |
| deb／apt | Ubuntu 原生系統套件 | 通常共用系統函式庫 | 預設沒有 |
| snap | 跨發行版的應用程式、CLI、服務及 IoT 套件 | 內含或使用 base snap | 有 |
| flatpak | 跨發行版的桌面應用程式 | 使用共用 runtime | 有 |
| AppImage | 下載後直接執行的單一檔案 | 通常包含在檔案內 | 預設沒有 |
| homebrew | 安裝 CLI、開發工具及函式庫 | 由 Homebrew 管理 | 沒有 |

簡單定位

- 系統、driver、底層函式庫：APT
- 桌面應用程式：snap 或 flatpak
- 單檔直接執行：AppImage
- CLI／開發工具：apt、snap 或 homebrew

## Snap 格式

`.snap` 是一個唯讀、壓縮的 SquashFS 檔案系統映像。

內容大致包括：

- 執行檔
- 函式庫
- 資源及設定檔
- meta/snap.yaml
    - Snap 名稱及版本
    - 執行入口
    - Base snap
    - Apps 和 services
    - Plugs 和 slots
    - Confinement 模式

安裝後通常掛載於 `/snap/<name>/<revision>/`，而 `/snap/<name>/current` 是指向目前 revision 的 symlink。
更新時 snapd 會掛載新的 revision 再切換 `current`，所以能快速回復到舊版。

資料部份則分成兩個

- 使用者資料: `~/snap/<name>/`
- 系統或 service 資料: `/var/snap/<name>/`

### Snap 的種類

`meta/snap.yaml` 的 `type` 決定 snap 的角色。

| type | 用途 | 例子 |
| - | - | - |
| app | 一般應用程式、CLI 或 service | firefox、lxd |
| base | 提供執行環境（類似小型 rootfs） | core26 |
| snapd | snapd 本身 | snapd |
| gadget | 開發板的 bootloader 與分割區定義 | pc |
| kernel | 核心與 initrd | pc-kernel |

gadget 與 kernel 主要用於 Ubuntu Core，請參考 [Ubuntu Core 入門](../ubuntu-core.md)。

## 常用術語

- App: Snap 提供的程式或 service
- Base: 提供基本執行環境，例如 core26
- Interface: 一類系統資源及其安全政策，例如 home、network
- Plug: 資源需求端
- Slot: 資源提供端
- Connection: plug 與 slot 的實際連接
- Confinement: snap 的隔離等級
- Version: 應用程式自己的版本
- Revision: Snap Store 對每次上傳產生的編號
- Channel: 更新通道，例如 stable、beta、edge

## 啟動 snap 流程

- `snapd`: 主要負責事前的安裝更新
- 使用者執行程式
- `/snap/bin/<app>`
- `snap run`: 讀取 metadata，準備啟動
- `snap-confine`: 建立 sandbox、mount namespace，套用安全政策
- `snap-exec`: 找到實際 command，最後執行應用程式
- 真正的應用程式

## 確保安全性的方法

- SquashFS: 應用程式本體唯讀、不可修改
- Mount namespace: 控制程式看到的檔案系統
- AppArmor: 限制可存取的檔案、socket、D-Bus 等資源
- Seccomp: 限制可使用的 system calls
- Capabilities: 限制 process 的系統特權
- cgroup／udev: 管理 service 及硬體裝置存取
- UNIX permissions: UID、GID、檔案權限仍然有效
- Interfaces: 以 plug／slot 開放必要資源

## Confinement

Confinement 決定 snap 被 sandbox 限制到什麼程度，在 `snapcraft.yaml` 以 `confinement` 設定。

| 模式 | 行為 | 使用時機 |
| - | - | - |
| strict | 完整套用 sandbox，只能透過 interface 存取資源 | 預設，也是上架 stable 的首選 |
| devmode | 不阻擋，只記錄違規 | 開發時找出需要哪些 interface |
| classic | 幾乎沒有 sandbox，可廣泛存取主機 | 編譯器、IDE 等必須存取整個系統的工具 |

- classic snap 安裝時要加 `--classic`
- classic 上架需要經過 Store 審核
- devmode 的 snap 不能發佈到 stable 與 candidate channel

## Interface、Plug 與 Slot

Strict snap 預設無法存取網路、攝影機、家目錄以外的檔案等主機資源。
Interface 是 snap 向系統申請這些資源的機制，可以想成插頭與插座：

- Interface: 一類資源與對應的安全規則，例如 `network`、`camera`、`removable-media`
- Plug: 需求端，snap 宣告「我需要這類資源」，像插頭
- Slot: 提供端，擁有資源的一方宣告「我可以提供這類資源」，像插座
- Connection: 把 plug 接到 slot；沒有 connection，plug 就不會生效

```text
firefox:camera (plug) ───connection───> :camera (slot，由 snapd 提供)
```

### Slot 由誰提供

| 提供者 | 說明 | 例子 |
| - | - | - |
| 系統（snapd） | 主機本身的資源，slot 名稱前沒有 snap 名稱 | `network`、`home`、`camera`、`removable-media` |
| 另一個 snap | 由 snap 之間分享檔案或函式庫 | firefox 透過 `content` interface 使用 `mesa-2404` 的 GPU 驅動 |

### Connection 建立後會發生什麼事

Interface 只是規則範本，connection 建立時 snapd 才會把規則套用到 snap 上：

- AppArmor: 開放對應的檔案、socket、D-Bus
- seccomp: 允許對應的 system calls
- udev／cgroup: 允許存取對應的硬體裝置

中斷 connection 時這些規則會被移除。
所以 permission denied 最常見的原因，是 snap 有宣告 plug，但 connection 沒有建立。

### 自動連接與手動連接

| 方式 | 說明 | 例子 |
| - | - | - |
| 自動連接 | 低風險，安裝時由 snapd 自動連接 | `network`、`home` |
| 手動連接 | 較敏感，需要使用者自行 `snap connect` | `removable-media`、`camera` |
| 審核後自動連接 | 需要 Store 審核，通過後才能自動連接 | `system-files` |

查看與操作 connection 的指令請參考 [Snap 基本操作](usage.md)，在 `snapcraft.yaml` 宣告 plug 的方式請參考 [Snapcraft 打包](snapcraft.md)。

## Channel、Track 與 Risk

Channel 的格式是 `<track>/<risk>/<branch>`，其中只有 risk 是必要的。

- Track: 主要版本線，預設為 `latest`，例如 `2.0`、`3.0`
- Risk: 穩定度，由穩到不穩為 `stable`、`candidate`、`beta`、`edge`
- Branch: 臨時分支，用來測試修正，預設存活 30 天

例如 `latest/stable`、`3.0/beta`。省略 track 時使用 `latest`。
同一個 channel 在每個架構（amd64、arm64 等）各自指向一個 revision。

## 自動更新

snapd 預設每天檢查約 4 次 Store，並在背景更新已安裝的 snap。
Snap 會跟著安裝時指定的 channel 更新，更新失敗時會自動回復到舊 revision。
可以用 `snap refresh --hold` 暫停更新，詳見 [Snap 基本操作](usage.md)。
