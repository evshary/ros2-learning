---
title: Snap 基本操作
description: 介紹 snap 的安裝、更新、權限連接、資料路徑與疑難排解
keywords:
  - Linux
  - Ubuntu
  - Snap
  - snap CLI
---

本文整理日常使用 snap 的指令。
名詞與機制請先參考 [Snap 概念](concepts.md)。

## 搜尋與安裝

```bash
# 在 Snap Store 搜尋
snap find hello-world

# 顯示 snap 資訊，包含各 channel 目前的 revision
snap info hello-world

# 安裝，預設使用 latest/stable
sudo snap install hello-world

# 指定 channel
sudo snap install hello-world --channel=latest/edge

# classic snap 需要明確同意
sudo snap install code --classic

# 安裝本機 .snap，沒有簽章所以要加 --dangerous
sudo snap install ./hello_1.0_amd64.snap --dangerous
```

## 查看狀態

```bash
# 已安裝的 snap，包含 version、revision、channel
snap list

# 指定 snap 的詳細資訊
snap info hello-world

# 查看有哪些 app 與 service，以及是否啟動
snap services

# 查看最近的 snap 變更（安裝、更新、連接等）
snap changes
```

## 更新、回復與移除

```bash
# 更新全部或單一 snap
sudo snap refresh
sudo snap refresh hello-world

# 切換 channel
sudo snap refresh hello-world --channel=latest/beta

# 回到前一個 revision
sudo snap revert hello-world

# 查看下次自動更新的時間
snap refresh --time

# 暫停自動更新（可指定時間，例如 72h）
sudo snap refresh --hold=72h hello-world

# 移除，預設會建立資料 snapshot；--purge 則不保留
sudo snap remove hello-world
sudo snap remove --purge hello-world
```

## 管理權限

Strict snap 需要透過 interface 的 connection 才能存取資源，原理請參考 [Snap 概念](concepts.md)中的 Interface 章節。

```bash
# 查看指定 snap 的 plug、slot 與連接狀態
snap connections hello-world

# 查看某個 interface 有哪些 plug 與 slot
snap interface removable-media

# 手動連接；格式為 <snap>:<plug> <提供者>:<slot>，提供者省略時為系統
sudo snap connect hello-world:removable-media

# 中斷連接
sudo snap disconnect hello-world:removable-media
```

連接狀態中的 `-` 表示沒有連接，這是 permission denied 最常見的原因。

## 管理 Service 與設定

```bash
# 啟動、停止、重啟 service，--enable 同時設定開機自動啟動
sudo snap start --enable <name>.<service>
sudo snap stop <name>.<service>
sudo snap restart <name>

# 查看 service 的 log
sudo snap logs <name>
sudo snap logs -f <name>.<service>

# 讀寫 snap 的設定（由 snap 作者自行定義 key）
sudo snap set <name> <key>=<value>
sudo snap get <name>
```

## 資料路徑與環境變數

Snap 在執行時會設定這些環境變數，路徑中的 `<revision>` 會隨更新改變。

| 變數 | 路徑 | 特性 |
| - | - | - |
| `$SNAP` | `/snap/<name>/<revision>` | 唯讀，snap 本體 |
| `$SNAP_DATA` | `/var/snap/<name>/<revision>` | 可寫，每個 revision 各一份，更新時會複製 |
| `$SNAP_COMMON` | `/var/snap/<name>/common` | 可寫，所有 revision 共用 |
| `$SNAP_USER_DATA` | `~/snap/<name>/<revision>` | 可寫，使用者資料，每個 revision 各一份 |
| `$SNAP_USER_COMMON` | `~/snap/<name>/common` | 可寫，使用者資料，所有 revision 共用 |

- 需要跨更新保留的資料放在 `common`
- 需要與 revision 綁定、可隨 `snap revert` 回復的資料放在 `$SNAP_DATA`

## 疑難排解

### Permission denied

1. 用 `snap connections <name>` 確認需要的 interface 是否已連接
2. 查看 AppArmor 的拒絕紀錄，找出被擋下的路徑與 interface

    ```bash
    sudo journalctl -k | grep DENIED
    ```

3. 用 `sudo snap connect` 補上 connection

### 進入 snap 的環境除錯

```bash
# 在 snap 的 sandbox 內開 shell
snap run --shell <name>.<app>

# 在 sandbox 內查看環境變數
env | grep SNAP
```

### Service 無法啟動

```bash
# 查看 log 與狀態
sudo snap logs -n=50 <name>
snap services <name>

# 查看 systemd unit，名稱為 snap.<name>.<service>.service
systemctl status snap.<name>.<service>.service
```

### 更新或安裝卡住

```bash
# 查看變更清單與某次變更的詳細步驟
snap changes
snap change <ID>

# 中止卡住的變更
sudo snap abort <ID>
```
