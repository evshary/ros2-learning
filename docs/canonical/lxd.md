---
title: LXD 入門
description: 使用 LXD 建立容器與虛擬機器，搭配 Workshop 建立可重現的開發環境，並與 Multipass、Docker、VM 比較
keywords:
  - LXD
  - Workshop
  - Multipass
  - Docker
  - Ubuntu
---

LXD 是 Canonical 維護的系統容器與虛擬機器管理工具。它可以在同一個介面下建立 **系統容器**（共用主機 kernel、行為像一台完整 Linux 機器）與 **虛擬機器**（擁有自己的 kernel），並提供映像檔、快照、網路、儲存與資源限制等管理功能。

Workshop 是建立在 LXD 之上的開發環境工具：把專案需要的 Ubuntu 版本、SDK 與指令寫成定義檔，就能用一個指令重建相同的環境。

## 安裝與初始化

```bash
# Workshop 需要 LXD 6.8 以上，所以指定 6/stable channel
sudo snap install --channel=6/stable lxd

# 以預設值建立 storage pool 與 lxdbr0 網路橋接
lxd init --minimal
```

使用 `lxc` 指令前，目前的使用者需要在 `lxd` 群組內。加入群組後須重新登入才會生效。

```bash
sudo usermod -aG lxd "$USER"
```

## 容器基本操作

啟動一個 Ubuntu 24.04 容器，並查看狀態：

```bash
lxc launch ubuntu:24.04 c1
lxc list
```

```text
+------+---------+-----------------------+------+-----------+-----------+
| NAME |  STATE  |         IPV4          | IPV6 |   TYPE    | SNAPSHOTS |
+------+---------+-----------------------+------+-----------+-----------+
| c1   | RUNNING | 10.227.100.154 (eth0) | ...  | CONTAINER | 0         |
+------+---------+-----------------------+------+-----------+-----------+
```

剛啟動時尚未取得 IP，數秒後再執行 `lxc list` 即可看到。

在容器內執行指令或開啟 shell：

```bash
lxc exec c1 -- cat /etc/os-release
lxc exec c1 -- bash
```

與主機之間複製檔案：

```bash
lxc file push ./local.txt c1/tmp/local.txt
lxc file pull c1/tmp/local.txt ./local-copy.txt
```

限制資源（可在容器執行中直接套用）：

```bash
lxc config set c1 limits.cpu=1 limits.memory=512MiB
```

建立快照與還原：

```bash
lxc snapshot c1 before-change
lxc restore c1 before-change
```

停止與刪除：

```bash
lxc stop c1
lxc delete c1
```

## 建立虛擬機器

加上 `--vm` 就會建立擁有獨立 kernel 的虛擬機器，其餘指令（`list`、`exec`、`snapshot`、`delete`）用法相同：

```bash
lxc launch ubuntu:24.04 v1 --vm
```

虛擬機器需要 KVM 支援，開機也比容器慢；IP 與 agent 就緒需要稍等一段時間。

## 使用 Workshop 建立開發環境

Workshop 把 LXD 容器包裝成「跟著專案目錄走」的開發環境：專案目錄會掛載到 workshop 內的 `/project`，環境內容則由專案裡的 `.workshop/<名稱>.yaml` 描述。

### 安裝

```bash
sudo snap install --classic workshop
```

### 定義與啟動

在專案目錄（建議是 git repository）建立定義：

```bash
mkdir my-project
cd my-project
git init
workshop init dev --base ubuntu@24.04
```

`workshop init` 會產生 `.workshop/dev.yaml`：

```yaml
# workshop 名稱，之後的指令用它指定目標
name: dev
# 容器使用的 Ubuntu 版本
base: ubuntu@24.04
```

啟動並確認狀態：

```bash
workshop launch
workshop list
```

```text
WORKSHOP  STATUS  NOTES
dev       Ready   -
```

`workshop launch` 完成後會在專案目錄留下 `.workshop.lock`，它用來追蹤這個專案的 workshop，請保留在原處，不要加入版本控制。

### 在 workshop 內執行指令

```bash
# 執行單一指令
workshop exec dev -- lsb_release -d

# 開啟互動式 shell，工作目錄為 /project
workshop shell
```

專案目錄與 `/project` 是同一份檔案：在主機新增的檔案，workshop 內立即看得到，反之亦然。

### 定義常用動作

重複執行的指令可寫成 `actions`，用 `workshop run` 呼叫：

```yaml
name: dev
base: ubuntu@24.04
# 具名動作；"$@" 會把 workshop run 後面的參數帶進腳本
# 動作在每次 run 時才被解析，修改後不需要 refresh
actions:
  hello: echo "hello $@"
```

```bash
workshop actions
workshop run dev -- hello ros
```

```text
hello ros
```

### 加入 SDK

SDK 是定義檔中可重複使用的環境元件（例如語言執行環境或服務），發布在 SDK Store。SDK 依附於 Workshop：`sdk` 指令隨 `workshop` snap 一起安裝，只能搜尋與查看（`find`、`info`、`list`）；SDK 不會裝在主機上，而是在 `workshop launch` 或 `workshop refresh` 時下載並安裝到該 workshop 內。

先搜尋，再用 `--sdks` 指定名稱與 channel：

```bash
sdk find ollama
workshop init dev --sdks ollama/cpu/stable --base ubuntu@24.04
```

修改定義（換 base、增減 SDK、切換 channel）之後，用 `refresh` 套用：

```bash
workshop refresh
```

### 停止、啟動與移除

```bash
# 停止，不刪除；之後 start 不會從頭建置
workshop stop
workshop start

# 刪除 workshop，但保留 .workshop/dev.yaml，可再用 launch 重建
workshop remove
```

要查看 workshop 做過哪些操作，可用 `workshop changes` 與 `workshop tasks`。更多內容請參考官方的 [Workshop 文件](https://ubuntu.com/workshop/docs/)。

## 與 Multipass、Docker、VM 的比較

| 項目 | LXD（容器） | LXD（`--vm`）／一般 VM | Multipass | Docker |
| --- | --- | --- | --- | --- |
| 隔離層級 | 共用主機 kernel，以 namespace 與 cgroup 隔離 | 獨立 kernel，硬體虛擬化 | 獨立 VM | 共用主機 kernel，以 namespace 與 cgroup 隔離 |
| 設計目標 | 像一台完整 Linux 的系統容器 | 完整機器，可跑不同 kernel 或作業系統 | 快速建立 Ubuntu VM | 打包並執行單一應用程式 |
| 內部 init 與服務 | 有完整 init（systemd），可跑多個服務 | 有 | 有 | 通常一個容器一個主程序 |
| 啟動速度與資源 | 秒級，額外負擔小 | 較慢，需預留記憶體 | 較慢，需預留記憶體 | 秒級，額外負擔小 |
| 映像來源 | Ubuntu 等發行版映像 | 發行版映像或自備磁碟 | Ubuntu 映像 | OCI 映像（Docker Hub 等） |
| 管理功能 | 快照、網路、儲存、資源限制、叢集 | 同左（LXD 管理時） | 簡單的 launch、shell、mount | build、compose、registry |
| 適合情境 | 隔離的開發／測試環境、模擬多台機器 | 需要自己的 kernel、模組或其他 OS | 想快速得到一台乾淨的 Ubuntu | 部署與發布應用程式 |

選擇時可以這樣想：

- 要 **發布應用程式**：用 Docker，映像就是交付物。
- 要 **一台長期使用、像真機一樣的 Linux 環境**：用 LXD 容器。
- 要 **測試 kernel、驅動、systemd 開機流程或非 Linux 系統**：用 VM（LXD `--vm` 或一般 VM 軟體）。
- 要 **在 macOS 或 Windows 上快速取得 Ubuntu**：用 Multipass。
- 要 **跟著專案、可重現、可分享的開發環境**：用 Workshop（背後仍是 LXD 容器）。

Multipass 與 LXD 都是 Canonical 的工具，且都能建立 VM；差別在於 Multipass 把介面縮到最簡單（單機、Ubuntu 為主），LXD 則提供更完整的容器、VM、網路與儲存管理。
