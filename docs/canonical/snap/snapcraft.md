---
title: Snapcraft 打包
description: 使用 snapcraft 撰寫 snapcraft.yaml 並建置 snap
keywords:
  - Linux
  - Ubuntu
  - Snap
  - Snapcraft
---

Snapcraft 是製作 snap 的工具，讀取專案中的 `snapcraft.yaml`，依序拉取、建置、安裝各個 part，最後打包成 `.snap`。
概念請先參考 [Snap 概念](concepts.md)，建置完成後可依 [上架 Snap Store](store.md) 發佈。

## 安裝 snapcraft

```bash
sudo snap install snapcraft --classic

# 預設用 LXD 建置，確保環境乾淨
sudo snap install lxd
sudo lxd init --auto

# 把自己加入 lxd 群組，才能不用 sudo 操作 LXD
sudo usermod -aG lxd "$USER"
```

群組變更要登出整個桌面 session 再登入（或重新開機）才會生效，可用 `id` 確認輸出已包含 `lxd`。

## 專案結構

```text
hello/
├── snap/
│   └── snapcraft.yaml
└── hello.sh
```

`hello.sh` 的內容：

```bash
#!/bin/sh
echo "Hello from snap"
```

dump plugin 會保留原檔的權限，所以腳本一定要先設為可執行，否則 `snapcraft pack` 會以 `should be world-readable and executable` 失敗：

```bash
chmod +x hello.sh
```

## 撰寫 snapcraft.yaml

```yaml
# snap 名稱，上架時必須先在 Store 註冊
name: hello-shell
# 執行環境，core26 對應 Ubuntu 26.04
base: core26
# 給人看的名稱，顯示在 Store 與圖形介面，缺少時 snapcraft 會顯示 lint 警告
title: Hello Shell
# 應用程式自己的版本，顯示用，與 revision 無關
version: '0.1'
summary: Hello from a shell script
description: |
  A minimal snap that prints a message.

# 以下也是 metadata，缺少時同樣只會顯示 lint 警告（不影響打包）
# license 使用 SPDX 識別碼
license: MIT
# contact、issues、source-code、website 可填 email 或網址
contact: you@example.com
issues: https://example.com/hello-shell/issues
source-code: https://example.com/hello-shell
website: https://example.com/hello-shell

# strict 是預設的完整 sandbox；開發途中想放寬可改 devmode
confinement: strict
# stable 表示可發佈到 stable channel；測試中用 devel，Store 不允許 devel 進 stable／candidate
grade: stable

apps:
  hello-shell:
    # 執行檔路徑相對於 $SNAP
    command: bin/hello.sh
    # 宣告需要的 interface，network 之類的 plug 會在安裝時自動連接
    plugs:
      - network

parts:
  hello:
    # dump plugin 只把 source 的檔案複製進 snap，不編譯
    plugin: dump
    source: .
    # 只留下需要的檔案，避免把 snap/ 資料夾也打包進去
    organize:
      hello.sh: bin/hello.sh
    stage:
      - bin/hello.sh
```

## 建置

```bash
cd hello

# 在 LXD container 內建置，產生 hello-shell_0.1_amd64.snap
snapcraft pack
```

- 第一次建置會下載 base 並建立 container，需要一些時間
- `snapcraft pack` 與直接輸入 `snapcraft` 相同
- 要建置其他架構可使用 `snapcraft remote-build`，由 Launchpad 代為建置

不想使用 container 時，可以直接在主機建置，但主機的 Ubuntu 版本必須與 base 相同（core26 對應 Ubuntu 26.04），且會直接動到主機環境。

```bash
snapcraft pack --destructive-mode
```

## 本機安裝測試

```bash
# 未上架的 snap 沒有簽章，所以要加 --dangerous
sudo snap install ./hello-shell_0.1_amd64.snap --dangerous

hello-shell

# 查看 snap 內有哪些 connection
snap connections hello-shell
```

測試完成後移除：

```bash
# --purge 連同資料一起刪除，並且不留下移除前的資料快照
sudo snap remove --purge hello-shell

# 確認已移除，應該顯示 error: no matching snaps installed
snap list hello-shell
```

建置產生的檔案不會隨 `snap remove` 消失，需要時另外清理：

```bash
# 刪除 LXD build container 與 part 狀態
# 需在專案資料夾執行，清除的是 lxc --project snapcraft list 看到的對應 container
snapcraft clean

# 刪除打包出來的 snap 檔
rm hello-shell_0.1_amd64.snap
```

## Parts 與 Plugin

一個 snap 由多個 part 組成，每個 part 有自己的 plugin、source 與生命週期。

| 階段 | 動作 | 控制的 key |
| - | - | - |
| pull | 取得 source | `source`、`source-type`、`source-checksum`、`source-branch`、`source-tag`、`source-commit`、`source-depth`、`source-submodules`、`source-subdir` |
| build | 編譯，結果放在 `$CRAFT_PART_INSTALL` | `plugin`、`after`、`build-packages`、`build-snaps`、`build-environment`、`build-attributes`、`organize` |
| stage | 把各 part 的結果合併到 stage 目錄 | `stage`、`stage-packages`、`stage-snaps` |
| prime | 挑出最終進入 snap 的檔案 | `prime`、`permissions` |

- 歸類依據是 [Craft Parts 的 Part properties](https://documentation.ubuntu.com/craft-parts/latest/common/craft-parts/reference/part_properties/) 與 [Snapcraft 的 Parts lifecycle](https://documentation.ubuntu.com/snapcraft/stable/explanation/parts-lifecycle/)

常用 plugin

| plugin | 用途 |
| - | - |
| dump | 直接複製檔案 |
| nil | 不做事，搭配 `override-build` 自訂流程 |
| python | 以 pip 建置 Python 專案 |
| cmake、make、autotools | 一般 C／C++ 專案 |
| rust、go、npm | 對應語言的專案 |

```yaml
parts:
  my-app:
    plugin: make
    source: https://github.com/example/my-app.git
    source-tag: v1.0
    # 建置時需要的 apt 套件
    build-packages:
      - gcc
    # 執行時需要、會被打包進 snap 的 apt 套件
    stage-packages:
      - libssl3t64
```

## 除錯

```bash
# 只執行到某個階段，不繼續往下
snapcraft pull
snapcraft build

# 失敗或完成後，直接進入 container 的 shell
snapcraft pack --shell-after

# 建置失敗時進入 shell
snapcraft pack --debug

# 清除建置結果與 container
snapcraft clean
```

建置完成後，用 `snap run --shell <name>.<app>` 進入 sandbox 檢查，詳見 [Snap 基本操作](usage.md)。

### LXD 無法連網

建置時出現 `A network related operation failed in a context of no network access`，代表 LXD container 連不到外網。
主機同時安裝 Docker 時最常見，因為 Docker 會把 iptables 的 FORWARD 預設政策改成 DROP，擋掉 `lxdbr0` 的轉發流量。

```bash
# 看 FORWARD 的預設政策，DROP 就是上述原因
sudo iptables -S FORWARD | head -3
```

在 Docker 的 `DOCKER-USER` chain 放行 `lxdbr0`，重新開機後規則會消失：

```bash
sudo iptables -I DOCKER-USER -i lxdbr0 -j ACCEPT
sudo iptables -I DOCKER-USER -o lxdbr0 -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
```

設定完成後清除失敗留下的 container 並重新建置：

```bash
snapcraft clean
snapcraft pack
```

## 宣告 plug 與 slot

```yaml
apps:
  my-daemon:
    command: bin/my-daemon
    # service 會以 systemd unit 在背景執行
    daemon: simple
    plugs:
      - network
      - network-bind
      # 需要手動連接的 interface 也要在這裡宣告
      - removable-media

# 以 content interface 與其他 snap 分享檔案時，由提供者宣告 slot
slots:
  shared-data:
    interface: content
    content: shared-data
    read:
      - $SNAP/share
```

- 沒有宣告的 interface，snap 就無法使用
- 找不到該用哪個 interface 時，先以 `confinement: devmode` 安裝，再從 `journalctl -k | grep DENIED` 找出缺少的項目
