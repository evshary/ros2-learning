---
title: 上架 Snap Store
description: 註冊 snap 名稱、上傳並發佈到 Snap Store 的各個 channel
keywords:
  - Linux
  - Ubuntu
  - Snap
  - Snap Store
  - Snapcraft
---

本文說明如何把 [Snapcraft 打包](snapcraft.md)產生的 `.snap` 發佈到 [Snap Store](https://snapcraft.io/store)。
Channel 與 revision 的概念請參考 [Snap 概念](concepts.md)。

## 流程總覽

1. 建立 [Ubuntu One](https://login.ubuntu.com/) 帳號並登入
2. 註冊 snap 名稱
3. 上傳 `.snap`，Store 自動檢查並產生 revision
4. 把 revision 發佈到 channel
5. 補上 Store 頁面的 metadata

## 登入

```bash
# 開啟瀏覽器登入 Ubuntu One 帳號
snapcraft login

# 確認目前的帳號
snapcraft whoami
```

還沒有帳號時，先到 [Ubuntu One](https://login.ubuntu.com/) 註冊。

## 註冊名稱

```bash
# 名稱需與 snapcraft.yaml 的 name 一致，先註冊先得
snapcraft register hello-shell

# 查看自己註冊的名稱
snapcraft names
```

名稱已被使用，或與現有品牌太相近時，需要到 [Snapcraft Forum](https://forum.snapcraft.io/) 提出名稱申請。

註冊後的名稱無法自行刪除，會一直登記在你的帳號下。`snapcraft close` 可以下架 snap，但不會釋放名稱，要放棄名稱需向 Store 管理員申請撤銷。已註冊的名稱可在 [snapcraft.io/snaps](https://snapcraft.io/snaps) 查看。測試時請使用不會佔用好名稱的名稱，例如加上自己的前綴。

## 上傳與發佈

### 流程

Store 不負責編譯，上傳的是已經建置好的 `.snap`。預設使用 `snapcraft pack` 在本機建置，也可以用 `snapcraft remote-build` 或網站上的自動建置交給 Launchpad 編譯，再上傳到 Store。

上傳和發佈是兩個獨立的步驟，建議一步步來

1. **上傳**：把 `.snap` 傳到 Store，Store 做自動檢查並產生新的 revision。此時還沒有任何 channel 指向它，使用者不會裝到
2. **確認 revision**：確認 revision 編號與檢查結果
3. **發佈到 edge**：先發佈到最不穩定的 channel，自己安裝測試
4. **逐級提升**：測試沒問題，再把同一個 revision 依序發佈到 beta、candidate、stable

Revision 是 Store 每次上傳時指派的遞增編號，與 `snapcraft.yaml` 的 `version` 無關；channel 指向的是 revision，所以 `snapcraft release` 要使用 revision 編號。

### 第一次上傳

```bash
# 1. 上傳，不指定 channel，只產生 revision
snapcraft upload hello-shell_0.1_amd64.snap

# 2. 查看 revision 編號，以及各 channel 目前指向哪個 revision
snapcraft revisions hello-shell
snapcraft status hello-shell

# 3. 發佈到 edge，這裡的 1 是上一步看到的 revision 編號
snapcraft release hello-shell 1 edge

# 4. 從 edge 安裝，自己測試
sudo snap install hello-shell --edge

# 5. 沒問題後，逐級提升同一個 revision
snapcraft release hello-shell 1 beta
snapcraft release hello-shell 1 candidate
snapcraft release hello-shell 1 stable
```

發佈後也可以到 [snapcraft.io/snaps](https://snapcraft.io/snaps) 開啟自己的 snap，確認 Store 頁面與各 channel 的版本。

### 更新版本

已上架的 snap 要發行新版本，流程與第一次上傳相同，只是多了修改版本與重新打包，名稱不需要再註冊。

```bash
# 1. 修改 snap/snapcraft.yaml 的 version，例如 '0.1' 改成 '0.2'

# 2. 清除舊的建置狀態並重新打包，產生 hello-shell_0.2_amd64.snap
snapcraft clean
snapcraft pack

# 3. 上傳，Store 會給它新的 revision，例如 2
snapcraft upload hello-shell_0.2_amd64.snap

# 4. 查看新的 revision 編號
snapcraft revisions hello-shell

# 5. 先發佈到 edge 測試，再逐級提升
snapcraft release hello-shell 2 edge
snapcraft release hello-shell 2 beta
snapcraft release hello-shell 2 candidate
snapcraft release hello-shell 2 stable
```

- 每次上傳都會得到新的 revision 編號，由 Store 決定，與 `version` 無關
- 修改 `version` 不是 Store 的強制要求，但不改的話，使用者與你自己都分不出兩個 revision 的差別
- 新 revision 發佈到某個 channel 之前，該 channel 仍然是舊的 revision，已安裝的使用者不會被更新
- 使用者會依自己追蹤的 channel 自動更新，所以發佈到 `stable` 就等於推送給所有預設使用者
- 更新後發現問題，把舊的 revision 重新發佈到該 channel 就能回復，例如 `snapcraft release hello-shell 1 stable`

### 注意事項

- 每個架構要各自上傳、各自發佈
- Store 會做自動檢查，未通過時 revision 不能發佈，失敗原因會顯示在上傳輸出
- `grade: devel` 或 `confinement: devmode` 的 snap 只能發佈到 beta 與 edge

### 各 channel 的用途

```text
edge ──> beta ──> candidate ──> stable
```

- `edge`：每次 build 的結果，給開發者自己用
- `beta`：功能完成，給願意嘗鮮的測試者
- `candidate`：準備發行，讓測試者驗證
- `stable`：正式版本，預設使用者安裝的 channel

沒有發佈過 revision 的 channel 會沿用比它更穩定的 channel 的 revision（例如 `beta` 空著時，等同 `candidate`）。
要關閉某個 channel，讓它改為沿用更穩定的 channel，可以使用：

```bash
snapcraft close hello-shell beta
```

## 審核與特殊權限

多數 strict snap 通過自動檢查就能發佈，下列情況需要人工審核，需在 [Snapcraft Forum](https://forum.snapcraft.io/c/store-requests/19) 的 store-requests 版面提出申請

- 使用 `classic` confinement
- 要求特定 interface 自動連接，例如 `system-files`、`personal-files`
- 申請自訂 track，例如 `2.0`
- 名稱申請、使用者介面上的認證（verified publisher）

申請時需說明用途與為什麼 strict confinement 無法滿足。

## Store 頁面 metadata

多數資訊來自 `snapcraft.yaml`，上傳後會自動更新。

```yaml
name: hello-shell
# 一行簡介，最多 78 個字元
summary: Hello from a shell script
# 詳細說明，支援部分 Markdown
description: |
  A minimal snap that prints a message.
# 授權，使用 SPDX 識別碼
license: MIT
# 顯示於 Store 頁面的連結
website: https://example.com
contact: https://example.com/issues
source-code: https://github.com/example/hello-shell
# 圖示，PNG 或 SVG，建議 256x256 以上
icon: snap/gui/icon.png
```

截圖、分類與橫幅圖片，需要登入 [snapcraft.io](https://snapcraft.io/snaps) 的 Dashboard 設定。

## 在 CI 上傳

CI 不能開瀏覽器登入，需要先匯出有限權限的憑證。

```bash
# 只允許上傳並發佈到 edge 與 beta，憑證 90 天後過期
snapcraft export-login \
  --snaps=hello-shell \
  --channels=edge,beta \
  --acls=package_upload,package_release \
  --expires="$(date -d '+90 days' --iso-8601=seconds)" \
  credentials.txt
```

把 `credentials.txt` 的內容存成 CI 的 secret，使用時以環境變數提供，之後的 `snapcraft upload` 就不用互動登入。

```bash
export SNAPCRAFT_STORE_CREDENTIALS="$(cat credentials.txt)"
snapcraft upload --release=edge hello-shell_0.1_amd64.snap
```

- 憑證等同帳號權限，不要提交到版本控制
- 憑證過期前重新匯出並更新 secret

## 自動建置

不想自己維護 CI 時，可以到 [snapcraft.io](https://snapcraft.io/snaps) 的 Dashboard 把 GitHub repository 連到 snap，之後每次 push 由 [Launchpad](https://launchpad.net/) 自動建置並發佈到指定的 channel。
