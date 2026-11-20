---
title: Checkbox 入門
description: 安裝 Checkbox、執行第一個測試計畫並查看測試報告
keywords:
  - Checkbox
  - Ubuntu
  - 測試
---

Checkbox 是 Ubuntu 的系統與硬體測試工具。
它會依照測試計畫（test plan）執行一系列測試工作（job）：有些工作會自動完成，有些則需要使用者檢查並回報結果。
測試計畫可能包含失敗、略過或需要手動確認的工作，結果不一定代表整台機器有問題。

## Provider、Job、Test Plan 與 Launcher

| 名稱 | 用途 | 本例中的對應內容 |
| --- | --- | --- |
| Provider（提供者） | 收納 Checkbox 可載入的定義，例如 jobs、test plans、資源及輔助程式。 | `2026.example.org:graphics` provider 及 `units/graphics.pxu` |
| Job（測試工作） | 一項最小的測試步驟，描述要執行的指令、前置條件及如何判定結果。 | `glxgears_acceleration` job 執行 OpenGL 繪圖並請使用者確認結果。 |
| Test plan（測試計畫） | 將相關 jobs 組成一個可選取、可執行的測試流程，也可指定順序及相依關係。 | 互動介面中的 `Checkbox Base Tutorial Test Plan` 是一個 test plan。 |
| Launcher（啟動設定） | 以 INI 設定一次測試執行方式，例如選用哪個 test plan、篩選哪些 jobs、採用哪種介面及輸出報告方式；它不定義測試本身。 | 可用 launcher 預先設定某個 plan 與報告格式。 |

簡單來說，provider 收納 jobs 和 test plans。
Test plan 組織要執行的 jobs。
Launcher 則設定 Checkbox 如何執行這個 plan。

## 安裝

Checkbox 分成執行環境（runtime）和操作介面（frontend）。
Runtime 提供測試內容，frontend 提供操作介面；請選擇與 Ubuntu 版本相符的 runtime 和 frontend 通道（channel）。
以下以 Ubuntu 26.04 為例：

```bash
sudo snap install checkbox26
sudo snap install checkbox --channel=26.04/stable --classic
```

Ubuntu 24.04 可改用 `checkbox24` runtime 及 `24.04/stable` frontend channel。
其他版本請先查看 [Checkbox Snap 版本說明](https://documentation.ubuntu.com/checkbox/stable/reference/snaps/)及 `snap info checkbox`。

## 基本的執行操作

### 跑範例 Tutorial Test Plan

Checkbox 分成 agent 與 controller 兩個角色：
agent 是以 root 在背景執行的服務，安裝在受測機器上，負責實際執行測試；controller 是你操作的互動介面，連到 agent 後選擇並執行測試計畫。
兩者可以在同一台機器（本機測試），也可以在不同機器（跨機器測試）。

先在受測機器上確認 agent 已啟動。
Frontend snap 安裝後會附帶 `checkbox.agent` 服務：

```bash
snap services checkbox
```

如果 `checkbox.agent` 不是 `active`，請啟動並設為開機自動啟動：

```bash
sudo snap start --enable checkbox
```

接著用 controller 連到 agent：

```bash
# 本機測試：連到本機 agent，預設使用 18871 port
checkbox.checkbox-cli control localhost

# 跨機器測試：在操作用的電腦上連到受測機器的 agent
checkbox.checkbox-cli control <受測機器的 IP 或主機名稱>
```

跨機器測試時需注意：

* 兩台機器的 Checkbox 版本需相容，否則 controller 會回報 API 版本不符。
* Agent 預設監聽 `18871` port，防火牆需允許 controller 連入；可用 `--port` 指定其他 port。
* Agent 沒有身分驗證，請只在可信任的網路使用。
  若需要較安全的連線，可先建立 SSH 通道，再用 `control localhost` 連線：

```bash
# 將本機 18871 轉送到受測機器的 agent
ssh -N -L 18871:localhost:18871 <使用者>@<受測機器>
```

直接執行 `checkbox.checkbox-cli`（不加 `control`）也能在本機跑測試，但這種方式由該程式自己取得 sudo 權限。
若系統的 sudo 使用互動式驗證（例如 authd 網域帳號），Checkbox 會一直要求輸入 sudo 密碼而無法繼續；此時請改用 `control`。

在互動介面中：

1. 按 `f` 開啟篩選，輸入 `Tutorial`。
2. 選取 `Checkbox Base Tutorial Test Plan`，按 `Space`，再按 `Enter`。
3. 檢視要執行的測試工作；可按 `Space` 調整選取，然後按 `t` 開始。
4. 遇到手動測試工作時，閱讀步驟並依實際狀況回報結果。
5. 所有工作結束後，在 `Select jobs to re-run` 畫面按 `f`（Finish）結束測試階段（session）。

這個教學計畫包含刻意失敗或異常終止（crash）的範例，也會詢問人工測試結果；這些項目是用來示範 Checkbox 的操作流程。

### 查看報告

透過互動介面完成 session 後，終端機會顯示文字摘要及提交檔案（submission files）的完整路徑。
在出現這些路徑前，報告尚未完成匯出，請稍候。
檔案預設儲存在 `~/.local/share/checkbox-ng/`，包含 HTML、JSON、JUnit XML 及 `.tar.xz` 封存檔。
HTML 適合閱讀，JUnit XML 可供 CI 工具使用；除非你要提交結果，教學測試不需要上傳報告。

### 查看有哪些測試

Checkbox 的文字介面（TUI）載入測試計畫可能較慢。若只想瀏覽可用的測試計畫，可在終端機執行：

```bash
checkbox.checkbox-cli list "test plan" | less
```

## 建立自己的測試

以下示範建立 Checkbox provider，並加入 `glxgears` 圖形測試。
它會啟動 OpenGL 動畫，讓測試者確認繪圖器（renderer）使用 GPU 且畫面能正常繪製。
這是需要人工確認的基本功能測試（smoke test），不是效能基準測試。

### 準備開發環境

依照官方 [Writing test jobs](https://documentation.ubuntu.com/checkbox/stable/tutorial/writing-tests/test-case/) 教學準備 Checkbox 原始碼開發環境。
接著安裝測試所需的工具並建立 provider：

```bash
sudo apt install mesa-utils
checkbox-cli startprovider 2026.example.org:graphics
```

`startprovider` 會建立 provider 的目錄架構。
進入產生的 provider 根目錄；`manage.py` 位於此目錄，job 定義則放在 `units/`。

### 定義 job

在 `units/graphics.pxu` 加入以下測試工作定義：

```ini
id: glxgears_acceleration
plugin: user-interact-verify
category_id: com.canonical.plainbox::graphics
imports: from com.canonical.certification import executable
requires: executable.name == 'glxgears'
_summary: Verify an OpenGL rendering workload
_purpose:
 Check that OpenGL renders using the system graphics hardware.
_steps:
 1. Check the OpenGL renderer shown by glxinfo.
 2. Confirm it identifies the system GPU, not a software renderer such as llvmpipe.
 3. Watch the gears render, then close the window with Escape.
_verification:
 1. Did glxinfo identify the expected GPU renderer?
 2. Did the gears appear and animate without rendering errors?
command: glxinfo -B && glxgears
```

各欄位的用途如下：

* `id`：此 job 在 provider 內的唯一識別名稱。
* `plugin`：指定 job 類型。
  `user-interact-verify` 會執行指令，再讓使用者檢查結果並回報通過或失敗。
* `category_id`：指定分類，讓 Checkbox 將此工作放在圖形測試類別中顯示。
* `imports`：匯入 Checkbox 內建 `com.canonical.certification` provider 的 `executable` 資源；沒有這行時，Checkbox 會在目前 provider 中尋找該資源。
* `requires`：設定 job 的執行前提條件。
  只有 `executable` 資源確認系統有 `glxgears` 執行檔時，job 才會執行。
* `_summary`：在 Checkbox 介面和結果摘要中顯示的簡短名稱。
* `_purpose`：說明這項測試的目的。
* `_steps`：提供測試者操作步驟。
* `_verification`：測試完成後提供判定結果的問題。
* `command`：實際執行的 shell 指令。
  `&&` 表示只有 `glxinfo -B` 成功後才會啟動 `glxgears`。
  動畫視窗需由測試者關閉。

測試者應檢查 `glxinfo` 顯示的 renderer 是否為預期的 GPU，而不是 `llvmpipe` 等軟體 renderer。
不要設定固定 FPS 門檻，因為不同系統的效能差異很大。

### 驗證並執行

在 Checkbox 開發環境中，從 provider 根目錄執行以下指令：

```bash
# 驗證 provider 定義與相依資源
python3 manage.py validate
# 將 provider 註冊到目前的 Checkbox 開發環境
python3 manage.py develop
checkbox-cli run 2026.example.org::glxgears_acceleration
```

執行時，確認 `glxinfo` 顯示預期的硬體 renderer，而不是 `llvmpipe` 等軟體 renderer。
齒輪視窗會持續執行；按 Esc 關閉後，再依 Checkbox 提示確認測試結果。

值得注意的是我們只要跑一次 `python3 manage.py develop` 就好，下次跑 Python 環境 activate `. checkbox_venv/bin/activate` 的時候就會自動載入這個 test job。

### 加入 Graphics Test Plan

我們也可以新定義 task plan 來放這個 task job。
為了讓兩者分開管理，請在同一個 provider 的 `units/test-plans.pxu` 建立以下 test plan：

```ini
unit: test plan
id: graphics-acceleration
_name: Graphics Acceleration Test Plan
include:
  glxgears_acceleration
```

各欄位的用途如下：

* `unit`：宣告這筆定義是 test plan，而不是 job。
* `id`：test plan 的唯一識別名稱；執行時會和 provider namespace 組成完整名稱。
* `_name`：顯示在 Checkbox 互動選單中的計畫名稱。
* `include`：列出此計畫要執行的 job；此處引用同一 provider 中的 `glxgears_acceleration`。

若已對這個 provider 執行過 `python3 manage.py develop`，新增 test plan 後不必重跑；在 provider 根目錄驗證定義並預覽計畫內容：

```bash
python3 manage.py validate
checkbox-cli list "test plan" | grep graphics-acceleration
checkbox-cli list-bootstrapped 2026.example.org::graphics-acceleration
```

`list-bootstrapped` 會列出計畫實際展開後的工作。確認清單包含 `glxgears_acceleration` 後，可直接執行計畫：

```bash
checkbox-cli run 2026.example.org::graphics-acceleration
```

若要透過互動選單選取此計畫，請在同一個 Checkbox source development environment 執行 `checkbox-cli`，再從 test plan 清單選取 `Graphics Acceleration Test Plan`。

### 建立 Launcher

Test plan 決定「測什麼」，launcher 決定「怎麼跑」：選哪個 test plan、要不要略過互動選單、結果輸出到哪裡。
Launcher 是 INI 格式的檔案，不屬於 provider。
在任意位置建立 `graphics.conf`：

```ini
[launcher]
launcher_version = 1
# 此 launcher 的識別名稱，用來區分測試 session
app_id = org.example:graphics

[test plan]
# 指定要執行的 test plan
unit = 2026.example.org::graphics-acceleration
# 不顯示 test plan 選單，直接使用上面指定的 plan
forced = yes

[test selection]
# 不顯示 job 選單，直接執行 plan 內所有 job
forced = yes

[transport:local_file]
# 將結果檔案複製到指定路徑
type = file
path = /tmp/graphics-submission.tar.xz

[exporter:tar]
# 輸出格式：包含所有結果的 .tar.xz 封存檔
unit = com.canonical.plainbox::tar

[report:file]
# 將 tar 匯出結果送到上面的 local_file
transport = local_file
exporter = tar
forced = yes
```

在 Checkbox 開發環境中，用 launcher 執行：

```bash
checkbox-cli launcher graphics.conf
```

執行時不會再出現 test plan 與 job 選單，結束後會把結果存到 `/tmp/graphics-submission.tar.xz`，另外也會產生預設的 HTML、JSON 等報告。
`certify-ubuntu-server` 這類認證指令，也是這種 launcher：固定 test plan 並設定報告輸出方式。

## 執行 Desktop、Server 與 IoT 測試

Canonical 針對 OEM、ODM 提供 Ubuntu 的認證，包含 Desktop、Server、IoT 各個種類。
這些硬體測試是可以直接在自己電腦執行的，最底層也是會呼叫到 Checkbox。
詳細可以參考 [Hardware Certification documentation](https://certification.canonical.com/docs/programmes/)。

### 為什麼要另外安裝認證套件

認證用的 test plan 也包含在 Checkbox snap 裡，`checkbox-cli run` 同樣能執行。
`canonical-certification-client` 與 `canonical-certification-server` 本身是空的元套件，只相依 Checkbox 與對應的 provider。
它們真正多裝的，是測試時會呼叫的外部工具。
這些工具是系統層級的 deb 套件，用 snap 無法完整提供；缺少時，相關的 job 會失敗或被略過。

| | Desktop：`canonical-certification-client` | Server：`canonical-certification-server` |
| --- | --- | --- |
| Checkbox 元件 | `checkbox-provider-certification-client`、`checkbox-provider-tpm2` | `checkbox-provider-certification-server`、`checkbox-provider-gpgpu`、`canonical-certification-submit` |
| 圖形與顯示 | `glmark2`、`mesa-utils`、`wmctrl`、`render-bench`；`qmlscene` 等 QML 套件（圖形啟動器） | 無 |
| 週邊裝置 | `fswebcam`（相機）、`obexftp`（藍牙）、`iw`（無線網路） | `ipmitool`、`freeipmi-tools`、`msr-tools`（伺服器管理與 CPU） |
| 網路 | `ethtool` | `iperf`、`nmap`、`ipcalc` |
| 儲存與加密 | `hdparm`、`clevis`、`clevis-tpm2`、`cryptsetup-bin` | 無 |
| 虛擬化 | 無 | `qemu-system`、`qemu-utils`、`ovmf`、`cpu-checker` |
| 壓力與效能 | `stress` | `stress`、`stress-ng`、`bonnie++`、`numactl` |
| 韌體 | `fwts` | `fwts` |
| 其他 | `imagemagick` | `ssh`、`screen`、`git`、`gcc`、`make`、`cmake` |

Server 套件另外提供 `certify-ubuntu-server`、`test-cpu` 等 launcher 與 `canonical-certification-precheck`；Desktop 則是直接執行 test plan。
Ubuntu Core 不支援 deb，因此 Core 只需安裝 Checkbox 的 strict snap，不需要另外的認證套件。

下面每種安裝後，都附上一個只跑單一 job 的快速試跑指令，不需要執行完整認證測試。

### Server

Server 套件的 `certify-*`、`test-*` 其實是 Checkbox 的 launcher 設定檔，內容是固定的 test plan 與報告設定：

| 指令 | 執行的 test plan |
| --- | --- |
| `certify-ubuntu-server` | `server-full-26.04`（完整認證） |
| `test-cpu` | `cpu-only`（含 `stress-ng` 壓力測試） |
| `test-memory` | `memory-only`（含記憶體壓力測試） |
| `test-usb` | `usb-only`（需要接上 USB 裝置） |

`test-*` 用於認證後針對單一項目重測，不適合當作快速試跑；快速試跑請用下方的單一 job 或資訊收集 plan。

安裝 Server 測試套件 `canonical-certification-server`：

```bash
sudo add-apt-repository ppa:checkbox-dev/stable
sudo add-apt-repository ppa:firmware-testing-team/ppa-fwts-stable
sudo apt install canonical-certification-server
```

快速試跑：

```bash
# 只執行一個 job，收集 /proc/cpuinfo；不會進行完整認證
checkbox-cli run com.canonical.certification::cpuinfo_attachment

# 執行資訊收集 test plan；全自動，不含壓力測試
checkbox-cli run com.canonical.certification::server-info-attachment-automated
```

可以先確認測試環境：

```bash
canonical-certification-precheck
```

開始完整認證測試：

```bash
# 對應最新 Ubuntu 版本的 launcher，執行時間可能長達數小時
certify-ubuntu-server
```

可以在 `~/.local/share/checkbox-ng/submission_<DATECODE>.html` 看到最終 report。

### Desktop

Desktop 沒有 `certify-*` 腳本，而是直接執行對應 Ubuntu 版本的 test plan。
Test plan 已包含在前面安裝的 Checkbox snap 中，指令改用 `checkbox.checkbox-cli` 即可。
如果想用 deb 安裝，並一併裝上上表的工具，則安裝 `canonical-certification-client`，指令為 `checkbox-cli`：

```bash
sudo add-apt-repository ppa:checkbox-dev/stable
sudo apt install canonical-certification-client
```

先列出可用的 test plan：

```bash
# deb 安裝使用 checkbox-cli；snap 安裝使用 checkbox.checkbox-cli
checkbox-cli list "test plan" | grep client-cert-desktop-26-04
```

快速試跑：

```bash
# 只執行一個 job，收集 /proc/cpuinfo；不會進行完整認證
checkbox-cli run com.canonical.certification::cpuinfo_attachment
```

開始完整認證測試：

```bash
# 全部測試，包含需要人工確認的項目與壓力測試
checkbox-cli run com.canonical.certification::client-cert-desktop-26-04

# 也可以只跑不需要人工操作的部分
checkbox-cli run com.canonical.certification::client-cert-desktop-26-04-automated
```

### Core

Ubuntu Core 無法安裝 deb 或 classic 的 frontend，需要安裝 runtime 與 strict 的 frontend：

```bash
# runtime，提供測試內容
sudo snap install checkbox26
# uc26 是對應 Ubuntu Core 26 的 strict frontend channel
# --devmode 解除 snap 的權限限制，否則 Checkbox 無法在 /var/tmp 建立工作階段目錄
sudo snap install checkbox --channel=uc26/stable --devmode
```

快速試跑：

```bash
# 只執行一個 job，收集 /proc/cpuinfo；不會進行完整認證
checkbox.checkbox-cli run com.canonical.certification::cpuinfo_attachment
```

開始完整認證測試：

```bash
# Snap 內建的 Core 認證 launcher（主要供 Canonical 實驗室使用）
checkbox.client-cert-iot-ubuntucore

# 或指定 test plan
checkbox.checkbox-cli run com.canonical.certification::client-cert-iot-ubuntucore-26
```

## Agent 安全注意事項

Checkbox frontend 會啟動 Checkbox agent service。
官方文件提醒，這個 service 可提供未經驗證的 root 層級遠端控制；只應在可信任的測試環境使用，完成後若暫時不需要，可停止並停用：

```bash
sudo snap stop --disable checkbox
```

需要再次使用 Checkbox agent 時，可手動啟動：

```bash
sudo snap start checkbox
```

## 參考資料

* [Checkbox Tutorial](https://documentation.ubuntu.com/checkbox/stable/tutorial/)
* [Installing Checkbox](https://documentation.ubuntu.com/checkbox/stable/tutorial/using-checkbox/installing-checkbox/)
* [Running your first test plan](https://documentation.ubuntu.com/checkbox/stable/tutorial/using-checkbox/running-checkbox/)
* [Review test report](https://documentation.ubuntu.com/checkbox/stable/tutorial/using-checkbox/test-report/)
