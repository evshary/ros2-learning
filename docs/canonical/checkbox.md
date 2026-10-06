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
以下以 Ubuntu 24.04 為例：

```bash
sudo snap install checkbox24
sudo snap install checkbox --channel=24.04/stable --classic
```

Ubuntu 22.04 可改用 `checkbox22` runtime 及 `22.04/stable` frontend channel。
其他版本請先查看 [Checkbox Snap 版本說明](https://documentation.ubuntu.com/checkbox/stable/reference/snaps/)及 `snap info checkbox`。

## 基本的執行操作

### 跑範例 Tutorial Test Plan

啟動 Checkbox：

```bash
checkbox.checkbox-cli
```

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
