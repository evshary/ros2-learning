# 執行 F´ HelloWorld

HelloWorld 用最小的範例介紹 F´ 的基本開發流程，讓初學者理解 component、FPP、topology 與 deployment 之間的關係，並實際體驗 command 從 GDS 傳入後，component 如何回傳 event 及更新 telemetry。

F´ 官方的 HelloWorld 範例包含一個 `HiComponent`，用來示範 command、event 與 telemetry：

* `SAY_HI` command：由 Ground Data System（GDS）傳送一段文字給 component。
* `SayHiEvent` event：component 收到 command 後，將文字以 event 送回 GDS。
* `GreetingCount` telemetry：記錄 `SAY_HI` command 被執行的次數。

以下步驟使用官方已完成的範例專案，適合快速確認開發環境及 GDS 是否能正常運作。

## 下載範例

範例使用 Git submodule 引用 F´ framework，因此 clone 時需要加上 `--recurse-submodules`：

```bash
git clone --recurse-submodules https://github.com/fprime-community/fprime-tutorial-hello-world.git
cd fprime-tutorial-hello-world
```

若 clone 時沒有下載 submodule，可以另外執行：

```bash
git submodule update --init --recursive
```

## 準備環境

建立 Python virtual environment，並安裝範例指定的 F´ 工具：

```bash
python3 -m venv fprime-venv
source fprime-venv/bin/activate
pip install -r requirements.txt
```

每次開啟新的 terminal，都需要回到專案目錄並重新執行：

```bash
source fprime-venv/bin/activate
```

## 編譯 HelloWorld

在專案根目錄產生 CMake build cache，接著編譯 component 與 deployment：

```bash
fprime-util generate
fprime-util build
```

`generate` 完成後會建立 `build-fprime-automatic-native/`，`build` 則會將執行檔與 GDS dictionary 等成品放入 `build-artifacts/`。

## 啟動 HelloWorld

進入 `FirstDeployment` 並啟動 GDS：

```bash
cd HiNamespace/FirstDeployment
fprime-gds
```

`fprime-gds` 會自動尋找 dictionary、啟動 `FirstDeployment` 執行檔並開啟瀏覽器。若瀏覽器沒有自動開啟，可前往 http://127.0.0.1:5000/。

畫面右上角顯示綠色連線圖示後，可以測試 HelloWorld：

1. 開啟 **Commanding** 頁面。
2. 選擇 `HiNamespace.hiCmpntInstance.SAY_HI`。
3. 輸入一段 greeting 並送出 command。
4. 在 **Events** 頁面確認 `SayHiEvent` 及剛才輸入的文字。
5. 在 **Channels** 頁面確認 `GreetingCount` 已增加。

完成後，在執行 GDS 的 terminal 按下 `Ctrl+C` 即可停止 GDS 與 deployment。

完整的建立過程可參考 [F´ Hello World Tutorial](https://fprime.jpl.nasa.gov/latest/tutorials-hello-world/docs/hello-world/)，範例原始碼位於 [fprime-tutorial-hello-world](https://github.com/fprime-community/fprime-tutorial-hello-world)。
