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

## 繪製 Topology

F´ 提供 topology visualizer，可以將 `topology.fpp` 中的 component instance 與 port connection 顯示成關係圖。完成 `fprime-util generate` 後，進入 deployment 的 `Top/` 目錄執行：

```bash
cd HiNamespace/FirstDeployment/Top
fprime-util visualize
```

指令會啟動網頁版 visualizer。`topology.fpp` 中每個 `connections` 區塊會顯示為不同的圖，可以透過頁面上的選單切換，查看各 component instance 之間的 port 連線。

## 重要檔案

HelloWorld 專案中較重要的檔案如下：

```text
fprime-tutorial-hello-world/
├── CMakeLists.txt
├── settings.ini
├── requirements.txt
├── lib/fprime/
└── HiNamespace/
    ├── Components/HiComponent/
    │   ├── HiComponent.fpp
    │   ├── HiComponent.hpp
    │   ├── HiComponent.cpp
    │   └── CMakeLists.txt
    └── FirstDeployment/
        ├── Main.cpp
        ├── fprime-gds.yml
        └── Top/
            ├── instances.fpp
            ├── topology.fpp
            ├── FirstDeploymentTopology.cpp
            └── FirstDeploymentPackets.fppi
```

* `CMakeLists.txt`：整個 project 的 CMake 入口，載入 F´ build system 並加入專案中的原始碼目錄。
* `settings.ini`：設定 project 使用的 F´ framework、library 位置與預設 toolchain 等建置選項。
* `requirements.txt`：記錄相容版本的 `fprime-util`、FPP 與 GDS 等 Python 工具。
* `lib/fprime/`：以 Git submodule 引用的 F´ framework 原始碼。
* `HiComponent.fpp`：component 的模型，定義 component 類型、command、event、telemetry 與 port，是產生 C++ 介面的主要來源。
* `HiComponent.hpp`：component 的 C++ class 宣告及內部狀態，例如 `GreetingCount` 使用的計數變數。
* `HiComponent.cpp`：component 的行為實作，包括 `SAY_HI` command handler，以及送出 event 和更新 telemetry 的程式碼。
* `HiComponent/CMakeLists.txt`：將 FPP 模型與 C++ 實作註冊到 F´ build system。
* `FirstDeployment/Main.cpp`：deployment 執行檔的進入點，負責啟動與停止 topology。
* `FirstDeployment/fprime-gds.yml`：GDS 啟動這個 deployment 時使用的設定。
* `Top/instances.fpp`：定義 component instance 的型別、base ID、queue、stack 與 thread priority 等屬性。
* `Top/topology.fpp`：決定 deployment 包含哪些 component instance，以及它們的 port 如何連接。
* `Top/FirstDeploymentTopology.cpp`：放置 topology 初始化時需要的額外 C++ 設定。
* `Top/FirstDeploymentPackets.fppi`：定義要週期性下傳的 telemetry packet 及其 channel。

建置過程會根據 FPP 模型產生檔名帶有 `Ac` 的 C++ 檔案。這些是 build artifact，應修改 `.fpp` 模型後重新產生，不要直接編輯 generated code。

FPP 中的 component、instance、topology，以及 command、event、telemetry、parameter 和 port，可參考獨立的 [FPP 基本介紹](fpp.md)。
