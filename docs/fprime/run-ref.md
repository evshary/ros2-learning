# 執行 F´ Ref Deployment

`TestDeploymentsProject/Ref` 是 F´ 原始碼中的參考 deployment，可在 Linux 或 macOS 上執行。它包含完整的 component、topology、Command and Data Handling，以及與 Ground Data System（GDS）通訊所需的功能，適合用來確認開發環境是否正常。

以下步驟以 F´ 原始碼的開發版本為例。

## 準備環境

先下載 F´，並建立 Python virtual environment：

```bash
git clone https://github.com/nasa/fprime.git
cd fprime
python3 -m venv fprime-venv
source fprime-venv/bin/activate
pip install -r requirements.txt
```

每次開啟新的 terminal，都需要回到 F´ 根目錄並重新啟用 virtual environment：

```bash
source fprime-venv/bin/activate
```

## 編譯 Ref

進入 Ref deployment，產生建置環境並編譯：

```bash
cd TestDeploymentsProject/Ref
fprime-util generate
fprime-util build
```

`fprime-util generate` 通常只需在第一次建置，或刪除 build artifacts 後重新執行。修改 component 或 topology 後，一般只需再次執行 `fprime-util build`。

F´ 底層使用 CMake 作為建置系統。`fprime-util generate` 是對 CMake configure 階段的包裝，會讀取 `CMakeLists.txt`、FPP 模型與 toolchain 設定，在 `build-artifacts/` 中產生 CMake cache 和建置檔案，但不會真正編譯程式。接著執行 `fprime-util build`，才會透過 CMake 呼叫實際的編譯工具，完成程式碼產生、編譯與連結。

## 清除建置結果

若要清除 project 的 CMake cache、編譯結果及其他建置產物，可以執行：

```bash
fprime-util purge
```

這個指令不會刪除 component、FPP 模型或 C++ 原始碼。清除後需要重新產生建置環境並編譯：

```bash
fprime-util generate
fprime-util build
```

## 啟動 GDS 與 Ref

在 Ref 目錄中執行：

```bash
fprime-gds
```

GDS 會自動找到 Ref 的 dictionary、啟動 Ref 執行檔，並開啟瀏覽器。若瀏覽器沒有自動開啟，可前往 http://127.0.0.1:5000/。

畫面右上角顯示綠色連線圖示後，可以在下列頁面觀察或操作 Ref：

* **Commanding**：傳送 command，例如 command dispatcher 的 `CMD_NO_OP`。
* **Channels**：查看 telemetry channel。
* **Events**：查看 Ref 產生的 event。
* **Logs**：檢查 GDS 與 Ref 的執行紀錄。

按下 `Ctrl+C` 即可停止 GDS 與 Ref。

更多資訊可參考 [F´ Ref 官方文件](https://fprime.jpl.nasa.gov/latest/Ref/)與 [GDS 使用說明](https://fprime.jpl.nasa.gov/latest/docs/user-manual/overview/gds-introduction/)。
