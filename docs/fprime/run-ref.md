# 執行 F´ Ref Deployment

`TestDeploymentsProject/Ref` 是 F´ 原始碼中的參考 deployment，可在 Linux 或 macOS 上執行。它包含完整的 component、topology、Command and Data Handling，以及與 Ground Data System（GDS）通訊所需的功能，適合用來確認開發環境是否正常。

以下步驟以 F´ 原始碼的開發版本為例。

## 準備環境

先下載 F´，並建立 Python virtual environment：

```bash
git clone https://github.com/nasa/fprime.git
cd fprime/TestDeploymentsProject
python3 -m venv fprime-venv
source fprime-venv/bin/activate
pip install -r ../requirements.txt
```

每次開啟新的 terminal，都需要回到 `TestDeploymentsProject` 並重新啟用 virtual environment：

```bash
source fprime-venv/bin/activate
```

## 編譯 Ref

在 `TestDeploymentsProject` 目錄產生建置環境並編譯：

```bash
fprime-util generate
fprime-util build
```

`fprime-util generate` 通常只需在第一次建置，或刪除 build cache 後重新執行。修改 component 或 topology 後，一般只需再次執行 `fprime-util build`。

F´ 底層使用 CMake 作為建置系統。`fprime-util generate` 是對 CMake configure 階段的包裝，會讀取 `CMakeLists.txt`、FPP 模型與 toolchain 設定，但不會真正編譯程式。接著執行 `fprime-util build`，才會透過 CMake 呼叫實際的編譯工具，完成程式碼產生、編譯與連結。

完成兩個指令後，`TestDeploymentsProject` 中主要會出現以下資料夾：

```text
TestDeploymentsProject/
├── build-fprime-automatic-native/  # generate 建立的 CMake cache 與建置檔案
└── build-artifacts/                # build 輸出的執行檔、dictionary 等成品
    └── <platform>/
```

`<platform>` 會依使用的作業系統或 toolchain 而不同，例如原生 Linux 建置通常是 `Linux`。FPP 自動產生的 C++ 程式碼與編譯過程中的中間檔案主要位於 build cache；Ref 執行檔與 GDS 使用的 topology dictionary 則可在 `build-artifacts/` 下找到。

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

在 `TestDeploymentsProject` 目錄中執行：

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

## 使用 `--no-app` 分開啟動

預設執行 `fprime-gds` 時，GDS 會自動啟動 Ref。若要自行控制 Ref 的啟動參數、搭配 debugger，或連接另一台裝置上的 deployment，可以加上 `--no-app`，只啟動 GDS：

```bash
cd fprime/TestDeploymentsProject
source fprime-venv/bin/activate
fprime-gds --no-app
```

GDS 啟動後會在預設的 TCP port `50000` 等待 Ref 連線。接著開啟另一個 terminal，啟用相同的 virtual environment，再手動執行 Ref：

```bash
cd fprime/TestDeploymentsProject
source fprime-venv/bin/activate
./build-artifacts/Linux/Ref/bin/Ref -a 127.0.0.1 -p 50000
```

`-a` 指定 GDS 位址，`-p` 指定通訊埠。macOS 或其他 toolchain 的 platform 目錄名稱可能不是 `Linux`，請將路徑中的 `Linux` 換成 `build-artifacts/` 下實際產生的目錄名稱。

停止時，分別在 Ref 與 GDS 所在的 terminal 按下 `Ctrl+C`。

更多資訊可參考 [TestDeploymentsProject/Ref README](https://github.com/nasa/fprime/tree/devel/TestDeploymentsProject/Ref)、[F´ Ref 官方文件](https://fprime.jpl.nasa.gov/latest/Ref/)與 [GDS 使用說明](https://fprime.jpl.nasa.gov/latest/docs/user-manual/overview/gds-introduction/)。
