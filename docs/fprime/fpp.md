# FPP 基本介紹

FPP（F Prime Prime）是用來描述 F´ 軟體架構的建模語言。它負責說明「系統有哪些東西，以及它們如何連接」，實際的處理邏輯則主要寫在 C++。

為了方便理解，可以把 FPP 模型大致分成三個層次：

```text
Component 定義
├── Command
├── Event
├── Telemetry
├── Parameter
└── Input / output port
                    │
                    ▼
Instance 定義
建立 component 的具體物件
                    │
                    ▼
Topology 定義
選擇 instance 並連接它們的 port
```

Enum、struct、array 和 port type 等共用定義可以放在 component 外，再由 component 的介面引用。

## Component

Component 將一組相關介面與行為收在一起。例如相機 component 可以提供拍照 command、影像輸出 port、溫度 telemetry，以及錯誤 event。

FPP 負責定義 component 的外部介面，並產生對應的 C++ 介面與資料格式；C++ 的 `.hpp` 與 `.cpp` 則實作收到 command 或 port call 之後要做什麼。依執行方式，component 可分為 passive、queued 和 active。

Component 中常見的介面如下：

| 項目 | 簡單說明 | 常見用途 |
| --- | --- | --- |
| Command | 要求 component 執行一個動作 | 開啟裝置、改變模式、執行計算 |
| Event | 記錄某個時間點發生的事情 | 啟動完成、收到命令、偵測到錯誤 |
| Telemetry | 回報系統目前或一段時間內的數值 | 溫度、電壓、計數器、目前模式 |
| Parameter | 可讀取或修改的設定值 | 取樣頻率、門檻值、校正參數 |
| Port | Component 之間呼叫與傳遞資料的介面 | 傳送感測資料、要求其他 component 工作 |

* **Command**：從外部要求 component 執行動作，可以帶參數。例如要求相機使用指定曝光時間拍照。命令通常由 GDS 或系統內的 sequencer 送出，再由 command dispatcher 分派到目標 component。
* **Event**：表示「某件事情發生了」，通常包含時間、嚴重程度和補充資料。例如 component 啟動完成、命令執行失敗或感測值超出範圍，適合用來閱讀操作紀錄與診斷問題。
* **Telemetry**：Component 回報的狀態或量測值，例如目前溫度、已處理的命令數量或電源狀態。Event 強調發生了一件事，telemetry 則強調一個可以持續更新及觀察的值。
* **Parameter**：Component 運行時使用的設定值，例如溫度警告門檻或控制器增益。它可以有預設值，也能透過命令更新；F´ 的 parameter service 可以集中保存及載入這些設定。
* **Port**：Component 之間的介面。FPP 可以先在 component 外定義可重複使用的 port type，再在 component 內宣告 input port 或 output port。Output port 發出呼叫，input port 接收呼叫，只有型別相容的 port 才能連接。Typed port 會明確定義參數型別；serialized port 則使用通用的位元組資料，適合讓不知道實際型別的 component 轉送資料。實際連線由 topology 決定。

## Instance

Component 只是型別，instance 才是 deployment 中實際使用的物件。例如先定義一種 `Camera` component，再建立 `frontCamera` 和 `rearCamera` 兩個 instance。

每個 instance 有自己的名稱和狀態，也可以設定 base ID。Active component 的 instance 還能設定 priority、queue size 和 stack size 等執行參數。

## Topology

Topology 描述一個 deployment 的完整組成：

* 包含哪些 component instance。
* 哪個 output port 連到哪個 input port。
* Command、event、telemetry、排程和通訊資料如何在系統中流動。

因此 topology 很像系統的接線圖。Component 定義「插座的形狀」，instance 是實際設備，而 topology 決定要使用哪些設備，以及如何把它們接起來。

## 其他常見內容

除了上述核心項目，FPP 也能定義：

* **基本資料型別**：整數、浮點數、布林值和字串。
* **Enum**：一組有名稱的固定選項，例如系統模式。
* **Struct**：將多個欄位組成一筆資料。
* **Array**：固定長度、相同型別的一組資料。
* **Constant**：模型中重複使用的固定值。
* **Module**：將名稱與定義分組，避免不同模組使用相同名稱時發生衝突。
* **Packet set**：指定 telemetry channel 如何組成週期性下傳的 packet。

大型專案也常使用 `.fppi` 檔案保存可重複引用的 FPP 片段，再由 `.fpp` 檔案匯入。

## FPP 與產生的 C++ 程式碼

執行 `fprime-util generate` 和 `fprime-util build` 時，F´ 會檢查 FPP 模型，並產生 component base class、port class、資料型別、topology 連線程式碼及 GDS dictionary 等內容。

這些產生的檔案通常帶有 `Ac`，代表 autocoded。不要直接修改它們；應修改 `.fpp` 後重新建置。開發者撰寫的 `.hpp` 與 `.cpp` 主要負責實作 component 的實際行為。

## HelloWorld 中的對應

在 [HelloWorld](hello-world.md) 範例中：

* `HiComponent.fpp` 定義 `SAY_HI` command、`SayHiEvent` event 和 `GreetingCount` telemetry。
* `Top/instances.fpp` 建立 `hiCmpntInstance`。
* `Top/topology.fpp` 將它加入 `FirstDeployment`，並連接其他 F´ component。

簡單來說，修改 component 能做什麼時看 component FPP；建立具體物件時看 instance；理解整個系統有哪些元件及如何連接時看 topology。
