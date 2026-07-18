---
title: 模糊測試
description: 簡介模糊測試 (fuzz test) 的基本概念
keywords:
  - test
---

fuzz 測試專門用在產生隨機的輸入，來確保程式的結果都會符合我們預期。
fuzz 測試和一般測試的不同在於，我們不會預先知道他的輸入，所以更能夠發現一些預想不到的問題。
人工很難列舉出所有可能的輸入，fuzzing 能幫助我們去產生各式各樣可能的輸入，探索程式的可能行為空間。
很多安全漏洞或是 bug 都是透過 fuzz 測試來找出來的。

## Fuzz 種類

比較常用的有兩大類：libFuzzer 和 AFL

libFuzzer 是 fuzz test 很常被使用的技術，是 LLVM toolchain 所提供的 fuzzing runtime library。
通常用在有原始程式碼的待測目標函式，一起編譯後直接進行測試。
主要特色有：

* In-process execution：同個 process 呼叫不同 function，速度很快，但也容易造成 global state 污染
* Coverage-guided：透過 LLVM instrumentation，知道 input 會經過程式走過的路徑 (coverage)，並且依此來調整 input
* 透過 LLVM 的 sanitizer 來鎖定各種常見的問題，例如 ASan、UBSan 等等

AFL 是另外一種 fuzz test 類型，主要基於外部的 fuzz engine。
AFL 可以運用在沒有程式碼的二進位檔案直接測試，更加有彈性。
主要特色是：

* AFL 如果有程式碼，可以在編譯時自行加入 instrumentation 來偵測 coverage。如果沒有的話，則可以使用 QEMU mode 動態偵測 coverage
* Fork server：透過 fork 快速複製 process，在確保隔離的情況下，提高執行速度
* 外部控制：會有外部的 engine 來控制輸入和執行

不管是哪種基本上都會有如下特色：

* 透過 coverage 來尋找新的可能路徑
* 利用 mutation 來變化 input
* 收集有價值的 corpus 集合，提供未來測試
* 發生問題的時候會嘗試收斂造成問題的最小可能 input

### 如何選擇工具

選擇 fuzzing 工具時，要先確認是否有原始碼、目標是 userspace 程式還是完整 firmware、CPU 架構是否與測試主機相同，以及是否依賴特定周邊設備。

* 如果有 source code，應優先使用 libFuzzer 或 AFL++ 搭配 harness。libFuzzer 適合直接測試單一函式或 library；AFL++ 透過外部 process 控制目標，在執行隔離上較有彈性。
* 如果只有 binary，可以使用 AFL++ QEMU mode。
* 如果要測試 raw image 中的單一 parser 或函式，可以使用 Unicorn 或 Fuzzware。
* 如果需要執行完整 firmware，可以使用 QEMU system mode 或 Renode 模擬 CPU、記憶體與周邊設備。
* 如果目標依賴無法模擬的硬體，才考慮直接在真實硬體上測試。

下表除了整理各類目標的建議工具，也列出 coverage 的來源。
Fuzzer 會利用 coverage 判斷 input 是否讓程式走到新路徑，並決定是否將它保留到 corpus 中。

| 測試目標 | 建議方法 | Coverage 來源 |
| --- | --- | --- |
| Linux ELF，有 source code | Compiler instrumentation | 編譯時插入 coverage 計數邏輯 |
| Linux ELF，只有 binary | AFL++ QEMU mode | QEMU 動態翻譯時收集 basic block 資訊 |
| ARM Embedded Linux ELF | AFL++ QEMU mode | QEMU user mode 執行目標架構程式並收集 coverage |
| Raw image，只測單一 parser 或函式 | Unicorn / Fuzzware | 透過 emulator 的 basic-block hook 記錄執行路徑 |
| Raw image，測試完整 firmware | QEMU system mode / Renode | 由系統 emulator 記錄 CPU 執行過的路徑 |
| 無法模擬的 firmware | 真實硬體 | ETM、SWD 或自訂 instrumentation |

## 可以測試的問題種類

一般來說 fuzz 可以測出如下幾種 bug：

* crash / panic：常見的有 abort、panic、assert 等等
* sanitizer issue：有如下幾種，一般來說 Rust 比較少這類的問題
    * ASan：double free、overflow、use-after-free
    * UBSan：signed overflow、misaligned accesss
    * LSan：memory leak
* Oracle：判斷結果是否正確
    * round-trip：decode 和 encode 行為是否一致
    * differential：不同的實作是否有同樣的結果
* Invariant：確定系統規則要成立，例如 seq number 不能往後倒退、不能有重複 ID 等等
    * 當我們要驗證 protocol 或系統在亂塞入各種 input 是否符合預期時，通常不是把程式內的確認邏輯重新複製一份到 fuzz test 驗證中。而是設定不該被違反的規則，在測試後，依然同樣要遵守。
    * 怎麼找出合理且必須要遵守的系統規則是這個測試的真正關鍵，也是最難的地方

## fuzz 的品質

* 足夠完整的測試
    * 有包含 raw bytes、structured message、protocol state 等等
    * 除了 crash 外也有 sanitizer、Oracle、Invariant 等等
* coverage
    * 一般來說 fuzz 測試都會搭配編譯器來評估每個 input 有走過哪些程式路徑，走得越多就代表測試越完整，我們的信心度也會比較大。
* corpus 品質
    * 好的 fuzz 測試工具會將能夠走新路徑的 input 存起來（稱之為 corpus），並且基於這些 corpus 來進行不同的變化。
    * 如果 corpus 品質高，fuzz tool 可以走遍更多 protocol path
    * 測試失敗的 crash artifact 也應該存起來，轉成 regression test
* 定期運行
    * 整合到 CI 和 OSS-Fuzz

## OSS-Fuzz

Google 有提出 [OSS-Fuzz 平台](https://github.com/google/oss-fuzz)可以幫開源軟體做 fuzz test。
我們只要在該平台設定好怎麼編譯和執行，Google 會自動幫軟體進行 fuzz test，當有發現問題也會回報給我們。
這樣就不用擔心自己沒有足夠的資源可以來進行 fuzz test，畢竟 fuzz test 通常都需要跑上多個小時才能發現問題。

下面會列出大致流成為和，而詳細步驟可以參考[官方文件](https://google.github.io/oss-fuzz/)：

1. 在貢獻之前，要先 sign [CLA (Contributor License Agreements)](https://cla.developers.google.com/about) 才行
2. 接著 clone oss-fuzz，並且在 projects 下加上要放入的 project 資料夾
3. 資料夾裡面會有三個部份，可以參考 [zenoh](https://github.com/google/oss-fuzz/tree/master/projects/zenoh) 的設定：
   * project.yaml：project 的相關 configuration，例如要用什麼 fuzzer、程式語言、有問題要寄信給誰等等，這邊的聯絡信箱必須是跟 Google 綁定的才行
   * Dockerfile：編譯 fuzz 測試的環境，記住 clone project 的時候可以加上 `--depth 1`，減少從 GitHub 下載流量
   * build.sh：如何編譯 fuzz 測試
4. 完成後我們可以在本地端先做測試

    ```bash
    # 製作 docker image
    python3 infra/helper.py build_image your-project
    # 編譯 fuzzers，編譯結果會放在 build 資料夾下
    python3 infra/helper.py build_fuzzers your-project
    # 運行 fuzzer
    python3 infra/helper.py run_fuzzer your-project your-fuzzer-name
    # 如果希望要有 coverage 結果，可以做下面的測試
    python3 infra/helper.py build_fuzzers --sanitizer coverage your-project
    python3 infra/helper.py coverage --no-corpus-download your-project
    ```

5. 都沒問題後就可以發 PR 到 GitHub 上了
6. 被 merge 並等一段時間之後，用自己綁定的 Google 帳號登入 [oss-fuzz](https://oss-fuzz.com/) 就可以看到運行 Fuzz test 的結果了
    * 我們也可以在 `https://introspector.oss-fuzz.com/project-profile?project=<your_project_name>` 上直接看執行結果的 log 和圖表（例如 [zenoh](https://introspector.oss-fuzz.com/project-profile?project=zenoh)），這個不需要權限

## 實作範例

以下使用 [binary-fuzzing-labs](https://github.com/evshary/binary-fuzzing-labs) 中的範例，實際操作 libFuzzer 與 AFL++ source instrumentation。範例以 Ubuntu 為基礎，先安裝所需工具並 clone repository：

```bash
sudo apt update
sudo apt install git clang make afl++ build-essential llvm

git clone https://github.com/evshary/binary-fuzzing-labs.git
cd binary-fuzzing-labs
```

Fuzzer 預設會持續執行，測試過程中可以按 `Ctrl-C` 停止。

### libFuzzer 範例

這個範例將 fuzz target 與 libFuzzer 編譯到同一個 process 中。libFuzzer 會不斷產生 input，並直接呼叫 `LLVMFuzzerTestOneInput` 測試目標函式。

```bash
cd 02-libfuzzer

# make build 實際執行下列編譯指令：
# clang -std=c11 -Wall -Wextra -Wpedantic -O1 -g \
#   -fsanitize=fuzzer,address,undefined fuzz_target.c -o fuzzer
make build

# make fuzz 會建立輸出目錄，再執行：
# ./fuzzer corpus -artifact_prefix=artifacts/
make fuzz
```

<!-- markdownlint-disable MD046 -->
??? example "展開查看 libFuzzer 執行結果"

    ```text
    INFO: Running with entropic power schedule (0xFF, 100).
    INFO: A corpus is not provided, starting from an empty corpus
    #2      INITED cov: 2 ft: 2 corp: 1/1b
    #411    NEW    cov: 5 ft: 5 corp: 2/8b
    #555    NEW    cov: 7 ft: 7 corp: 3/15b
    ...
    ERROR: libFuzzer: deadly signal
    SUMMARY: libFuzzer: deadly signal
    Test unit written to artifacts/crash-<hash>
    ```
<!-- markdownlint-enable MD046 -->

這個範例不需要準備 seed，libFuzzer 會從空 input 開始建立 corpus。執行時可以從輸出觀察幾個重要欄位：

* `cov`：目前觸及的 coverage。
* `corp`：保留在 corpus 中的 input 數量與總大小。
* `exec/s`：每秒執行的測試次數。
* `NEW`：這個 input 找到了新路徑，已被加入 corpus。

當 libFuzzer 找到隱藏的 `LIBFUZZ` 條件後，會觸發 crash，並將輸入存在 `artifacts/crash-*`。可以用下列指令重現問題：

```bash
# make reproduce 會取得第一個 crash artifact，實際執行類似：
# ./fuzzer artifacts/crash-<hash> -runs=1
make reproduce
```

這個 target 同時啟用 AddressSanitizer 與 UndefinedBehaviorSanitizer，有助於找出 memory safety 與 undefined behavior 問題。

### AFL++ source instrumentation 範例

這個範例使用 `afl-clang-fast` 在編譯時插入 coverage 邏輯。與 libFuzzer 直接呼叫函式不同，這裡的 target 是獨立 process，會從檔案讀取 input。

```bash
cd ../03-source-instrumentation

# 確認 AFL++ compiler wrapper 可以使用
afl-clang-fast --version

# make build 實際執行下列編譯指令：
# afl-clang-fast -std=c11 -Wall -Wextra -Wpedantic -O1 -g target.c -o target
make build

# make fuzz 會建立非空 seed，再執行：
# afl-fuzz -i input -o out -- ./target @@
make fuzz
```

<!-- markdownlint-disable MD046 -->
??? example "展開查看 AFL++ 執行結果"

    ```text
    afl-fuzz++4.09c
    [+] Loaded a total of 1 seeds.
    [+] All right - fork server is up.
    [*] Target map size: 24
    [*] Entering queue cycle 1.
    ...
    Statistics: 7 new corpus items found, 45.83% coverage achieved,
    0 crashes saved, 0 timeouts saved
    ```
<!-- markdownlint-enable MD046 -->

`make fuzz` 會自動建立一個非空的 seed，並將它放在 `input/`。AFL++ 會透過 `@@` 將每次產生的 input 檔案路徑傳給 target，測試結果則會放在 `out/default/`：

* `queue/`：能夠觸及新路徑、被保留的 corpus。
* `crashes/`：能夠造成 target crash 的 input。
* `hangs/`：執行時間超過限制的 input。

如果 AFL++ 在練習環境中因 core dump handler 或 CPU frequency scaling 設定而拒絕執行，可以在短時間的學習測試中使用：

```bash
# 實際執行的 afl-fuzz 指令與上方相同，只是額外設定環境變數
AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1 AFL_SKIP_CPUFREQ=1 make fuzz
```

這些環境變數會略過部分系統檢查，不應直接用在長時間的 fuzzing campaign。正式測試時，應依 `afl-fuzz` 的提示調整系統設定。

範例已經提供可以觸發 `FUZZBUG!` 條件的 regression input，可用來確認 crash 能夠穩定重現：

```bash
# make reproduce 實際會將 regression input 傳給 target：
# ./target 'regression/FUZZBUG!'
make reproduce

# make build-sanitized 實際執行：
# afl-clang-fast -std=c11 -Wall -Wextra -Wpedantic -O1 -g \
#   -fsanitize=address,undefined -fno-omit-frame-pointer \
#   target.c -o target-sanitized
make build-sanitized
./target-sanitized 'regression/FUZZBUG!'
```
