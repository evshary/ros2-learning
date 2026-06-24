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
* Coverage-guided：透過 LLVM instrumentation，知道 input 會經過程式走過的路徑 (converage)，並且依此來調整 input
* 透過 LLVM 的 sanitizer 來鎖定各種常見的問題，例如 ASan、UBSan 等等

AFL 是另外一種 fuzz test 類型，主要基於外部的 fuzz engine。
AFL 可以運用在沒有程式碼的二進位檔案直接測試，更加有彈性。
主要特色是：

* AFL 如果有程式碼，可以在編譯時自行加入 intrumentation 來偵測 coverage。如果沒有的話，則是用 QEMU mode / Frida mode 等等方式動態偵測 coverage
* Fork server：透過 fork 快速複製 process，在確保隔離的情況下，提高執行速度
* 外部控制：會有外部的 engine 來控制輸入和執行

不管是哪種基本上都會有如下特色：

* 透過 coverage 來尋找新的可能路徑
* 利用 mutation 來變化 input
* 收集有價值的 corpus 集合，提供未來測試
* 發生問題的時候會嘗試收斂造成問題的最小可能 input

### 如何選擇

* 如果有程式原始碼，最好的方式就是直接用 libFuzzer 或 AFL 加上 harness 來測試，更好得到 coverage
    * 可以根據自己使用的程式語言生態，速度和隔離性的平衡來看要用 libFuzzer 或 AFL
* 如果只有二進位程式或無法重新編譯，基本上只能選擇 AFL 來做到 binary-only coverage-guided fuzzing。
* 如果程式本身無法直接在 userspace 執行（例如 CPU 架構不同、或是要跑在某個 firmware 之上等等），就可以考慮搭配 qemu 或 Renode 來進行 fuzz 測試

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
