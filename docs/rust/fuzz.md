---
title: Rust 模糊測試
description: 簡介怎麼在 Rust 進行各種 fuzz 測試
keywords:
  - Rust
---

fuzz 測試專門用在產生隨機的輸入，來確保程式的結果都會符合我們預期。
fuzz 測試和一般測試的不同在於，我們不會預先知道他的輸入，所以更能夠發現一些預想不到的問題。
很多安全漏洞或是 bug 都是透過 fuzz 測試來找出來的。

關於 fuzz 測試，並不是測越久就越好，重點是要看測試的 coverage 有沒有提昇。
一般來說 fuzz 測試都會搭配編譯器來評估每個 input 有走過哪些程式路徑，走得越多就代表測試越完整，我們的信心度也會比較大。
好的 fuzz 測試工具會將能夠走新路徑的 input 存起來（稱之為 corpus），並且基於這些 corpus 來進行不同的變化。

在 Rust 中如果要使用 fuzz 測試，有幾個推薦的函式庫。
我們將其分成兩大類：

* fuzz 測試框架：會不斷去修改輸入的 byte，並餵給待測函式，並且提供測試結果/報告，甚至有些可以直接開啟 debugger 來找問題。
* 輔助測試函式庫：如果輸入只是單純 raw data 的改變，測試還是會受限，我們需要其他函式庫幫忙，例如產生特殊的 struct，又或者是根據一定規則來產生測試輸入

一般來說是選擇一個 fuzz 測試框架，然後另外搭配輔助測試函式庫。

## 分類

### fuzz 測試框架

* [cargo-fuzz](https://github.com/rust-fuzz/cargo-fuzz)
    * 使用 LLVM 提供的 fuzzing engine —— libFuzzer
    * Rust 社群最熱門的選擇，一般來說會推薦優先使用
    * 只有單一 process，速度非常快
    * 但也因此缺乏 process 隔離，狀態可能會被污染

<!-- markdownlint-disable MD046 -->
??? "libFuzzer 補充"

    libFuzzer 是 fuzz test 很常被使用的技術，是 LLVM toolchain 所提供的 fuzzing runtime library。
    最主要的優勢有：
    
    * Coverage-guided：透過 LLVM instrumentation，知道 input 會經過程式走過的路徑 (converage)，並且依此來調整 input
    * Mutation：可以用各種技術來變化 input
    * Corpus：會收集目前有價值的 input 集合
    * In-process execution：同個 process 呼叫不同 function，速度很快，但也容易造成 global state 污染
    * Minimization：發生 crash 的時候，會自動找出造成 crash 的最小 input

<!-- markdownlint-enable MD046 -->

* [afl.rs](https://github.com/rust-fuzz/afl.rs)
    * 提供一層 Rust wrapper 來使用 AFL (American Fuzzy Lop)，是外部的 fuzz engine
    * mutation（變換 input）的能力很強
    * 使用 fork process 的方式測試，隔離很好
    * 但速度也相較更慢

<!-- markdownlint-disable MD046 -->
??? "AFL 補充"

    AFL 是另一種 fuzz test 的類型，主要是基於外部的 fuzz engine。
    主要的特色有：
    
    * Coverage-guided：有自己的 instrumentation 技術，可以用在 clang 或 gcc 等不同編譯器
    * Mutation：可以用各種技術來變化 input
    * Corpus：會收集目前有價值的 input 集合
    * Fork server：透過 fork 快速複製 process，在確保隔離的情況下，提高執行速度
    * 外部控制：會有外部的 engine 來控制輸入和執行

<!-- markdownlint-enable MD046 -->

* [honggfuzz-rs](https://github.com/rust-fuzz/honggfuzz-rs)
    * 提供一層 Rust wrapper 來使用 Google 所開發的 Honggfuzz，是外部的 fuzz engine
    * 有整合 debugger、sanitizer (ASan/UBSan)，方便進行分析
    * 同樣也有用 fork process 來隔離
* [libAFL](https://github.com/AFLplusplus/LibAFL)
    * 可以自己定義 fuzz engine，而不是使用他人定義好的框架
    * 學習成本高，但也有最高的彈性
* [bolero](https://github.com/camshaft/bolero)
    * 能夠一個工具搞定 fuzz test + property testing + shrinking
    * 不過算是新興專案，社群活躍度並沒有那麼大

### 輔助測試函式庫

* [arbitrary](https://github.com/rust-fuzz/arbitrary)
    * 測試框架一般只有產生 raw byte 輸入，但是實際函式的輸入會是 struct，我們需要將 raw byte 轉換成 struct，這就是 arbitrary 幫我們做的事情
* [prop-test](https://github.com/proptest-rs/proptest)
    * 我們可以針對測試輸入設定一些規則，而不是使用完全隨機的輸入，這樣更有機會幫我們鎖定應該測試的範圍
    * prop-test 還可以在發現某個錯誤時，主動幫我們縮小測試的範圍，鎖定到底是誰導致這個錯誤產生

## 實戰

這邊弄了一個範例 GitHub repo [rust_fuzz_examples](https://github.com/evshary/rust_fuzz_examples)，幫助我們快速理解這些函式庫的使用。

這個範例是一個將字串轉為數字的函式，但是留了一個 bug，只要字串是是 "0"，就會導致函式 panic。
我們分別用不同 fuzz 函式庫來說明。

### cargo-fuzz

```bash
cd cargo_fuzz_example
# 安裝
cargo install cargo-fuzz
rustup toolchain install nightly
# fuzz 測試 parse_port
cargo +nightly fuzz run parse_port
# 嘗試找出最小造成 bug 的 input
cargo +nightly fuzz tmin parse_port fuzz/artifacts/parse_port/<crash-file>
```

可以看到 cargo-fuzz 是在當前 crate 下另外有一個 fuzz 的資料夾，放 fuzz test 相關的程式碼。
運行的結果，正常沒問題的 input 會放在 corpus 資料夾，會 crash 的 input 就放到 artifacts 資料夾下。

要特別注意的是 cargo-fuzz 會需要使用 nightly feature，所以執行時要加上 `+nightly`。

隨著我們測試的時間拉長，corpus 的數量也就會隨之增加，我們也可以利用這些 corpus 來評估針對 source code 的部份測試覆蓋度（有哪些程式碼有被測試到）

```bash
# 重新編譯 binary，涵蓋 coverage instrumentation，這能用來統計所有部份程式走過的路徑
# 然後他就會重跑 fuzz/corpus/parse_port/* 底下所有 corpus
# 把統計結果輸出 raw profile，也就是 *.profraw
# 並且 merge 多個 *.profraw 到 coverage.profdata
cargo +nightly fuzz coverage --sanitizer none parse_port fuzz/corpus/parse_port
# 用 LLVM 的工具比較 parse_port 這個 binary 內的 coverage mapping 和 coverage.profdata 內的計數結果，回推出整體的 coverage 結果
LLVM_BIN="$(dirname "$(rustc +nightly --print target-libdir)")/bin"
# 產生 summary report
"$LLVM_BIN"/llvm-cov report \
  target/x86_64-unknown-linux-gnu/coverage/x86_64-unknown-linux-gnu/release/parse_port \
  --instr-profile fuzz/coverage/parse_port/coverage.profdata
# 或是用 HTML 顯示
"$LLVM_BIN"/llvm-cov show \
  target/x86_64-unknown-linux-gnu/coverage/x86_64-unknown-linux-gnu/release/parse_port \
  --instr-profile fuzz/coverage/parse_port/coverage.profdata \
  src/lib.rs
```

### afl.rs

```bash
cd afl_rs_example
# 安裝
cargo install cargo-afl
cargo afl system-config
# 編譯 afl 測試的 binary，特別注意這邊我是故意用 afl-harness feature 來隔開，不影響正常程式運作
cargo afl build --features afl-harness --bin fuzz_parse_port
# 用 in/seed.txt 來當作 input 基底來隨機調整，結果會放到 out 資料夾
cargo afl fuzz -i in -o out target/debug/fuzz_parse_port
```

測試會造成 crash 的 input 會放在 `out/default/crashes` 底下。

### honggfuzz-rs

```bash
cd honggfuzz_rs_example
# 安裝
cargo install honggfuzz
# 跑 fuzz test
cargo hfuzz run parse_port
```

`hfuzz_workspace/parse_port/input/seed.txt` 要填入一般正常的輸入，然後運行失敗的結果和報告會被放到 `hfuzz_workspace/parse_port/`。

### libAFL

```bash
cd libafl_example
# 我們可以用 libAFL 產生出 input，然後再餵給待測函數
cargo run
```

可以觀察 `mutate_seed` 函式，libAFL 給我們很大的彈性來看如何產生測試的 input。

### bolero

```bash
cd bolero_example
# 安裝
cargo install cargo-bolero
rustup toolchain install nightly
# 跑 fuzz test
cargo +nightly bolero test parse_port_matches_std
```

可以觀察 `parse_port_matches_std`，我們可以在 input 這邊多動一些手腳。

### arbitrary

```bash
cd arbitrary_example
# 測試 (我們寫成一般 Rust 測試，一般來說是跟前面的 fuzz framework 搭配
cargo test
```

我們可以先在要當成輸入的 struct 加上 `#[derive(Arbitrary)]`，然後再用 `arbitrary::Unstructured` 把一般的 byte 輸入轉換成 struct。

### proptest

```bash
cd proptest_example
# 測試 (我們寫成一般 Rust 測試，一般來說是跟前面的 fuzz framework 搭配
cargo test parse_port_matches_std -- --ignored
```

運行測試發現錯誤時，proptest 會產生最小可複製的測試 case 放在 `proptest-regressions/` 資料夾下，下次再次執行時，會優先從這個資料夾執行有問題的 case。

## 參考連結

* [Rust Fuzz Book](https://rust-fuzz.github.io/book/): 主要是介紹 Rust 中 afl.rs and cargo-fuzz 這兩個測試工具。
