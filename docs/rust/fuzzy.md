---
title: Rust fuzzy
description: 簡介怎麼在 Rust 進行各種 fuzzy 測試
keywords:
  - Rust
---

fuzzy 測試專門用在產生隨機的輸入，來確保程式的結果都會符合我們預期。
fuzzy 測試和一般測試的不同在於，我們不會預先知道他的輸入，所以更能夠發現一些預想不到的問題。
很多安全漏洞或是 bug 都是透過 fuzzy 測試來找出來的。

在 Rust 中如果要使用 fuzzy 測試，有幾個推薦的函式庫。
我們將其分成兩大類：

* fuzzy 測試框架：會不斷去修改輸入的 byte，並餵給待測函式，並且提供測試結果/報告，甚至有些可以直接開啟 debugger 來找問題。
* 輔助測試函式庫：如果輸入只是單純 raw data 的改變，測試還是會受限，我們需要其他函式庫幫忙，例如產生特殊的 struct，又或者是根據一定規則來產生測試輸入

一般來說是選擇一個 fuzzy 測試框架，然後另外搭配輔助測試函式庫。

## 分類

### fuzzy 測試框架

* [cargo-fuzz](https://github.com/rust-fuzz/cargo-fuzz)
    * Rust 社群最熱門的選擇，一般來說會推薦優先使用
    * 只有單一 process，速度非常快
    * 但也因此缺乏 process 隔離，狀態可能會被污染
* [afl.rs](https://github.com/rust-fuzz/afl.rs)
    * 提供一層 Rust wrapper 來使用 AFL (American Fuzzy Lop)
    * mutation（變換 input）的能力很強
    * 使用 fork process 的方式測試，隔離很好
    * 但速度也相較更慢
* [honggfuzz-rs](https://github.com/rust-fuzz/honggfuzz-rs)
    * 提供一層 Rust wrapper 來使用 Google 所開發的 Honggfuzz
    * 有整合 debugger、sanitizer (ASan/UBSan)，方便進行分析
    * 同樣也有用 fork process 來隔離

### 輔助測試函式庫

* [arbitrary](https://github.com/rust-fuzz/arbitrary)
    * 測試框架一般只有產生 raw byte 輸入，但是實際函式的輸入會是 struct，我們需要將 raw byte 轉換成 struct，這就是 arbitrary 幫我們做的事情
* [prop-test](https://github.com/proptest-rs/proptest)
    * 我們可以針對測試輸入設定一些規則，而不是使用完全隨機的輸入，這樣更有機會幫我們鎖定應該測試的範圍
    * prop-test 還可以在發現某個錯誤時，主動幫我們縮小測試的範圍，鎖定到底是誰導致這個錯誤產生

## 實戰

這邊弄了一個範例 GitHub repo [rust_fuzzy_examples](https://github.com/evshary/rust_fuzzy_examples)，幫助我們快速理解這些函式庫的使用。

這個範例是一個將字串轉為數字的函式，但是留了一個 bug，只要字串是是 "0"，就會導致函式 panic。
我們分別用不同 fuzz 函式庫來說明。

### cargo-fuzzy

```bash
cd cargo_fuzzy_example
# 安裝
cargo install cargo-fuzz
rustup toolchain install nightly
# fuzzy 測試 parse_port
cargo +nightly fuzz run parse_port
```

可以看到 cargo-fuzzy 是在當前 crate 下另外有一個 fuzz 的資料夾，放 fuzzy test 相關的程式碼。
運行的結果，正常沒問題的 input 會放在 corpus 資料夾，會 crash 的 input 就放到 artifacts 資料夾下。

要特別注意的是 cargo-fuzzy 會需要使用 nightly feature，所以執行時要加上 `+nightly`。

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
# 跑 fuzzy test
cargo hfuzz run parse_port
```

`hfuzz_workspace/parse_port/input/seed.txt` 要填入一般正常的輸入，然後運行失敗的結果和報告會被放到 `hfuzz_workspace/parse_port/`。

### arbitrary

```bash
cd arbitrary_example
# 測試 (我們寫成一般 Rust 測試，一般來說是跟前面的 fuzzy framework 搭配
cargo test
```

我們可以先在要當成輸入的 struct 加上 `#[derive(Arbitrary)]`，然後再用 `arbitrary::Unstructured` 把一般的 byte 輸入轉換成 struct。

### proptest

```bash
cd proptest_example
# 測試 (我們寫成一般 Rust 測試，一般來說是跟前面的 fuzzy framework 搭配
cargo test parse_port_matches_std -- --ignored
```

運行測試發現錯誤時，proptest 會產生最小可複製的測試 case 放在 `proptest-regressions/` 資料夾下，下次再次執行時，會優先從這個資料夾執行有問題的 case。
