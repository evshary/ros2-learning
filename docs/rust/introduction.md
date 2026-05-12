---
title: Rust 介紹
description: 簡介 Rust，並列出常用的 Rust 資源
keywords:
  - Rust
---

Rust 是一門越來越熱門的程式語言，因為他語法的設計，特別有安全性，可以避免很多常犯的 memory 問題。
Rust 目前應用的領域有各類系統程式、網路協定設計、區塊鏈合約等等。
而在車用領域，也開開始有很多廠商為了 safety 問題而開始採用 Rust。他們相信使用 Rust 可以寫出更加安全的程式，進而避免安全性的各種問題。
除了安全性外，Rust 的效能也幾乎和 C/C++ 差不多，很多傳統的專案為了能利用 Rust 的高性能和安全性，常常都會 RIIR (Rewrite it in Rust)。

## 安裝與使用

要使用 Rust，我們需要先安裝 rustup 這套管理 Rust 的工具。
我們可以用 rustup 來指定要使用哪個版本的 Rust，下載方式可以直接用 curl。

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

如果要更新 rustup，他也有自更新的機制。

```bash
rustup update
```

有了 rustup，我們就可以來決定使用哪個版本的 Rust。
這邊用到的是 cargo———Rust 的專案管理和建構工具。

```bash
# 創立新的專案，會產生出 Cargo.toml
cargo new my_project
# 在不產生 binary 下快速確認有無編譯失誤
cargo check
# 建置專案
cargo build
# 編譯 release 版本
cargo build --release
# 運行專案
cargo run
```

我們也可以在編譯過程加上一些參數。

編譯通常是平行化，所以使用的記憶體非常多。
如果想要減少同時編譯的 job 數量，可以用如下方式

```bash
# 用 -j 參數控制
cargo build -j 1
# 用環境變數控制
CARGO_BUILD_JOBS=1 cargo build
```

cargo 其實還有很多其他功能，包括調整程式碼格式 lint、測試、效能等等。
`cargo fmt` 會參考 `rustfmt.toml` 的設定，而 `cargo clippy` 會參考 `clippy.toml` 的設定。

```bash
# 調整程式碼格式，底層會呼叫 rustfmt
cargo fmt
cargo fmt -- --check # 如果只是確認而不去修改
# lint 分析，看程式品質
cargo clippy
cargo -- -D warnings # 設定哪些等級的 lint 要報錯，預設是 warnings
# 測試專案
cargo test
# 評測效能
cargo bench
```

## 常用資源

這邊也列出一些跟 Rust 相關的資源。

* [官網](https://rust-lang.org/)：Rust 官網
* [Rust playground](https://play.rust-lang.org/):線上 compiler，可以用來測試 Rust 語法
* [Compiler Explorer](https://rust.godbolt.org/):可以用來觀察 Rust 編譯成組合語言會長怎麼樣
* [Rust Community Crate](https://crates.io/):社群所開發的套件會放在這邊

## 教學

### Rust 本身

* The Rust Programming Language: 官方文件
    * [英文](https://doc.rust-lang.org/book/?search=)
    * [中文](https://rust-lang.tw/book-tw/)
* Rust by Example
    * [英文](https://doc.rust-lang.org/rust-by-example/)
    * [中文](https://rustwiki.org/zh-CN/rust-by-example/index.html)
* [rustlings](https://github.com/rust-lang/rustlings): 有些 Rust 的練習可以確保知道基本概念
* [The Cargo Book](https://doc.rust-lang.org/cargo/): Rust 官方建構系統與套件管理器 cargo 的詳細介紹
* The Rustonomicon (死靈書): 講解寫 unsafe Rust 要注意的事項
    * [英文](https://doc.rust-lang.org/nomicon/)
    * [中文](https://nomicon.purewhite.io/)
* [Rust Forge](https://forge.rust-lang.org/): 理解 Rust project release、team、contribution 等流程
* [Rust语言圣经(Rust Course)](https://course.rs/about-book.html): 中國人寫的教學
    * [Rust By Practice( Rust 练习实践 )](https://zh.practice.rs/why-exercise.html): 附上的練習題
* [Rust学习笔记](https://skyao.io/learning-rust/): 這個有點像查詢的資料庫

### 影片教學

* [All Rust features explained](https://youtu.be/784JWR4oxOI): Rust 和其他語言不同的特色

### Low-Level Concurrency

* [Rust Atomics and Locks: Low-Level Concurrency in Practice](https://marabos.nl/atomics/): 講解 Rust 底層的 concurrency 如何運作
