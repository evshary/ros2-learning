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
# 跑 doctests
cargo test --doc
# 評測效能
cargo bench
```

## 好用 cargo 工具

cargo 本身也可以直接從 crates.io 下載並編譯安裝執行檔工具，安裝的 binary 會放在 `~/.cargo/bin` 下。

```bash
# 目前有安裝哪些套件
cargo install --list
# 安裝套件
cargo install xxxx
## 鎖定版本
cargo install xxxx --version 1.0.1
## 用 crate 的 Cargo.lock 安裝，避免 dependency 問題
cargo install xxxx --locked
# 解安裝套件
cargo uninstall xxxx
```

好用指令：

* cargo-binstall：優先找 prebuilt binary 安裝，可以省去編譯時間

```bash
# 安裝
cargo install cargo-binstall
# 使用方式
cargo binstall cargo-nextest
```

* cargo-expand：用來看 macro / derive 展開後的 Rust code

```bash
cargo install cargo-expand
# 展開所有程式
cargo expand
# 只產開 my_module
cargo expand my_module
```

* cargo-nextest：next-generation test runner，更好的測試框架，但目前不支援 doctests

```bash
# 安裝要用 locked
cargo install --locked cargo-nextest
# 跑所有測試
cargo nextest run
# 跑 my_crate 測試
cargo nextest run -p my_crate
# 列出測試
cargo nextest list
# 跑 test_name 測項
cargo nextest run test_name
```

* cargo-deny：檢查 dependency graph 的工具，包含
    * advisories：安全漏洞 / RustSec advisories
    * licenses：license 是否允許
    * bans：禁用特定 crate / 檢查 duplicate versions
    * sources：crate 來源是否允許

```bash
# 安裝要用 locked
cargo install --locked cargo-deny
# 初始化設定，會產生 deny.toml
cargo deny init
# 跑全部檢查
cargo deny check
```

* cargo-cache：管理 Cargo cache 中的東西，也就是 `~/.cargo`

```bash
# 安裝
cargo install cargo-cache
# 看目前 cache 狀況
cargo cache
# 移除所有 cache
cargo cache --remove-dir all
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
* Rust by Example
    * [英文](https://doc.rust-lang.org/rust-by-example/)
    * [中文](https://rustwiki.org/zh-CN/rust-by-example/index.html)
* [rustlings](https://github.com/rust-lang/rustlings): 有些 Rust 的練習可以確保知道基本概念
* [The Cargo Book](https://doc.rust-lang.org/cargo/): Rust 官方建構系統與套件管理器 cargo 的詳細介紹
* The Rustonomicon (死靈書): 講解寫 unsafe Rust 要注意的事項
    * [英文](https://doc.rust-lang.org/nomicon/)
    * [中文](https://nomicon.purewhite.io/)
* [Rust Forge](https://forge.rust-lang.org/): 理解 Rust project release、team、contribution 等流程
* [Rust语言圣经(Rust Course)](https://beatai.org/rust-course/about-book): 中國人寫的教學

### 影片教學

* [All Rust features explained](https://youtu.be/784JWR4oxOI): Rust 和其他語言不同的特色

### Low-Level Concurrency

* [Rust Atomics and Locks: Low-Level Concurrency in Practice](https://mara.nl/atomics/): 講解 Rust 底層的 concurrency 如何運作
