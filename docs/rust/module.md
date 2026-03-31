---
title: Rust Module
description: Rust 模組概念
keywords:
  - Rust
---

Rust 的 project 架構有很多名詞，不過從大到小，大概如下： workspace > package > crate > mod

```raw
. <------------------------------ workspace
├── Cargo.lock
├── Cargo.toml --- [workspace]
│                  members = ["pakcageA", "packageB"]
├── packageA
│   └...
└── packageB <------------------- package
    ├ Cargo.toml
    ├ tests/    (test)
    ├ benches/  (benchmark)
    ├ examples/ (exmple)
    └ src/      (source code)
      ├── lib.rs <--------------- crate (includes mod)
      ├── main.rs <-------------- crate (includes mod)
      └── bin/
          ├── bin1.rs <---------- crate (includes mod)
          └── bin2.rs <---------- crate (includes mod)
```

## workspace

workspace 是多個 packages 的集合，有兩種類型

1. root package: 最外層(根目錄)是一個 package，底下包含了其他 package
2. virtual manifest: 最外層(根目錄)不是 package，在 workspace 底下的所有 package 都在單獨目錄中

workspace 的特性為

1. 所有 package 共用同個 Cargo.lock，位於根目錄下，確保大家使用相同版本的 dependency
2. 所有 package 的輸出都會集中在根目錄下的 target 資料夾
3. 只有根目錄下的 Cargo.toml 的 `[patch]`, `[replace]`, `[profile.*]` 才有效用，其他 package 的則是會被忽略

## package && crate

很多時候我們不一定需要 workspace，這時候最大的單位就是一個 package

一個 package 可以包括多個 binary crates，和最多一個 library crates

### 引用外部 library

在 `Cargo.toml` 的 `[dependencies]` 下放入 library，crate 即可直接用 use。

```toml
[dependencies]
futures="0.3.12"
zenoh={git="url"}
```

## mod

一個 crate 中可以多個 mod，也可以視為是 namespace

舉個例子，假設我們在同個 crate 中有 2 個 mod，要呼叫對方的 function 有兩個方法：絕對路徑和相對路徑

```rust
mod a {
    pub mod a1 {
        pub fn a2() {}
    }
}

mod b b {
    pub fn test() {
        // absolute
        create::a::a1::a2();
        // relative
        super::a::a1::a2();
    }
}
```

### 可見性

另外，也要注意在 mod 中需要加上 pub 才能讓其他人 access，這也就是可見性的設定，有如下幾種

* `pub`: 可見性沒限制
* `pub(crate)`: 當前 crate 可見
* `pub(self)`: 當前 mod 可見
* `pub(super)`: 在父 mod 可見
* `pub(in <path>)`: 在某路徑的 mod 可見，例如 `pub(in crate::a)`

### use & mod 差異

在程式中，我們可以看到常常會有 `use xxx` 和 `mod xxx` 的出現，兩者差異如下：

* use: 用來縮寫，可以不用寫那麼長的路徑
* mod: 兩種用法
    * 區分 namespace
    * 引入其他的檔案

舉例來說，假設有兩個檔案，兩者可以這樣使用

file.rs

```rust
pub mod module {
    pub fn func1() {}
}
```

lib.rs

```rust
mod file;  // <--- 尋找當前目錄的 file.rs 或是 file 資料夾底下的 mod.rs
pub use crate::file::module; // <--- 縮寫
pub fn test() {
    // 原來應該要寫成 crate::file::module::func1()
    module::func1();
}
```
