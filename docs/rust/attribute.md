---
title: Rust Attribute
description: Rust 屬性系統
keywords:
  - Rust
---

在看 Rust 的程式的時候會常常看到 `#[]`，這個是 Rust 的屬性系統 (attribute)。
他的功能在於提供給編譯器或其他工具在編譯時的一些額外資訊，例如改變編譯行為、code 生成、lint、macro 展開等等。

下面是最基本的語法

```rust
// 單純 attribute，例如 #[test]
#[attribute]
// 帶入 key、value，例如 #[cfg(feature = "foo")]
#[attribute(key = "value")]
// 帶入其他參數，例如 #[allow(dead_code)]
#[attribute(arg1, arg2)]
```

## 分類

一般來說有下面這幾個分類

### 編譯器內建 attribute

調整 compiler 的行為，例如 cfg、inline、repr、derive 等等

* 條件編譯：決定某個程式是否存在

```rust
#[cfg(feature = "foo")]
#[cfg(target_os = "linux")]
```

* derive：產生實作

```rust
#[derive(Debug, Clone, PartialEq)]
struct A {
    x: i32,
}
```

* 編譯優化 / 行為控制

```rust
#[inline]
#[inline(always)]
#[inline(never)]
```

* 測試相關

```rust
#[test]
fn test_add() {
    assert_eq!(1 + 1, 2);
}

#[test]
#[ignore = "requires network access"]
fn test_api() {}
```

* repr：記憶體布局控制

```rust
#[repr(C)]
struct MyStruct {
    a: u8,
    b: u32,
}
```

* doc：產生文件，`///` 就是 `doc` 的語法糖

```rust
#[doc = "This is a function"]
fn foo() {}
// 等同於
/// This is a function
fn foo() {}
```

* feature： 把某些 nightly 的功能啟動

```rust
#![feature(async_fn_in_trait)]
```

### lint attribute

lint / warning 控制，決定哪些要跳出 warning

```rust
#[allow(dead_code)]
#[deny(unused_variables)]
#[warn(missing_docs)]
```

### macro attribute

也可以用 attribute 來展開 macro

```rust
#[macro_use]
extern crate foo;
```

```rust
#[tokio::main]
async fn main() {}
```

## crate/mod level attribute

如果我們想要讓某個 attribute 附在整個 scope 上，那就需要使用 `#![...]`

* crate level

```rust
#![no_std]
#![feature(async_fn_in_trait)]

fn main() {}
```

* mod level

```rust
mod foo {
    #![allow(unused)]

    fn bar() {}
}
```

## feature

feature 是其中一個最常用到的 attribute，用來決定編譯時期哪些功能要被啟動

在 `Cargo.toml` 中加上 `[features]`

```rust
[features]
default = ["feature_a"]

feature_a = []
feature_b = []
```

程式中就可以用 `#[cfg(feature = "xxx")]`

```rust
#[cfg(feature = "feature_a")]
fn foo() {
    println!("feature_a enabled");
}

#[cfg(not(feature = "feature_a"))]
fn foo() {
    println!("feature_a disabled");
}
```

編譯時期可以決定要開哪個 feature

```bash
# 開 feature_a
cargo build --features feature_a
# 開 feature_a 和 feature_b
cargo build --features "feature_a feature_b"
# 不要開任何 feature (default的)
cargo build --no-default-features
# 不要開任何 default feature，然後再開 feature_b
cargo build --no-default-features --features feature_b
```

還有其他進階使用方法，例如條件整理

```rust
// 有 foo
#[cfg(feature = "foo")]
// 沒有 foo
#[cfg(not(feature = "foo"))]
// 需要有 a 和 b
#[cfg(all(feature = "a", feature = "b"))]
// a 和 b 任一成立即可
#[cfg(any(feature = "a", feature = "b"))]
```
