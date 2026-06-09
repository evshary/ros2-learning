---
title: Rust Error
description: Rust 中 thiserror 和 anyhow 的使用
keywords:
  - Rust
---

在 Rust 中有兩個很常被用來處理錯誤訊息的 crate，也很容易被搞混，這邊比較一下用法。

* `thiserror`: 可以自己定義錯誤類型，通常用在 library API 的設計之中。
* `anyhow`: 要快速簡潔傳達錯誤訊息，不需要細部的錯誤種類，通常用在 application 快速回傳錯誤給上層 caller。

也可以兩者同時使用，內部都用 thiserror 定義的 type，但對外則是用 anyhow 加上 context 訊息並向上傳遞。

## thiserror

我們可以把程式會遇到的各式各樣錯誤用 thiserror 統籌管理，定義屬於我們自己的錯誤格式

```rust
use std::fs;
use thiserror::Error;

// 定義我們自己的錯誤型別。
// thiserror 會幫這個 enum 自動實作 std::error::Error。
#[derive(Debug, Error)]
enum AppError {
    // #[from] 的意思是：
    // 可以自動把 std::io::Error 轉成 AppError::ReadFile。
    // 因此 fs::read_to_string(path)? 可以直接使用 ?。
    #[error("failed to read file: {0}")]
    ReadFile(#[from] std::io::Error),

    // 例如檔案內容是 "abc"，parse::<i32>() 就會失敗。
    // #[from] 讓 ParseIntError 可以自動轉成 AppError::ParseNumber。
    #[error("failed to parse number: {0}")]
    ParseNumber(#[from] std::num::ParseIntError),
}

fn read_number_from_file(path: &str) -> Result<i32, AppError> {
    // fs::read_to_string 失敗時會回傳 std::io::Error。
    // 因為 AppError::ReadFile 有 #[from]，
    // 所以 ? 會自動把 std::io::Error 轉成 AppError。
    let content = fs::read_to_string(path)?;

    // 如果 parse 轉換失敗，會得到 ParseIntError。
    // 因為 AppError::ParseNumber 有 #[from]，
    // 所以 ? 會自動把 ParseIntError 轉成 AppError。
    let number = content.trim().parse::<i32>()?;

    Ok(number)
}

fn main() {
    match read_number_from_file("number.txt") {
        Ok(n) => println!("number = {}", n),
        Err(e) => eprintln!("error: {}", e),
    }
}
```

## anyhow

```rust
use anyhow::{bail, Context, Result};
use std::fs;

// anyhow::Result 可以省略 error 的部份，原本應該是 Result<i32, anyhow::Error>
// 注意的是錯誤部份要有 std::error::Error 才能轉換成 anyhow::Error
fn read_number_from_file(path: &str) -> Result<i32> {
    // 如果讀檔失敗，with_context 會在原本錯誤外面加上更多資訊。
    let content = fs::read_to_string(path)
        .with_context(|| format!("failed to read file: {}", path))?;

    // 如果 parse 失敗，也加上目前讀到的內容，方便 debug。
    let number = content
        .trim()
        .parse::<i32>()
        .with_context(|| format!("failed to parse number from content: {:?}", content.trim()))?;

    // bail! 會直接回傳 Err(anyhow::Error)。
    // 這裡用來處理「不是系統錯誤，但不符合我們程式規則」的情況。
    if number <= 0 {
        bail!("number must be positive, got {}", number);
    }

    Ok(number)
}

fn main() -> Result<()> {
    // 對於 application 來說，我們不在乎 error type，而是希望把錯誤訊息傳到最上層方便辨認，這時候就適合 anyhow
    let number = read_number_from_file("number.txt")?;

    println!("number = {}", number);

    Ok(())
}
```
