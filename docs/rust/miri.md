---
title: Rust Miri
description: 使用 Miri 檢查 Rust 程式中的未定義行為
keywords:
  - Rust
  - Miri
---

Miri 是 Rust 的未定義行為（Undefined Behavior，UB）檢查工具，特別適合用來測試 `unsafe` 程式碼。
例如越界指標存取、使用已釋放的記憶體，以及違反指標別名規則等問題，可能通過編譯甚至看似正常執行，但 Miri 可以協助找出它們。

Miri 的原理是使用直譯器執行 Rust 的中介表示（MIR），並在執行過程中檢查每一次記憶體操作是否符合 Rust 的規則。
因此它不是形式驗證，也不能保證找出所有問題：只有實際執行到的程式路徑才會被檢查。

## 安裝 Miri

Miri 需要 Rust nightly toolchain。使用 `rustup` 安裝 nightly、Miri 與建立 Miri sysroot 所需的 Rust 原始碼：

```bash
rustup toolchain install nightly
rustup component add miri rust-src --toolchain nightly
```

確認安裝成功：

```bash
cargo +nightly miri --version
```

## 最簡單的範例

先建立一個專案：

```bash
cargo new miri-example
cd miri-example
```

將 `src/main.rs` 改成：

```rust
fn main() {
    let numbers = [10, 20, 30];
    let pointer = numbers.as_ptr();

    unsafe {
        println!("{}", *pointer.add(3));
    }
}
```

陣列只有三個元素，所以合法索引是 0 到 2。程式卻使用 `pointer.add(3)` 讀取陣列外的記憶體，這是未定義行為。

執行 Miri：

```bash
cargo +nightly miri run
```

Miri 會停止程式並指出問題所在，錯誤訊息類似：

```text
error: Undefined Behavior: constructing invalid value of type &i32:
encountered a dangling reference (going beyond the bounds of its allocation)
```

這個錯誤代表 Miri 成功抓到問題。將 `pointer.add(3)` 改成 `pointer.add(2)` 後再次執行，程式會正常輸出 `30`。

如果要檢查專案中的測試，可以改用：

```bash
cargo +nightly miri test
```

## 限制

Miri 執行速度比一般程式慢，並且不支援所有作業系統或 FFI 操作，因此適合用來執行範圍較小、專門測試 `unsafe` 程式碼的測試案例。
