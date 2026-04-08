---
title: Rust unsafe
description: Rust unsafe 機制
keywords:
  - Rust
---

雖然 Rust 在編譯時期就可以幫忙確認一些安全性的問題，但是有時候為了方便，我們會希望可以省略這些確認。
這個就是 Rust 中所謂的 unsafe 的機制。

## unsafe 省略的檢查

當我們用 unsafe block 把程式碼框起來時，並不代表所有檢查都可以跳過，主要是可以要編譯器省略下面五種檢查：

<details>
  <summary>解引用原始指標(dereferencing raw pointers)</summary>

```rust
let mut num = 5;
// 建立原始指標是安全的 (Safe)
let r1 = &num as *const i32; 
let r2 = &mut num as *mut i32;

unsafe {
    // 解引用（讀取或寫入）是不安全的 (Unsafe)
    println!("r1: {}", *r1);
    *r2 = 10;
    println!("修改後的 num: {}", num);
}
```

</details>

<details>
  <summary>呼叫 Unsafe 函式</summary>

```rust
let v = vec![1, 2, 3];
// 安全寫法：v[10] 會 panic

unsafe {
    // 這是 Unsafe 函式，它假設索引 10 絕對存在
    // 如果不存在，會讀取到隔壁鄰居的記憶體（未定義行為）
    let val = v.get_unchecked(10); 
    println!("抓到值了：{}", val);
}
```

</details>

<details>
  <summary>修改可變靜態變數(static mut)</summary>

```rust
static mut COUNTER: u32 = 0;
fn add_to_counter(inc: u32) {
    unsafe {
        // 存取與修改全域變數必須在 unsafe 中
        COUNTER += inc;
    }
}
fn main() {
    add_to_counter(5);
    unsafe { println!("Counter: {}", COUNTER); }
}
```

</details>

<details>
  <summary>存取 Union 的欄位</summary>

```rust
#[repr(C)]
union MyValue {
    f: f32,
    u: u32,
}

fn main() {
    let mut val = MyValue { u: 1065353216 }; // 這在二進位中剛好是 1.0 (f32)
    unsafe {
        // 編譯器不知道現在裡面存的是 u 還是 f
        // 如果你存入 u 卻讀取 f，編譯器會隨便你
        println!("以浮點數讀取：{}", val.f);
    }
}
```

</details>

<details>
  <summary>實作 Unsafe Trait</summary>

要求使用該 trait 的使用者必須確定符合某個 safety 標準，類似簽下切結書表示自己已經知道

```rust
// 假設我們有一個標記 Trait，代表這個結構可以在某些極限環境下運行
unsafe trait SecureStorage {}
struct MyVault;
// 實作時必須加上 unsafe，代表「我發誓我已經處理好安全細節了」
unsafe impl SecureStorage for MyVault {}

fn main() {
    // 使用這個 Trait 的函式會相信 MyVault 已經符合規則
}
```

</details>

## unsafe 使用情境

上面提到的是 unsafe 所省略的檢查，而為何我們需要省略這些檢查呢？
主要有這四種使用情境：

* 要進行記憶體操作：如果要實作 memory pool、linked list 等等直接記憶體的操作，我們必須要用 unsafe 來做
* 避免 borrow check：某些資料結構，例如 Graph 的實作，會需要避開 Rust 編譯器的借用檢查
* FFI(Foreign Function Interface)：如果要呼叫其他程式語言的函式，一定得用 unsafe 才行
* 增加效能：一般 Rust 都會自行做一些檢查，如果我們希望可以完全省略，就必須使用 unsafe，例如 `*data.get_unchecked(index)` 就等同於 `data[index]`，但是省略檢查合法範圍

## unsafe 使用注意事項

* 我們應該要儘可能讓 unsafe 的區域越小越好，讓編譯器幫我們檢查大多數的程式碼
* `unsafe fn` 有兩層含意：
    * 開發者希望使用者要特別注意呼叫該函式需要符合某些前提條件，才能夠避免 UB (Undefined Behavior)
    * 等同於此函式內部被一個大的 unsafe block 包住，所以可以任意在內部使用 unsafe 語法
        * 不過這個通常不太被建議使用，一般還是要用有限的 unsafe block 包住 unsafe 語法比較好。
        * [`unsafe_op_in_unsafe_fn`](https://doc.rust-lang.org/nightly/edition-guide/rust-2024/unsafe-op-in-unsafe-fn.html) 會在 Edition 2024 預設被啟動，如果沒有符合就會收到編譯器的警告
* `unsafe fn` 是責任語法，意思是是否要用 `unsafe fn` 完全由開發者決定，編譯器不會介入
* 由於 `unsafe fn` 代表要使用者小心使用前提，如果我們可以藉由一些檢查來避開使用 `unsafe fn`，那就應該要這麼做
* 在 `unsafe fn` 之前應該要有 `/// # Safety` 的註解說明為何這是 unsafe，以及要滿足何種條件才能使用
* 在 unsafe block 之前應該要有 `// SAFETY:` 的註解來解釋這個區塊為何不安全

### unsafe 範例

假設我們有一個 unsafe 函式如下

```rust
pub unsafe fn read_at_safe(data: &[i32], index: usize) -> i32 {
    *data.as_ptr().add(index)
}
```

他的功用是存取 data 這個 array 的第 index 元素，等價於 `data[index]`。
然而這在 unsafe 使用上有些可以改善的地方

```rust
/// # Safety
/// Caller must ensure that:
/// - `index < data.len()`
///
/// This function performs unchecked pointer access.
pub unsafe fn read_at_safe(data: &[i32], index: usize) -> Option<i32> {
    // SAFETY:
    // The caller guarantees that `index` is in-bounds.
    unsafe {
        Some(*data.as_ptr().add(index))
    }
}
```

我們在函式的最開頭加上 `/// # Safety`，告訴使用者這個 `unsafe fn` 應該要符合什麼條件。
除此之外，也用 `unsafe {}` 把不安全的函式框起來，並且在前面標注 `// SAFETY` 告知安全的條件為何。
不過，其實我們還可以更進一步。

```rust
pub fn read_at_safe(data: &[i32], index: usize) -> Option<i32> {
    if index < data.len() {
        // SAFETY:
        // - `index < data.len()` ensures the access is in-bounds.
        // - `data.as_ptr()` is valid for `data.len()` elements.
        // - `add(index)` stays within the allocated object.
        // - The element is initialized, so dereferencing is safe.
        unsafe {
            Some(*data.as_ptr().add(index))
        }
    } else {
        None
    }
}
```

我們發現其實這個函式唯一會發生不安全的情況就是 index 超過 data 的合理長度。
如果我們能再存取之前就先做好檢查，這樣就根本不需要將這個函式當成是 `unsafe fn`，更加符合使用 unsafe 的規範。

### 檢查 unsafe_op_in_unsafe_fn

要如何來檢查 `unsafe_op_in_unsafe_fn` 呢？

我們可以直接跑 `cargo clippy -- -D unsafe_op_in_unsafe_fn`

又或者是在 `Cargo.toml` 加上

```toml
[lints.rust]
unsafe_op_in_unsafe_fn = "warn"
```
