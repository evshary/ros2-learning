---
title: 為何選擇 Rust
description: 說明 Rust 的安全機制
keywords:
  - Rust
---

我們常常看到別人說 Rust 比起其他程式語言還要安全，某某公司用 Rust 重寫程式後，減少多少記憶體的 bug，效率提昇多少。
甚至還有一種說法叫 RIIR (Rewrite It In Rust)，很多人都用 Rust 重新改寫既有程式碼。

但是為何 Rust 能夠提高安全性和效率呢？這邊稍微來解釋一下。

## 安全性

基本上 Rust 是在 compile 時間用 type system 來檢查各種規則，確保一些風險可以提前被排除掉。
下面說明各種 type system 用來提高安全性的機制。

### Ownership

Ownership 可以確保所有變數都只有一個擁有者，而這個擁有者要負責將其釋放。
這樣的好處避免

* use-after-free: 釋放後還被使用
* double free: 嘗試重複釋放資源
* memory leak: 忘記釋放資源

```rust
let s = String::from("hello");
let t = s;
// s 的擁有權一個 t，不再能被使用，而 t 要當前 scope 結束時負責釋放
```

### Borrowing

Rust 允許借用資料而不拿走 ownership。
他有個核心原則：同一時間可以有多個 immutable references 或是一個 mutable reference，兩者不能同時存在。
這個可以避免一份資料同時有人讀寫的混亂，造成 data race。

```rust
let mut x = 10;

let a = &x;
let b = &x;
// 可以有兩個 immutable references

let c = &mut x;
// 但是無法再有 mutable reference
```

### Lifetimes

Rust 的 Lifetimes 會檢查這個 reference 是否活得比它指向的資料還久。
這能避免 C / C++ 常出現的 dangling pointer ——— 指向已經不存在的資料。

```rust
fn foo() -> &i32 {
    let x = 10;
    &x // 無法編譯，因為 x 在離開 foo 後就會被釋放
}
```

### enum

Rust 鼓勵用 enum 來表示程式狀態，當使用 match 的時候可以確保我們有處理所有可能情況，不會遺漏。

```rust
enum ConnectionState {
    Disconnected,
    Connecting,
    Connected(Socket),
}

match state {
    ConnectionState::Disconnected => {}
    ConnectionState::Connecting => {}
    ConnectionState::Connected(socket) => {}
}
```

### Result / Option

C/C++ 通常會用負數（如 -1）或是 nullptr 來表示錯誤或是空值，但是 Rust 會用更顯式的方式呈現。
Rust 有 `Option<T>` 以及 `Result<T>`，分別用來表示是否空值，以及是否錯誤。
這樣的好處是讓開發者更一目了然函式回傳結果，以及也強迫要處理所有可能情況。

```rust
let value: Option<i32> = Some(10);
match value {
    Some(v) => println!("{}", v),
    None => println!("no value"),
}

let result: Result<i32, String> = Err("錯誤狀態");
match result {
    Ok(r) => println!("{r}"),
    Err(s) => println!("{s}"),
}
```

### Thread safety

Rust 做了很多事情來確保 Thread safety。

* 首先，要求要傳到某個 thread 的資料需要有 Send trait，保證可以移動。
* 再來，要求多個 thread 共享某個變數必須要有 Sync trait，確保沒有 data race。
* 然後，如果為了確保共享變數的 lifetime 不會比 thread 還早結束，要求要用 Arc 來包裹起來。
* 最後，如果多個 thread 想要同時修改變數，就需要符合 borrowing rule，所以需要 Mutex / RWLock / Atomic。

```rust
let counter = Arc::new(Mutex::new(0));

for _ in 0..2 {
    let counter = Arc::clone(&counter);

    thread::spawn(move || {
        let mut value = counter.lock().unwrap();
        *value += 1;
    });
}
```

### unsafe

最後，有時我們還是無法避免要繞過 Rust 的一些規則，Rust 提供了 unsafe 區塊讓我們把不安全的程式包在限定的區塊。
unsafe 並非是 type system 一環，但是卻是 Rust 中保持一定彈性的重要方法。
這樣我們就只需要關心這些 unsafe 區塊有沒有任何風險即可，可以參考 [unsafe 的教學](./unsafe.md)。

## 效率

Rust 比起很多程式語言還要有效率，最主要的因素是他的設計比較接近 C/C++。
在提供接近底層的方式控制記憶體和資料結構的同時，又用 type system 確保了安全性。

不過同時這也造成了 Rust 在編譯通常會花上不少時間。

下面分幾個部份來探討：

### 沒有 Garbage Collector

很多程式語言為了方便，會有 Garbage Collector 來確保開發者不用手動管理記憶體。
但是這是有很多副作用的，例如額外 CPU 和記憶體使用量、很難預測資源釋放等等。

Rust 則是用 ownership 來確保資源生命週期。

### Zero-cost abstractions

Rust 允許我們用 iterator、generic、trait 等高階抽象，但編譯後會做最佳化。
這樣開發者可以用抽象的方式寫程式，卻不一定犧牲效能。

例如

```rust
let sum: i32 = numbers.iter()
    .filter(|x| **x > 0)
    .map(|x| x * 2)
    .sum();
```

實際編譯後運作會像是

```rust
let mut sum = 0;

for x in &numbers {
    if *x > 0 {
        sum += x * 2;
    }
}
```

### 靜態編譯靜態編譯

前面我們提到的 type system 很多檢查都是在編譯時期就確認的，可以避免 runtime 的時候還要做檢查，降低額外成本。

### 不需要大量 copy

Rust 的 ownership 和 borrowing 可以讓我們安心傳遞 reference，不用每次都複製資料。
最重要的是使用 reference 還不需要特別擔心 data race / dangling reference 等等問題。

### 明示成本

Rust 很多使用的成本都會顯示在型態上面，這樣工程師會自行評估哪些真的有需求，而哪些可以省略。
下面是一些常見的隱藏成本。

| 型別 | 暗示的成本 |
| - | - |
| `Box<T>` | heap allocation |
| `Rc<T>` | reference counting |
| `Arc<T>` | atomic reference counting |
| `Mutex<T>` | lock / unlock |
| `dyn Trait` | dynamic dispatch |
| `String` | heap allocation |
| `Vec<T>` | heap allocation |
