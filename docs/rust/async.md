---
title: Rust async
description: Rust async 概念說明
keywords:
  - Rust
---

為什麼會需要 async 呢？
最主要的是因為程式的運作有時候會需要等待 I/O 的回應，等待的過程中勢必將資源讓給其他人執行。
一般我們會想說開 thread，但是當這個 I/O 數量很大的時候（例如同時有成千上百的網路連線），開 thread 的 overhead 就會很大。
這時候就是 async 可以派上用場的地方。

* async 的優點：開銷比 thread 還要小
* async 的缺點：因為是 FSM 的架構，所以編譯出來的程式碼會很大且複雜

一般來說，thread 適合 CPU bound 的程式，async 適合 I/O bound 的程式。

常用的名詞

* future：延遲的計算。也就是尚未執行的程式區塊，要交給 executor 執行
* task：future 被放入要給 executor 執行的 queue，就叫做 task

## 常用 function

* `block_on`：只能用在非 async，會呼叫 future，當下 thread block，並且執行 async task (future)
* `spawn`：用在 async 中，當下的 thread 不會 block，把 future 放入 task queue，注意的是他是跑在 worker thread 上面
* `spawn_blocking`：當我們要跑的是 CPU-heavy 的程式碼，那就不適合用 async，spawn_blocking 會把這段程式放在 blocking thread 上面執行，而不會影響 async runtime。
* `await`：用在 async 中，當下的 thread 會 block，然後把 future 放入 task queue，並執行其他的 async task
* `select`：當有我們同時跑多個 future，且希望哪個先完成就做對應的處理，就可以用 select。要記住他必須是 cancellation safe。

## syntax

* `await`：其實是 Rust 提供的特別語法，可以看成下面的程式碼轉換

```rust
some_future.await;
// 等同於
loop {
    match some_future.poll() {
        Pending => yield,  // some_future 還沒完成，所以把權限讓出給其他人執行
        Ready(x) => break,
    }
}
```

* `async fn` 是 syntax sugar，等同於下面的程式碼

```rust
async fn foo() -> String {
    //...
}
// 等同於
fn foo() -> impl Future<Output=()> {
    async {
        //...
    }
}
```

* 如果是 tokio，main 也可以有 syntax sugar

```rust
use tokio;

#[tokio::main]
async fn main() {}
// 等同於
fn main(){
  tokio::runtime::Builder::new_multi_thread()
        .worker_threads(N)  
        .enable_all()
        .build()
        .unwrap()
        .block_on(async { ... });
}
```

## runtime

[三種 runtime](https://magiclen.org/rust-async-await/)

* tokio
* smol
* async-std：已經不再維護

## Pin & Unpin

參考資料

* [Rust语言圣经 - 定海神针 Pin 和 Unpin](https://course.rs/advance/async/pin-unpin.html)
* [Rust 的 Pin 与 Unpin](https://folyd.com/blog/rust-pin-unpin/): 這個講的比較清楚
