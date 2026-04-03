---
title: Rust async
description: Rust async 概念說明
keywords:
  - Rust
---

為什麼會需要 async 呢？
最主要的是因為程式的運作有時候會需要等待 I/O 的回應，等待的過程中如果沒有適當的機制 CPU 將會閒置。
一般我們會想說開 thread，但是當這個 I/O 數量很大的時候（例如同時有成千上百的網路連線），開 thread 的 overhead 就會很大。
這時候就是 async 可以派上用場的地方。

async 本質上就是狀態機（FSM）和 executor 的一種組合。
相對於 thread 是使用 OS 來切換任務並排程，async 則是由程式自行決定什麼時候讓出執行權。
async 中一樣有很多任務，這些任務會交由 executor 來執行。
當某個任務要去存取需要等待比較久的 I/O 時，可以主動交還執行權給 executor，讓 executor 安排其他的任務來運行。
一旦 I/O 部份完成，就會通知 executor 可以把該任務重新放入 queue 中來等待運行權。
這樣的好處是，少數幾個 thread 就可以負責執行大量任務，
透過在 user space 進行任務切換，避免頻繁建立 thread 和 OS 層級的 context switch 所帶來的 overhead。

在 async 中，最基本的單位是 future。
future 是「尚未完成的計算」，本質上是一個狀態機，可以被反覆詢問是否已完成。
future 不會自動執行，需要由 executor 去 poll 才會回答現在是可以運行 (Ready)，還是還在等待 I/O (Pending)。
當一個 future 被交由 executor 管理與排程時，通常會稱之為一個 task。

比較一下 thread 和 async：

* async 的優點：
    * 可以用少量 thread 處理大量 I/O 任務
    * context switch 發生在 user space，成本較低
* async 的缺點：
    * 需要 cooperative scheduling，future 內如果沒有主動讓出執行權限可能會造成 starvation
    * 除錯與理解難度較高
    * 編譯後的狀態機可能較複雜（但通常不是主要瓶頸）

需要注意的是，async 並不等於多執行緒，
它解決的是 concurrency（並發），而不是 parallelism（平行）。
兩者要處理的問題是不一樣的。

一般來說，CPU-bound 的工作適合使用 thread 或 parallelism（例如 thread pool），
而 I/O-bound 的工作則更適合使用 async，以提升資源利用率。

## syntax

* `await`：Rust 提供的特別語法，讓當前 future 暫停執行並把執行權交給 executor，等被 await 的 future 完成後才會從該處繼續往下執行

```rust
some_future.await;
// 等同於
loop {
    match some_future.poll(cx) {
        Poll::Pending => {
            // 註冊 waker，讓 future 完成時可以喚醒
            return Poll::Pending;
        }
        Poll::Ready(x) => break x,
    }
}
```

* `async fn` 是 syntax sugar，等同於下面的程式碼，會回傳 future，可以被 await

```rust
async fn foo() -> String {
    //...
}
// 等同於
fn foo() -> impl Future<Output=String> {
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

## 常用 function

* `block_on`：只能用在非 async 環境，用來啟動 executor 並執行一個 future，呼叫的 thread 會被阻塞，直到 future 完成
* `spawn`：用在 async 中，把 future 放入 task queue，交給 executor 排程，呼叫的 task 不會被阻塞，注意的是他是跑在 worker thread 上面
* `spawn_blocking`：當我們要跑的是 CPU-heavy 的程式碼，那就不適合用 async，spawn_blocking 會把這段程式放在 blocking thread 上面執行，而不會影響 async runtime。
* `await`：用在 async 中，當下的 task 會暫停執行，然後把該 future 放入 task queue，並執行其他的 async task
* `select`：同時等待多個 future，哪個先完成就處理哪個。需要注意 future 必須是 cancellation safe，因為未完成的 future 可能會被中途 drop。

## runtime

前面我們提到 executor，但實際上 executor 其實是被包在 runtime 中。

runtime = executor + I/O driver + timer + thread pool

其中 executor 負責排程 task，
而 I/O driver（例如 epoll / io_uring）則負責在 I/O 完成時喚醒對應的任務。

Rust 中有些常見的 runtime

* tokio：最熱門的 runtime，社群最大，功能也最多，甚至還有專門的 TokioConf
* smol：輕量版本的 runtime
* monoio：能夠 leverage Linux io_uring 的高效能
* async-std：API 設計類似 Rust std library，但已經不再維護

不同 runtime 的差異，不只是 API，
而是底層的 thread 模型與 I/O 機制設計，
這會直接影響效能、延遲以及系統的可擴展性。

## Pin & Unpin

Pin 是用來保證資料在記憶體不會被移動的機制，通常是用在 self-referential 的資料結構，避免因為資料移動造成資料結構出現問題。

Unpin 是 Rust 中的一個 trait，代表該型別可以自由被移動，大多數的型別都支援，除了 self-referential struct 和 future。

```rust
// 創一個有 pointer 的資料結構
struct SelfRef {
    data: String,
    ptr: *const String,
}
// 把 ptr 指向自己的 data
let mut x = SelfRef {
    data: "hello".to_string(),
    ptr: std::ptr::null(),
};
x.ptr = &x.data;

// 把 x 移動，這時候 x.ptr 就會指向錯誤的地方
let y = x;

// 為了避免被 move，我們需要用 Unpin
use std::pin::Pin;
let p = Pin::new(&mut x);
```

### 和 async 的關係

async 由於要儲存當前 future 的所有變數狀態，而這些變數可能有 reference

```rust
// future
async fn foo() {
    let s = String::from("hello");
    let r = &s;
    do_something(r).await;
}
// 編譯後的示意
struct FooFuture {
    state: ...,
    s: String,
    r: *const String, // 指向 s
}
```

如果 future 可以 move，r 就會出錯，所以 executor 在 poll 的時候會強制只能用 Pin，讓 future 不能被 move

```rust
trait Future {
    fn poll(self: Pin<&mut Self>, ...) -> Poll<T>;
}
```

### 參考資料

* [Rust语言圣经 - 定海神针 Pin 和 Unpin](https://course.rs/advance/async/pin-unpin.html)
* [Rust 的 Pin 与 Unpin](https://folyd.com/blog/rust-pin-unpin/): 這個講的比較清楚
