---
title: Zenoh 共享記憶體
description: Zenoh 共享記憶體(shared memory)介紹以及如何使用
keywords:
  - Zenoh
---

## 運作原理

TODO

## 使用方式

### Examples

如果要嘗試 Shared Memory 的範例，我們需要確保 shared_memory 這個 features 有被啟用。
因此編譯的時候需要加上 `--all-features`。

```bash
cargo build --all-features --all-targets
```

我們能夠執行 shared memory 的範例來觀察

```bash
# 第一個 terminal
./target/debug/examples/z_pub_shm
# 第二個 terminal
./target/debug/examples/z_sub_shm
```

我們可以分別在同個機器(使用 shared memory 通訊)和不同機器(使用網路通訊)分別執行 `z_sub_shm`。
從下面可以看到收到的 buffer 分別是 `SHM (MUT)` 和 `RAW` 兩種不同的形式。

```bash
# 同個機器，使用 Shared memory 的情況
>> [Subscriber] Received PUT ('demo/example/zenoh-rs-pub': '[   0] Pub from Rust SHM!') [SHM (MUT)]
# 不同機器，沒有使用 Shared memory 的情況
>> [Subscriber] Received PUT ('demo/example/zenoh-rs-pub': '[   0] Pub from Rust SHM!') [RAW]
```

### Coding

TODO

### Docker

如果你是在 docker 中運行 shared memory，由於預設 docker 允許使用 64 MB，用超過的話會有問題。
你可以把 `/dev/shm` 掛入 container (`-v /dev/shm:/dev/shm`) 或者指定一個比較大的 `shm-size` (`--shm-size=32g`) 來避免這個問題。

### 如何判斷有無使用 Shared memory

要如何確認真的是使用 shared memory 呢？
這邊有兩種方法：

1. 如果是使用 Linux 的話，可以觀察 `/dev/shm` 有沒有增加 `*.zenoh` 的檔案。這邊會放置預設 Linux 的 shared memory 空間。
2. 可以用 `export RUST_LOG=z=trace` 觀察，發布訊息時有沒有出現 `ext_shm: Some(ShmType)`，這代表使用 SHM 傳送，不然的話只會出現 `ext_shm: None`

## 設定檔

* `transport_optimization`
    * 除了自行使用 backend 產生 buffer，我們也可以直接用 `transport_optimization` 針對超過一定 size 的訊息自動用 shared memory 來傳輸
    * `pool_size` 代表有多少記憶體可以使用
    * `message_size_threshold` 則是超過多大的訊息才需要改用 shared memory 傳輸

## 參考連結

* [Zenoh Shared Memory RFC 規範](https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/SHM.md)
