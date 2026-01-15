---
title: Zenoh debug
description: 介紹 debug Zenoh 的一些技巧
keywords:
  - Zenoh
---

這邊介紹一些 debug Zenoh 的小技巧

## Example

如果自己寫的 Zenoh 程式無法通訊，可以用官方的 Example 程式來測試。

我們這邊假設是使用 Rust 版本的 Zenoh。

* 測試 publish 和 subscribe 能不能用 multicast 找到對方互通

```bash
# 啟動 publisher
cargo run --example z_pub
# 另外一個 terminal 跑 subscriber
cargo run --example z_sub
```

* 測試能不能直接用 IP 值連

```bash
# 啟動 publisher，並指定要聽 port 7447
cargo run --example z_pub -- --listen tcp/127.0.0.1:7447 --no-multicast-scouting
# 另外一個 terminal 跑 subscriber，並指定要連到 port 7447
cargo run --example z_sub -- --connect tcp/127.0.0.1:7447 --no-multicast-scouting
```

## Log

因為 Zenoh 是用 Rust 寫的，所以我們可以輕易使用 RUST_LOG 來顯示不同 level 的 log。

```bash
# 顯示基本的 debug 資訊
RUST_LOG=zenoh=debug zenohd
# 顯示更細的訊息，但是這會嚴重影響效能
RUST_LOG=zenoh=trace zenohd
```

當然大部分情況我們可能只想觀察部份 log 而已，所以可以善用 filter 機制

```bash
# 和建立連線相關
RUST_LOG=zenoh_transport::unicast::establishment=debug zenohd
```
