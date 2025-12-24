---
title: Zenoh debug
description: 介紹 debug Zenoh 的一些技巧
keywords:
  - Zenoh
---

這邊介紹一些 debug Zenoh 的小技巧

## Log

首先是怎麼印出 log，因為 Zenoh 是用 Rust 寫的，所以我們可以輕易使用 RUST_LOG 來顯示不同 level 的 log。

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
