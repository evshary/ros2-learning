---
title: Eclipse uProtocol
description: 介紹 Eclipse uProtocol 的功能
keywords:
  - SDV
---

Eclipse uProtocol 最早是由 GM (General Motors) 所提出，並且貢獻給　Eclipse Foundation，期望可以兼容多種不同協定，例如 MQTT、SOME/IP、Binder 等等，最終成為車用協定的標準。
不過可惜的是在 2024 年 GM 大裁員後，他們已經放棄經營這塊，目前是由社群自主經營為主。

## 架構

uProtocol 的目標是提供一致的 API 讓上層軟體呼叫，他們不用在乎底下真實實際協定怎麼運作。
如果要比喻的話，uProtocol 之於車用，就像是 ROS 之於機器人一樣。
最終目標是所有開發者都能遵循一樣的 API，讓上層軟體可以重複被使用，而不用換個平台就要重新開發一次。

目前 uProtocol 支援多種程式語言，不過 Rust 是目前功能最成熟的。
不同程式語言有 up-xxx 的實作框架，例如 up-rust、up-cpp 等等，
而底層的實際傳輸的協定就可以用這個框架來指定不同 API 行為應該如何，例如 up-transport-zenoh-rust 等等。

支援的程式語言

* [up-rust](https://github.com/eclipse-uprotocol/up-rust)：Rust 實作
* [up-python](https://github.com/eclipse-uprotocol/up-python)：Python 實作
* [up-java](https://github.com/eclipse-uprotocol/up-java)：Java 實作
* [up-cpp](https://github.com/eclipse-uprotocol/up-cpp)：C++ 實作

支援的協定有

* [up-transport-zenoh-rust](https://github.com/eclipse-uprotocol/up-transport-zenoh-rust)：用 Rust 實作的 Zenoh 傳輸
* [up-transport-mqtt5-rust](https://github.com/eclipse-uprotocol/up-transport-mqtt5-rust)：用 Rust 實作的 MQTT 傳輸
* [up-transport-iceoryx2-rust](https://github.com/eclipse-uprotocol/up-transport-iceoryx2-rust)：用 Rust 實作的 iceoryx 傳輸
* [up-transport-vsomeip-rust](https://github.com/eclipse-uprotocol/up-transport-vsomeip-rust)：用 Rust 實作的 SOME/IP 傳輸

不同 protocol 之間如果需要彼此溝通轉換，就會需要使用 [up-streamer-rust](https://github.com/eclipse-uprotocol/up-streamer-rust) 來連接，目前支援 Zenoh、MQTT、SOME/IP 之間的轉換。

## 常用連結

* [uProtocol 官網](https://uprotocol.org/)：介紹為何需要有 uProtocol
* [uProtocol GitHub](https://github.com/eclipse-uprotocol)：放置 uProtocol 程式碼的地方
* [up-spec](https://github.com/eclipse-uprotocol/up-spec)：uProtocol 的規格
* [slack channel](https://sdvworkinggroup.slack.com/archives/C0698TFGVBN)：在 Eclipse SDV slack 底下有個 uprotocol channel，有任何問題可以在這邊進行討論
