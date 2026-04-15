---
title: Zenoh 介紹
description: Zenoh 基礎概念介紹、各種相關資源
keywords:
  - Zenoh
---

Zenoh 是 [ZettaScale](https://www.zettascale.tech/) 所開發出的開源通訊軟體，目前和 CycloneDDS 一樣託管於 [Eclipse Foundation 底下](https://projects.eclipse.org/projects/iot.zenoh)。

Zenoh 和 DDS 類似，是一個通訊層的 middleware，可以幫助開發者更容易的交換資訊。
由於 ZettaScale 內部有許多當初設計 DDS 的研究人員，很清楚 DDS 本身的限制，為了避開這些問題，才特別設計新的協定。

具體 DDS 有哪些問題呢？

* DDS 並不能支援跨網域的傳輸，只能在 LAN
* DDS 強烈依賴 multicast，無法在沒有 multicast 的環境執行，例如 4G 或 5G
* DDS 一開始並非為了無線網路設計，所以不限制流量的話，他的 discovery 的機制會讓無線網路癱瘓
* DDS 實作很複雜，通常無法跑在嵌入式系統上

Zenoh 則是另起爐灶，在保留 DDS 去中心化和高效能的同時，也結合了中心化通訊的機制。

## 有用連結

Zenoh 的 [GitHub namepace](https://github.com/eclipse-zenoh) 裡面底下有很多 project，下面列出幾個常用的

* [zenoh](https://github.com/eclipse-zenoh/zenoh)：zenoh 本身的主程式
* [zenoh-spec](https://spec.zenoh.io/spec/)：Zenoh 的正式協定規格
* [Zenoh Roadmap](https://github.com/eclipse-zenoh/roadmap)：Zenoh 部份的規格定義放在這邊
* Zenoh 各種程式語言 API 的 repository
    * Rust: [zenoh](https://github.com/eclipse-zenoh/zenoh)
        * [基本 Zenoh API](https://docs.rs/zenoh/latest/zenoh/)
        * [進階 Zenoh API](https://docs.rs/zenoh-ext/latest/zenoh_ext/)
    * C: [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c)
    * C++: [zenoh-cpp](https://github.com/eclipse-zenoh/zenoh-cpp)
    * Python: [zenoh-python](https://github.com/eclipse-zenoh/zenoh-python)
    * Kotlin: [zenoh-kotlin](https://github.com/eclipse-zenoh/zenoh-kotlin)
    * Java: [zenoh-java](https://github.com/eclipse-zenoh/zenoh-java)
    * Typescript: [zenoh-ts](https://github.com/eclipse-zenoh/zenoh-ts)
* 嵌入式版本
    * [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico)：用 C 語言寫成，有較低的 footprint
    * [zenoh-nostd](https://github.com/ZettaScaleLabs/zenoh-nostd)：用純 Rust 寫成，且不需要用到標準函式庫，所以可以用在嵌入式系統
* Plugin：Zenoh 支援多種 plugin，可以嵌入在 Zenoh 程式之中，將 Zenoh 的訊息轉成其他的 protocol
    * [zenoh-plugin-dds](https://github.com/eclipse-zenoh/zenoh-plugin-dds)：結合 zenoh 和 DDS，將兩者的資料互相轉換
    * [zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)：雖然也是跟 DDS 相連，但是這特別限定是用在 ROS 2 的情境下
    * [zenoh-plugin-mqtt](https://github.com/eclipse-zenoh/zenoh-plugin-mqtt)：將 MQTT 和 Zenoh 的訊息互相轉換
    * [zenoh-plugin-webserver](https://github.com/eclipse-zenoh/zenoh-plugin-webserver)：可以用 Web URL 來取得 Zenoh 的資料
* Backend：Zenoh 也支援多種 backend，可以將收到的訊息儲存在特定 Database 之中
    * [zenoh-backend-influxdb](https://github.com/eclipse-zenoh/zenoh-backend-influxdb)：將資料儲存在 InfluxDB 中，支援 V1 和 V2
    * [zenoh-backend-filesystem](https://github.com/eclipse-zenoh/zenoh-backend-filesystem)：將資料儲存在檔案系統之中
    * [zenoh-backend-rocksdb](https://github.com/eclipse-zenoh/zenoh-backend-rocksdb)：將資料儲存在 RocksDB 之中
    * [zenoh-backend-s3](https://github.com/eclipse-zenoh/zenoh-backend-s3)：將資料儲存在 S3 資料庫之中
* [zenoh-dissector](https://github.com/eclipse-zenoh/zenoh-dissector)：Zenoh 的 wireshark plugin，可以用來解析 Zenoh 的封包

如果你想了解更多使用 Zenoh 的 project，可以參考 [awesome-zenoh](https://github.com/kydos/awesome-zenoh)。
