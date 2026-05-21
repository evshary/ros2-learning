---
title: DDS QoS 的概念
description: 介紹 DDS 中各種 QoS 的使用
keywords:
  - DDS
---

QoS (Quality of Service) 是 DDS 最重要也最容易讓人混亂的一部份。
在 DDS 規格中有多達 20 多種 QoS，功能強大，但是非常複雜。
它不只是「本地端的傳輸偏好設定」，還會直接影響 writer 和 reader 能不能成功配對。

## 為什麼 DDS 需要 QoS

可能會有人不懂，為何會需要 QoS 的設定，但其實不同系統對資料的要求差很多。
有些情境比較在意可靠性，例如我們要送一些指令或是接收緊急事件的時候。
有些情境更在意低延遲，就像是要傳輸影像給對方的時候。
還有些則希望晚加入的節點也能拿到之前已經發佈的資料。
這些總總要求，不可能只用一種傳輸方式來解決，需要更多細微設定，也就是 QoS 存在的意義。

## QoS 不只影響傳輸，也影響配對

一個很重要的觀念是：Discovery 成功，不代表資料一定會通。
writer 和 reader 要能通訊，除了前面提過 name 和 type 要一致外，還需要 QoS 能相容。
能否相容的關鍵在於 RxO (Request vs Offered)，會影響相容性的 QoS 都會註明 RxO 為何。
一般來說 writer 的 QoS 如果比 reader 的 QoS 還嚴格，通常是可以相通，反過來則不行。
例如 Reliable 比 Best Effort 還要嚴苛，所以 writer 是 Reliable 可以跟任意 reader 通訊。

有哪些 QoS 有 RxO 的限制，可以參考 [RTI](https://community.rti.com/static/documentation/connext-dds/current/doc/manuals/connext_dds_professional/qos_reference/qos_reference/BasicQoS.htm) 這邊的文件

## QoS 可以掛在不同實體 (entity)

QoS 可以被掛在 participant、topic、writer、reader 等等實體上面。

* DomainParticipant：管理全局資源的 QoS，如 Resource Limit，也是其他實體預設的 QoS。
* Topic：定義某些 Topic 預設要用的 QoS。
* Publisher：底下 Writer 的預設 QoS。
* Writer：發送端的 QoS 規則。
* Subscriber：底下 Reader 的預設 QoS。
* Reader：接收端的 QoS 規則。

[RTI](https://community.rti.com/static/documentation/connext-dds/current/doc/manuals/connext_dds_professional/qos_reference/qos_reference/BasicQoS.htm) 這邊的文件也有提到各種 QoS 可以被套用在哪些實體上面。

## 常見 QoS

### Reliability

`Reliability` 用來決定資料是否需要可靠送達。

常見設定：

* `BestEffort`：盡力送，不保證每筆都到
* `Reliable`：需要確認與重傳

Reliable 不一定比較好，雖然可以帶來更高的保證，但也可能帶來更高延遲、更多狀態維護，以及更大的資源消耗。
對某些高頻感測資料來說，`BestEffort` 反而更合適。

### Durability

`Durability` 用來決定晚加入的 reader 能不能拿到過去已經發送的資料。

常見設定：

* `Volatile`：不保留歷史給晚加入者
* `Transient Local`：writer 可保留部分歷史給晚加入者
* `Transient`：reader 也會幫忙保留部份歷史
* `Persistent`：就算是重開機，歷史資料還是會保存

### History

`History` 決定 entity 會保留多少筆樣本。

常見設定：

* `KeepLast`：只保留最近 N 筆
* `KeepAll`：保留所有樣本，直到資源限制介入

### Deadline

`Deadline` 描述的是資料更新週期的期望。
如果超過某段時間沒有收到新資料，系統可以視為 deadline miss。

### Liveliness

`Liveliness` 用來確認 writer 是否還活著。
它不是資料內容本身，而是「這個資料來源是否還存在」的保證機制。

常見設定：

* automatic / manual：底層 DDS 自動確認對方是否還存在，還是要上層手動確認
* lease duration：多久時間判斷對方已經不存在

### Lifespan

`Lifespan` 用來描述資料的有效期限。
如果一筆樣本已經過期，即使晚一點送到 reader，也可能被直接丟棄。

### Ownership

當有多個 writer 同時發佈資料的時候，reader 要優先聽誰的。
通常我們會把這個當成是 backup 來用，例如同時收兩方的指令，但是只聽優先權高的，一旦優先權高的那方掛掉，就改成聽優先權低的。

常見設定：

* shared：預設狀態，reader 會收所有資料
* exclusive：根據優先權來決定要收誰的資料，只會收優先權最大的那個 writer
* strength：誰的優先權比較高

### Transport Priority

根據優先級，在傳輸層排程的時候來決定誰可以優先傳。
然而這並不是保證，只能算是給 DDS middleware 的一種暗示，實際怎麼運作還是根據 DDS vendor 的實作來決定。
不同 DDS 實作可能會調整 thread priority、socket priority、queue priority，或是封或是封包的 traffic class，甚至還搭配 TSN，所以不一定絕對有效。
