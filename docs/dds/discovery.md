---
title: DDS Discovery 機制
description: 介紹 DDS 的 entity 是如何做到彼此發現的
keywords:
  - DDS
---

DDS 是去中心化的系統，和其他協定最大的不同在於它能夠做到彼此發現，而不需要有一個中央的伺服器來事先註冊。
當節點加入網路以後，會透過 discovery 來找到其他的節點 (participant)，然後再建立實際的資料流。

我們可以把 discovery 分成兩個部份：

* `SPDP`：用來尋找網路上有哪些 participant
* `SEDP`：用來發現 participant 底下有哪些 endpoint (reader / writer)

## SPDP

SPDP 全名是 `Simple Participant Discovery Protocol`，目標是讓同個 DDS domain 的 Participant 彼此知道對方存在。

SPDP 本身也是利用 RTPS 的 Reader / Writer 機制來完成。
每個 participant 會有兩個 built-in endpoints。

* `SPDPbuiltinParticipantWriter`：用來對外宣告自己存在，entity ID 是 `ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER`
* `SPDPbuiltinParticipantReader`：用來接收別人的宣告，entity ID 是 `ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER`

兩者會透過 `DCPSParticipant` 這個固定的 topic 名稱來交換資訊，其 type 為 `ParticipantBuiltinTopicData`。

這個宣告的封包是透過 multicast 來發送，因此不需要有對方的 IP。
封包內部包含如下資訊

* Participant GUID / GUID Prefix
* Domain ID 相關資訊
* Vendor ID：哪些的 DDS
* Protocol version
* 可用的 locator，例如 IP address / UDP port
* 內建 endpoint 資訊
* lease duration：如果一段時間都沒收到更新，就會認為這個 participant 已經離線，並移除相關資訊。

大概的流程如下圖：

```raw
Participant A                              Participant B
     │                                           │
     │  SPDP announcement: "A is here"           │
     ├──────────────────────────────────────────>│
     │                                           │
     │  SPDP announcement: "B is here"           │
     │<──────────────────────────────────────────┤
     │                                           │
     │       Participant discovery complete      │
     │                                           │
     │  接著才開始 SEDP endpoint discovery         │
```

## SEDP

SEDP 全名是 `Simple Endpoint Discovery Protocol`，讓 participant 彼此可以交換底下有哪些 endpoint。
endpoint 是收送資料的通訊端點，也就是 data reader 和 data writer。

SEDP 本身也是利用 built-in Reader / Writer，有分成下面三類

* SEDPbuiltinPublicationsWriter / SEDPbuiltinPublicationsReader
    * 用來交換彼此 DataWriter 的資訊
    * 訂閱的 topic name 是 `DCPSPublication`
* SEDPbuiltinSubscriptionsWriter / SEDPbuiltinSubscriptionsReader
    * 用來交換彼此 DataReader 的資訊
    * 訂閱的 topic name 是 `DCPSSubscription`
* SEDPbuiltinTopicsWriter / SEDPbuiltinTopicsReader
    * 用來交換彼此 Topic 的資訊，這並非強制，並不是所有實作都會有
    * 訂閱的 topic name 是 `DCPSTopic`

封包的資訊大概會有

* DataWriter / DataReader 的 GUID
* Topic name
* Type name
* QoS 相關資訊
* Locator，也就是這個 endpoint 的資料要往哪裡送 / 從哪裡收
* Endpoint kind，例如 writer 或 reader

假設 A 有個 writer，B 有個 reader，雙方用 /chatter 這個 topic 溝通，SEDP 的流程大概如下：

```raw
Participant A                                    Participant B

  Publication Writer ── A 的 DataWriter 資訊 ──> Publication Reader
  Subscription Reader <─ B 的 DataReader 資訊 ── Subscription Writer
  A:/chatter DataWriter  <──── matched ────>  B:/chatter DataReader
  A:/chatter DataWriter  ───── data ───────>  B:/chatter DataReader
```

Writer 和 Reader 得知對方的資訊以後，就就會在本地比較對方 topic / type / QoS 和自己的有沒有 match，有了才會進行進一步通訊。
