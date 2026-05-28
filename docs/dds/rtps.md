---
title: DDS RTPS 封包格式
description: 介紹 RTPS 與 wire protocol 的基本概念
keywords:
  - DDS
---

`RTPS` 全名是 `Real-Time Publish-Subscribe`，被規範在 OMG 標準中，主要是提供給 DDS 生態系所使用
通常我們都會稱之為 DDSI-RTPS，也就是 DDS 的 wire protocol，包含如下資訊：

* message header 長什麼樣子
* submessage 有哪些種類
* discovery 與 user data 要怎麼封裝和傳送

如果把 DDS 看成概念與 API 層，RTPS 就更接近網路上的實際封包格式。

## RTPS Message 與 Submessage

RTPS 的封包通常可以拆成：

```raw
RTPS Message
├── Header (包含 protocol version、vendor id、GUID prefix)
└── Submessage 1
└── Submessage 2
└── Submessage 3
```

Submessage 是實際的資料或控制單元，有下面幾個種類（並非全部）：

* DATA：writer 傳資料給 reader
    * 裡面通常包括 readerId、writerId、writerSeqNum、serializedPayload、inlineQoS(optional) 等等
* DATA_FRAG：如果 DATA 太大，會切成多個 DATA_FRAG
* HEARTBEAT：宣告 writer 目前有哪些 sequence numbers
    * 如果 writer 的 HEARTBEAT 紀錄 firstSN=10, lastSN=15，代表已經傳了 10 到 15 的資料，reader 可以自行判斷有無缺資料
* ACKNACK：reader 回報哪些 sequence numbers 收到了 / 缺了
* GAP：告訴 reader 某些 sequence numbers 不會再送
* INFO_TS：設定後續 submessage 的 timestamp
* INFO_DST：指定後續 submessage 的目的 participant
    * 它常用於 unicast 或定向傳送場景，讓接收端判斷這個 message 是否是給自己的
* INFO_SRC：指定後續 submessage 的來源 participant

一個 RTPS Message 可以包多個 Submessage，而且 Submessage 會依序被解讀。
所以像是 `INFO_TS`、`INFO_DST`或`INFO_SRC` 類似於上下文設定，會影響後續的 message 解讀。

## 常見傳輸

* Best Effort

```raw
Writer ── DATA(seq=1) ──> Reader
Writer ── DATA(seq=2) ──> Reader
Writer ── DATA(seq=3) ──> Reader
```

* Reliable

```raw
Writer ── DATA(seq=1) ─────> Reader
Writer ── DATA(seq=2) ─X    Reader
Writer ── DATA(seq=3) ─────> Reader

Writer ── HEARTBEAT(1..3) ─> Reader
Reader ── ACKNACK(missing 2) -> Writer
Writer ── DATA(seq=2) ─────> Reader
```

* 完整從 SPDP、SEDP 到 Data 的流程
    * Reader ID 有可能為 unknown，代表有興趣的 reader 自己接收，通常用在傳送給多個節點的情況

```raw
Participant A                                  Participant B
==============                                 ==============

[1] SPDP: A announces itself

SPDPbuiltinParticipantWriter
  DATA {
    writerId = SPDP_PARTICIPANT_WRITER
    readerId = SPDP_PARTICIPANT_READER
    payload  = ParticipantBuiltinTopicData(A)
  }
──────────────────────────────────────────────>
                                      SPDPbuiltinParticipantReader


[2] SPDP: B announces itself

SPDPbuiltinParticipantReader
<──────────────────────────────────────────────
                                      SPDPbuiltinParticipantWriter
                                      DATA {
                                        writerId = SPDP_PARTICIPANT_WRITER
                                        readerId = SPDP_PARTICIPANT_READER
                                        payload  = ParticipantBuiltinTopicData(B)
                                      }


[3] SEDP: A announces its DataWriter

SEDPbuiltinPublicationsWriter
  DATA {
    writerId = SEDP_PUBLICATIONS_WRITER
    readerId = SEDP_PUBLICATIONS_READER
    payload  = PublicationBuiltinTopicData(A:/chatter writer)
  }
──────────────────────────────────────────────>
                                      SEDPbuiltinPublicationsReader


[4] SEDP: B announces its DataReader

SEDPbuiltinSubscriptionsReader
<──────────────────────────────────────────────
                                      SEDPbuiltinSubscriptionsWriter
                                      DATA {
                                        writerId = SEDP_SUBSCRIPTIONS_WRITER
                                        readerId = SEDP_SUBSCRIPTIONS_READER
                                        payload  = SubscriptionBuiltinTopicData(B:/chatter reader)
                                      }


[5] Local matching on both sides

A learns:
  B has DataReader /chatter

B learns:
  A has DataWriter /chatter

Both compare:
  topic name
  type name
  QoS compatibility
  locators


[6] User data starts

A:/chatter user DataWriter
  DATA {
    writerId = A:/chatter writer entity id
    readerId = B:/chatter reader entity id or unknown
    writerSeqNum = 1
    payload = serialized DDS message
  }
──────────────────────────────────────────────>
                                      B:/chatter user DataReader
```
