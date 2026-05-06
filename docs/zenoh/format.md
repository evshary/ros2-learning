---
title: Zenoh 封包格式
description: 介紹 Zenoh 的實際封包格式
keywords:
  - Zenoh
---

## VLE

在 Zenoh 中，常常會使用 VLE (Variable-Length Encoding) 的方式來表達數值。
這個與一般表達方式不同的方式在於，他的長度會是動態的。
當數值小的時候可以用一個 byte 表示，但是數值大的時候可能會比一般表達方式大一個 byte。
然而這並不是太大的問題，因為大多數情況數值都不會太大。
VLE 的原理其實就只是保留一個 byte 中的第一個 bit 當 continuous bit，當後面七個 bit 用完的時候，就會把第一個 bit 拉成 1，然後往下長下一個 byte。
所以 127 的表達方式是 `0x7F`，然後 128 就會變成 `0x80 0x01` 了。
詳細的邏輯可以參考 [spec](https://spec.zenoh.io/spec/1.0.0/wire/primitives.html#vle) 的說明。

## 封包格式

讓我們實際看看 Zenoh 實際的封包格式，這邊一樣要分成 Scouting 和 Transport 的部份

### Scout & Hello 封包

Scouting 的部份有 Scout 和 Hello 兩種封包

<!-- markdownlint-disable MD046 -->
??? "SCOUT 封包"

    * 第一個 byte 有 ID 和 flag，ID 固定是 0x01 代表 Scout message，flag 則是用來表示是否有 extension
    * 第二個 byte 是版本，目前為 0x09
    * 第三個 byte 有兩個功能，如果 I=1 代表有 ZenohID，WHAT 則是用來表示自身角色 (Router, Peer, Client)
    * 第四個 byte 之後就是看前面的 flag 有沒有要加上 ZenohID 和 extension

    ```text
    Flags:
      Z  If Z==1, extension chain follows.
    
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X|  SCOUT  |   ID = 0x01
    +-+-+-+---------+
    |    version    |   Protocol version (u8); current value: 0x09
    +---------------+
    |zid_len|I|WHAT |   Packed byte (see below)
    +-+-+-+-+-+-+-+-+
    ~      ZID      ~   if I==1: ZenohID (1 + zid_len bytes)
    +---------------+
    ~  [ScoutExts]  ~   if Z==1: extension chain
    +---------------+
    ```
<!-- markdownlint-enable MD046 -->

<!-- markdownlint-disable MD046 -->
??? "HELLO 封包"

    * 第一個 byte 一樣是 ID 和 flag，ID 固定是 0x02，flag 則是表達有沒有可連接清單和 extension
    * 第二個 byte 一樣是版本，0x09
    * 第三個 byte 則是 ZenohID 長度和當前的角色 (Router, Peer, Client)
    * 後面的部份就是實際的 ZenohID、連接清單、extension

    ```text
    Flags:
      L  If L==1, explicit locator list is present.
         If L==0, the source address of the packet is the locator.
      Z  If Z==1, extension chain follows.

     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|L|  HELLO  |   ID = 0x02
    +-+-+-+---------+
    |    version    |   Protocol version (u8); current value: 0x09
    +---------------+
    |zid_len|X|X|WAI|   Packed byte (see below)
    +-+-+-+-+-+-+-+-+
    ~      ZID      ~   ZenohID (1 + zid_len bytes)
    +---------------+
    % loc_count: z8 %   if L==1: number of locator strings
    ~   locators    ~   if L==1: loc_count × <utf8;z8> locator strings
    +---------------+
    ~  [HelloExts]  ~   if Z==1: extension chain
    +---------------+
    ```
<!-- markdownlint-enable MD046 -->

### Transport 封包

Transport Messages 有多個，例如 SYN、OPEN、FRAME、KEEPALIVE、CLOSE 等等，其中最重要的 FRAME。

FRAME 底下還可以切分成更細的 Network Messages，例如 OAM、RESPONSE、REQUEST、DECLARE、PUSH、INTEREST 等等。

而最常用的 Network Messages，PUSH、REQUEST、RESPONSE 底下還可以有 Data Sub-Messages，如 PUT、DEL、QUERY、REPLY、ERR 等等。

#### Transport Messages

<details>
  <summary>INIT 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|S|A|   INIT  |   ID = 0x01
    +-+-+-+---------+
    |    version    |   Protocol version (u8); current = 0x09
    +---------------+
    |zid_len|X|X|WAI|   Packed byte: bits 7:4 = zid_len, bits 1:0 = WhatAmI
    +-+-+-+-+-+-+-+-+
    ~      ZID      ~   (1 + zid_len) bytes
    +---------------+
    |X|X|X|X|RID|FSN|   if S==1: Resolution byte
    +---------------+
    |   batch_lo    |   if S==1: Batch size (u16 LE)
    +---------------+
    |   batch_hi    |
    +---------------+
    ~  <u8;z16>    ~   Cookie — InitAck (A==1) ONLY
    +---------------+
    ~  [InitExts]   ~   if Z==1
    +---------------+
    ```

</details>

<details>
  <summary>OPEN 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|T|A|   OPEN  |   ID = 0x02
    +-+-+-+---------+
    %    lease      %   VLE-encoded lease duration (unit per T flag)
    +---------------+
    % initial_sn    %   VLE-encoded initial sequence number
    +---------------+
    ~  <u8;z16>    ~   Cookie (OpenSyn, A==0 only) — MUST match cookie from InitAck
    +---------------+
    ~  [OpenExts]   ~   if Z==1
    +---------------+
    ```

</details>

<details>
  <summary>CLOSE 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|S|  CLOSE  |   ID = 0x03
    +-+-+-+---------+
    |    reason     |   Close reason code (u8); see xref:session:close.adoc[]
    +---------------+
    ~  [CloseExts]  ~   if Z==1
    +---------------+
    ```

</details>

<details>
  <summary>KEEPALIVE 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X| KALIVE  |   ID = 0x04
    +-+-+-+---------+
    ~  [KAliveExts] ~   if Z==1
    +---------------+
    ```

</details>

<!-- markdownlint-disable MD046 -->
??? "FRAME 封包"

    * 第一個 byte 是 id (0x05 代表 frame) 和 flag
        * r：是否 reliable
        * Z：是否用到 extensions
    * 第二個 byte 是 squence number
        * 使用 Variable-length integer(VarInt)，長度會隨著 seq 增長而增加，如果 0-127 只需要一個 byte，更大的話用到 2-4 bytes 都可以
    * 第三個部份則是可擴充的欄位，是 TLV 格式
        * 如果 Header 有設定 flag 這邊才會出現，例如 QoS、timestamp 都是在這邊
    * 最後的部份則是會放入多個 Network Messages

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|R|  FRAME  |   ID = 0x05
    +-+-+-+---------+
    %    seq_num    %   VLE-encoded sequence number (resolution-dependent)
    +---------------+
    ~  [FrameExts]  ~   if Z==1
    +---------------+
    ~  [NetworkMsg] ~   one or more back-to-back serialised NetworkMessages (*)
    +---------------+
    ```
<!-- markdownlint-enable MD046 -->

<details>
  <summary>FRAGMENT 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|R| FRAGMENT|   ID = 0x06
    +-+-+-+---------+
    %    seq_num    %   VLE-encoded sequence number (same space as FRAME on this channel)
    +---------------+
    ~  [FragExts]   ~   if Z==1
    +---------------+
    ~  [fragment]   ~   raw fragment bytes (remainder of the batch)
    +---------------+
    ```

</details>

#### Network Messages

<details>
  <summary>OAM 封包</summary>

    Orchestration, Administration, Maintainence (OAM) 相關的封包

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|ENC|  OAM    |   ID = 0x1F
    +-+-+-+---------+
    %    id : z16   %   OAM identifier
    +---------------+
    ~  [oam_exts]   ~   if Z==1
    +---------------+
    ~     body      ~   body encoding selected by ENC
    +---------------+
    ```

</details>

<details>
  <summary>PUSH 封包</summary>

    用在 Publish 資料上面

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|N|  PUSH   |   ID = 0x1D
    +-+-+-+---------+
    % key_scope:z16 %   VLE ExprId (0 = global scope)
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16> suffix string
    +---------------+
    ~  [push_exts]  ~   if Z==1 (QoS, Timestamp, NodeId)
    +---------------+
    ~   PushBody    ~   PUT (0x01) or DEL (0x02) sub-message
    +---------------+
    ```

</details>

<details>
  <summary>REQUEST 封包</summary>

    用在發送 Query 資料上

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|N| Request |   ID = 0x1C
    +-+-+-+---------+
    % request_id:z32%   VLE request identifier
    +---------------+
    % key_scope:z16 %   VLE ExprId
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    +---------------+
    ~  [req_exts]   ~   if Z==1 (QoS, Timestamp, NodeId, QueryTarget, Budget, Timeout)
    +---------------+
    ~  RequestBody  ~   QUERY (0x03) sub-message
    +---------------+
    ```

</details>

<details>
  <summary>RESPONSE 封包</summary>

    用在回覆 Query 資訊上面

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|N| Response|   ID = 0x1B
    +-+-+-+---------+
    % request_id:z32%   VLE — correlates to the originating REQUEST
    +---------------+
    % key_scope:z16 %
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    +---------------+
    ~  [reply_exts] ~   if Z==1 (QoS, Timestamp, ResponderId)
    +---------------+
    ~  ResponseBody ~   REPLY (0x04) or ERR (0x05) sub-message
    +---------------+
    ```

</details>

<details>
  <summary>RESPONSE_FINAL 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X| ResFinal|   ID = 0x1A
    +-+-+-+---------+
    % request_id:z32%   VLE — must match the originating REQUEST
    +---------------+
    ~  [rf_exts]    ~   if Z==1 (QoS, Timestamp)
    +---------------+
    ```

</details>

<details>
  <summary>DECLARE 封包</summary>

    DECLARE 有幾個用途，把 key expression 對應到 ID 減少傳輸量、宣告 subscriber、queryables、liveliness token。
    值得注意的是 publish 或 query 並不需要 declare，這可以減少協定的複雜度。

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|I| DECLARE |   ID = 0x1E
    +-+-+-+---------+
    %interest_id:z32%   if I==1: VLE interest ID this declare responds to
    +---------------+
    ~  [decl_exts]  ~   if Z==1 (QoS, Timestamp, NodeId)
    +---------------+
    ~  declaration  ~   DeclareBody sub-message (see xref:session:declarations.adoc[])
    +---------------+
    ```

</details>

<details>
  <summary>INTEREST 封包</summary>

    比較晚加入的節點如果要得知當前網路上的狀態，就必須送出 INTEREST 來得到之前已經送出過的 DECLARE。

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|Mod| INTEREST|   ID = 0x19; Mod in bits 6:5
    +-+-+-+---------+
    %    id : z32   %   VLE — Interest identifier
    +---------------+
    |A|M|N|R|T|Q|S|K|   Options byte (present only when Mod != Final=0b00)
    +---------------+
    % key_scope:z16 %   if Mod!=Final && R==1: VLE ExprId
    +---------------+
    ~  key_suffix   ~   if Mod!=Final && R==1 && N==1: <u8;z16>
    +---------------+
    ~  [int_exts]   ~   if Z==1 (QoS, Timestamp, NodeId)
    +---------------+
    ```

</details>

#### Data Sub-Messages

以下是 Network Messages 會對應到的 Data Sub-Messages

* PUSH: PUSH, DEL
* REQUEST: QUERY
* RESPONSE: REPLY, ERR

<details>
  <summary>PUT 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|E|T|   PUT   |   ID = 0x01
    +-+-+-+---------+
    ~  ts: <u8;z16> ~   if T==1: Timestamp
    +---------------+
    ~   encoding    ~   if E==1: Encoding field (z32 with S-bit + optional schema)
    +---------------+
    ~  [put_exts]   ~   if Z==1 (SourceInfo, Shm, Attachment)
    +---------------+
    ~ pl: <u8;z32>  ~   Payload bytes
    +---------------+
    ```

</details>

<details>
  <summary>DEL 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|T|   DEL   |   ID = 0x02
    +-+-+-+---------+
    ~  ts: <u8;z16> ~   if T==1: Timestamp
    +---------------+
    ~  [del_exts]   ~   if Z==1 (SourceInfo, Attachment)
    +---------------+
    ```

</details>

<details>
  <summary>QUERY 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|P|C|  QUERY  |   ID = 0x03
    +-+-+-+---------+
    |consolidation  |   if C==1: ConsolidationMode (u8)
    +---------------+
    ~ ps: <u8;z16>  ~   if P==1: query parameters (UTF-8)
    +---------------+
    ~  [qry_exts]   ~   if Z==1 (SourceInfo, QueryBody, Attachment)
    +---------------+
    ```

</details>

<details>
  <summary>REPLY 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|C|  REPLY  |   ID = 0x04
    +-+-+-+---------+
    |consolidation  |   if C==1: ConsolidationMode (u8)
    +---------------+
    ~  [repl_exts]  ~   if Z==1
    +---------------+
    ~   ReplyBody   ~   PushBody: PUT (0x01) or DEL (0x02)
    +---------------+
    ```

</details>

<details>
  <summary>ERR 封包</summary>

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|E|X|   ERR   |   ID = 0x05
    +-+-+-+---------+
    ~   encoding    ~   if E==1: Encoding field
    +---------------+
    ~  [err_exts]   ~   if Z==1 (SourceInfo, Shm)
    +---------------+
    ~ pl: <u8;z32>  ~   Error payload bytes
    +---------------+
    ```

</details>
