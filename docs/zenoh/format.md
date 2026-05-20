---
title: Zenoh 封包格式
description: 介紹 Zenoh 的實際封包格式
keywords:
    - Zenoh
---

## VLE

在 Zenoh 中，常常會使用 VLE (Variable-Length Encoding) 的方式來表達數值。
這個與一般表達方式不同的方式在於，它的長度會是動態的。
當數值小的時候可以用一個 byte 表示，但是數值大的時候可能會比一般表達方式大一個 byte。
然而這並不是太大的問題，因為大多數情況數值都不會太大。
VLE 的原理其實就只是保留一個 byte 中的第一個 bit 當 continuous bit，當後面七個 bit 用完的時候，就會把第一個 bit 拉成 1，然後往下長下一個 byte。
所以 127 的表達方式是 `0x7F`，然後 128 就會變成 `0x80 0x01` 了。
詳細的邏輯可以參考 [spec](https://spec.zenoh.io/spec/1.0.0/wire/primitives.html#vle) 的說明。

## 封包總覽

如果目的是查 wire format，可以先記住 Zenoh 有兩條主要路徑：

- `Scouting path`
    - `SCOUT`
    - `HELLO`
- `Session / data path`
    - 外層可能會有 batch
    - 預設 transport 下，batch 中通常放的是 transport message
    - `FRAME` 裡面才會再放 one or more network messages
    - 某些 network message 的 body 才會再放 sub-message

可以用下面這個封裝關係來理解：

```text
Scouting path
SCOUT
HELLO

Session / data path
Batch
└─ Transport message
   ├─ OAM
   ├─ INIT | OPEN | CLOSE | KEEPALIVE | JOIN
   ├─ FRAME
   │  └─ one or more Network messages
   │     ├─ OAM
   │     ├─ PUSH -> PUT | DEL
   │     ├─ REQUEST -> QUERY
   │     ├─ RESPONSE -> REPLY | ERR
   │     ├─ RESPONSE_FINAL
   │     ├─ DECLARE -> exactly one declaration body
   │     └─ INTEREST
   └─ FRAGMENT -> raw fragment bytes
```

要特別注意幾件事：

- `PUT`、`DEL`、`QUERY`、`REPLY`、`ERR` 都不能獨立存在，只能當作外層 network message 的 body
- `D_KEYEXPR`、`D_SUBSCRIBER` 這類 declaration body 也不能獨立存在，只能放在 `DECLARE` 裡
- `extension` 不是獨立封包型別，而是依附在某個 message 或 body 後面的可擴充欄位
- 預設 transport 下，batch 主要承載 transport message，network message 會被包在 `FRAME` 裡；但 low-latency unicast 模式下，`NetworkMessage`、`KEEPALIVE`、`CLOSE` 也可能直接由 transport 傳送

## 封包格式

### Scout & Hello 封包

Scouting 的部份有 `SCOUT` 和 `HELLO` 兩種封包。

<!-- markdownlint-disable MD046 -->
??? "SCOUT 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/scouting/scout.html

    * 第一個 byte 有 ID 和 flag，ID 固定是 `0x01` 代表 `SCOUT`
    * 第二個 byte 是版本，目前為 `0x09`
    * 第三個 byte 有兩個功能，如果 `I=1` 代表有 ZenohID，`WHAT` 則是用來表示自身角色 (Router, Peer, Client)
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

    Spec: https://spec.zenoh.io/spec/1.0.0/scouting/hello.html

    * 第一個 byte 一樣是 ID 和 flag，ID 固定是 `0x02`
    * `L` flag 表達有沒有顯式 locator list
    * `Z` flag 表達後面有沒有 extension chain
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

### Session / Data Path

#### Batch

在 session / data path 中，最外層可以先理解成 `Batch`。
`Batch` 本身就是一串連續的 message，差別只在不同 link 類型會用不同方式界定 batch 的邊界。

- `stream link`
    - 例如 TCP / TLS / QUIC
    - batch 前面會有一個 `u16le` 的 length prefix
- `datagram link`
    - 例如 UDP unicast / UDP multicast
    - 整個 datagram payload 就是一個 batch，不會再額外加 length prefix

參考：

- Batching: https://spec.zenoh.io/spec/1.0.0/transport/batching.html
- Frame Format: https://spec.zenoh.io/spec/1.0.0/wire/frame-format.html

```text
Batch on stream link
+---------------+
| BatchLength   |   u16 little-endian
+---------------+
| Batch         |   one or more messages
+---------------+

Batch on datagram link
+---------------+
| Batch         |   one or more messages
+---------------+
```

#### Transport Messages

Transport message 是 batch 中真正出現的 message 類型。
在預設 transport 下是這樣，但在 negotiated low-latency unicast 模式下，`NetworkMessage`、`KEEPALIVE`、`CLOSE` 也可以直接出現在 batch 中。
其中 `FRAME` 和 `FRAGMENT` 負責承載或切片，`INIT` / `OPEN` / `CLOSE` / `KEEPALIVE` / `JOIN` 則偏向 session 或 link control。

<!-- markdownlint-disable MD046 -->
??? "Transport OAM 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

    Router 之間用來同步 `link-state` / `router graph` 的 `OAM_LINKSTATE` 就是承載在這個 `Transport OAM` 的 body 裡。

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|ENC|  OAM    |   ID = 0x00
    +-+-+-+---------+
    %    id : z16   %   OAM identifier
    +---------------+
    ~  [oam_exts]   ~   if Z==1
    +---------------+
    ~     body      ~   body encoding selected by ENC
    +---------------+
    ```

    參考：

    - Spec / Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html
    - Source / Router OAM handler: `zenoh/src/net/routing/hat/router/mod.rs`
    - Source / OAM_LINKSTATE message creation: `zenoh/src/net/protocol/network.rs`

??? "Transport OAM: OAM_LINKSTATE body"

    這個 body 是 `ZBuf` 內部的資料格式。
    `zenoh` 原始碼中的 `OAM_LINKSTATE` ID 是 `0x0001`，body 內容是一個 `LinkStateList`。

    ```text
    OAM body
    +----------------------+
    | LinkStateList        |
    +----------------------+

    LinkStateList
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |X|X|X|LK_ST_LS |
    +-+-+-+---------+
    ~ [link_states] ~   one or more LinkState entries
    +---------------+

    LinkState
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |X|X|X|G|H|L|W|P|
    +-+-+-+-+-+-+-+-+
    ~     psid      ~   per-link session-local node id
    +---------------+
    ~      sn       ~   sequence number for this node state
    +---------------+
    ~      zid      ~   if P==1
    +---------------+
    ~    whatami    ~   if W==1
    +---------------+
    ~  [locators]   ~   if L==1
    +---------------+
    ~    [links]    ~   neighbour psid list
    +---------------+
    ~   [weights]   ~   if H==1: per-link weights
    +---------------+
    ```

    Flag 意義：

    - `P`
        - 是否帶 `zid`
    - `W`
        - 是否帶 `whatami`
    - `L`
        - 是否帶 `locators`
    - `H`
        - 是否帶 `link_weights`
    - `G`
        - `is_gateway`

    這個 body 的作用是：

    - 宣告某個 router 的 `psid`
    - 宣告它目前知道的鄰居 `links`
    - 可選地附上 `locators` 和 `weights`

    Router 收到 `LinkStateList` 後，會把它寫進本地 `graph`，再重新計算 `tree` 和 `next hop`。

    參考：

    - Source / LinkState definitions: `zenoh/src/net/protocol/linkstate.rs`
    - Source / OAM_LINKSTATE constant: `commons/zenoh-protocol/src/network/oam.rs`
    - Source / encode LinkStateList into OAM body: `zenoh/src/net/protocol/network.rs`
    - Source / apply LinkStateList to graph: `zenoh/src/net/protocol/network.rs`

??? "INIT 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/open-accept.html

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
    ~  <u8;z16>     ~   Cookie - InitAck (A==1) ONLY
    +---------------+
    ~  [InitExts]   ~   if Z==1
    +---------------+
    ```

??? "OPEN 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/open-accept.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|T|A|   OPEN  |   ID = 0x02
    +-+-+-+---------+
    %    lease      %   VLE-encoded lease duration (unit per T flag)
    +---------------+
    % initial_sn    %   VLE-encoded initial sequence number
    +---------------+
    ~  <u8;z16>     ~   Cookie (OpenSyn, A==0 only)
    +---------------+
    ~  [OpenExts]   ~   if Z==1
    +---------------+
    ```

??? "CLOSE 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/close.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|S|  CLOSE  |   ID = 0x03
    +-+-+-+---------+
    |    reason     |   Close reason code (u8)
    +---------------+
    ~  [CloseExts]  ~   if Z==1
    +---------------+
    ```

??? "KEEPALIVE 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/transport/keep-alive.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X| KALIVE  |   ID = 0x04
    +-+-+-+---------+
    ~  [KAliveExts] ~   if Z==1
    +---------------+
    ```

<!-- markdownlint-disable MD046 -->
??? "FRAME 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/wire/frame-format.html

    * 第一個 byte 是 ID (`0x05`) 和 flag
        * `R`: 是否 reliable
        * `Z`: 是否用到 extensions
    * 第二個部份是 sequence number
        * 使用 VLE，長度會隨著序號增長而增加
    * 接著是可擴充欄位
    * 最後會放 one or more network messages

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|R|  FRAME  |   ID = 0x05
    +-+-+-+---------+
    %    seq_num    %   VLE-encoded sequence number (resolution-dependent)
    +---------------+
    ~  [FrameExts]  ~   if Z==1
    +---------------+
    ~  [NetworkMsg] ~   one or more back-to-back serialised NetworkMessages
    +---------------+
    ```
<!-- markdownlint-enable MD046 -->

<!-- markdownlint-disable MD046 -->
??? "FRAGMENT 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/transport/fragmentation.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|R| FRAGMENT|   ID = 0x06
    +-+-+-+---------+
    %    seq_num    %   VLE-encoded sequence number
    +---------------+
    ~  [FragExts]   ~   if Z==1
    +---------------+
    ~  [fragment]   ~   raw fragment bytes
    +---------------+
    ```
<!-- markdownlint-enable MD046 -->

<!-- markdownlint-disable MD046 -->
??? "JOIN 封包"

    `JOIN` 是 multicast session 使用的 transport message，可以視為 multicast 情境下與 `INIT` / `OPEN` 對應的協商封包。

    Spec: https://spec.zenoh.io/spec/1.0.0/transport/join.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|S|T|  JOIN   |   ID = 0x07
    +-+-+-+---------+
    |    version    |   Protocol version (u8); current = 0x09
    +---------------+
    |zid_len|X|X|WAI|   Packed byte
    +-+-+-+-+-+-+-+-+
    ~      ZID      ~   ZenohID (1 + zid_len bytes)
    +---------------+
    |X|X|X|X|RID|FSN|   if S==1: Resolution byte
    +---------------+
    |   batch_lo    |   if S==1: Batch size (u16 LE)
    +---------------+
    |   batch_hi    |
    +---------------+
    %    lease      %   VLE-encoded lease duration (unit per T flag)
    +---------------+
    %  next_sn_re   %   VLE next reliable sequence number
    +---------------+
    %  next_sn_be   %   VLE next best-effort sequence number
    +---------------+
    ~  [JoinExts]   ~   if Z==1
    +---------------+
    ```
<!-- markdownlint-enable MD046 -->

#### Network Messages

預設 transport 下，network messages 會被包在 `FRAME` 裡。
low-latency unicast 模式下，transport 也可能直接序列化 network message，而不經過 `FRAME`。

<!-- markdownlint-disable MD046 -->
??? "Network OAM 封包"

    Orchestration, Administration, Maintenance (OAM) 相關的 network message。

    Spec: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

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

??? "PUSH 封包"

    用在 publish 資料上面。

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html

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

??? "REQUEST 封包"

    用在發送 query 資料上。

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

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
    ~  [req_exts]   ~   if Z==1
    +---------------+
    ~  RequestBody  ~   QUERY (0x03) sub-message
    +---------------+
    ```

??? "RESPONSE 封包"

    用在回覆 query 資訊上面。

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/reply.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|N| Response|   ID = 0x1B
    +-+-+-+---------+
    % request_id:z32%   VLE - correlates to the originating REQUEST
    +---------------+
    % key_scope:z16 %
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    +---------------+
    ~  [reply_exts] ~   if Z==1
    +---------------+
    ~  ResponseBody ~   REPLY (0x04) or ERR (0x05) sub-message
    +---------------+
    ```

??? "RESPONSE_FINAL 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/reply.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X| ResFinal|   ID = 0x1A
    +-+-+-+---------+
    % request_id:z32%   VLE - must match the originating REQUEST
    +---------------+
    ~  [rf_exts]    ~   if Z==1
    +---------------+
    ```

??? "DECLARE 封包"

    `DECLARE` 是一個外層 network message，它的 body 裡只會放一個 declaration body。

    Spec: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|I| DECLARE |   ID = 0x1E
    +-+-+-+---------+
    %interest_id:z32%   if I==1: VLE interest ID this declare responds to
    +---------------+
    ~  [decl_exts]  ~   if Z==1 (QoS, Timestamp, NodeId)
    +---------------+
    ~  declaration  ~   exactly one DeclareBody sub-message
    +---------------+
    ```

??? "INTEREST 封包"

    比較晚加入的節點如果要得知當前網路上的狀態，就必須送出 `INTEREST` 來得到之前已經送出過的 `DECLARE`。

    Spec: https://spec.zenoh.io/spec/1.0.0/session/interests.html

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|Mod| INTEREST|   ID = 0x19; Mod in bits 6:5
    +-+-+-+---------+
    %    id : z32   %   VLE - Interest identifier
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
<!-- markdownlint-enable MD046 -->

#### Embedded Body Formats

以下這些 body format 都不能獨立存在，而是必須放在外層 message 裡。

##### Data Sub-Messages

以下是 network messages 會對應到的 data sub-messages：

- `PUSH`: `PUT`、`DEL`
- `REQUEST`: `QUERY`
- `RESPONSE`: `REPLY`、`ERR`

<!-- markdownlint-disable MD046 -->
??? "PUT 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html

    ```text
      7 6 5 4 3 2 1 0
     +-+-+-+-+-+-+-+-+
     |Z|E|T|   PUT   |   ID = 0x01
     +-+-+-+---------+
     ~  ts: <u8;z16> ~   if T==1: Timestamp
     +---------------+
     ~   encoding    ~   if E==1: Encoding field
     +---------------+
     ~  [put_exts]   ~   if Z==1 (SourceInfo, Shm, Attachment)
     +---------------+
     ~ pl: <u8;z32>  ~   Payload bytes
     +---------------+
    ```

??? "DEL 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html

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

??? "QUERY 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

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

??? "REPLY 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/reply.html

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

??? "ERR 封包"

    Spec: https://spec.zenoh.io/spec/1.0.0/data-plane/reply.html

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
<!-- markdownlint-enable MD046 -->

##### Declaration Body Formats

`DECLARE` 的 body 不是 data sub-message，而是另一套 declaration body。
這些 body 一次只會出現一個。

<!-- markdownlint-disable MD046 -->
??? "D_KEYEXPR / U_KEYEXPR"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

    ```text
    Flags:
      N  If N==1, key suffix is present.
      Z  If Z==1, extension chain follows.

    D_KEYEXPR (0x00)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|N| D_KEXPR |   ID = 0x00
    +-+-+-+---------+
    %  expr_id:z16  %   VLE - ExprId being assigned
    +---------------+
    % key_scope:z16 %   VLE - scope of the base expression (0 = global)
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16> suffix string
    +---------------+
    ~  [decl_exts]  ~   if Z==1
    +---------------+

    U_KEYEXPR (0x01)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X| U_KEXPR |   ID = 0x01
    +-+-+-+---------+
    %  expr_id:z16  %   VLE - ExprId to release
    +---------------+
    ~  [decl_exts]  ~   if Z==1
    +---------------+
    ```

??? "D_SUBSCRIBER / U_SUBSCRIBER"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

    ```text
    Flags:
      N  If N==1, key suffix is present.
      M  If M==1, sender's mapping space; else receiver's.
      Z  If Z==1, extension chain follows.

    D_SUBSCRIBER (0x02)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|N|  D_SUB  |   ID = 0x02
    +-+-+-+---------+
    %  subs_id:z32  %   VLE - Subscriber entity ID
    +---------------+
    % key_scope:z16 %   VLE ExprId (0 = global scope)
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    +---------------+
    ~  [decl_exts]  ~   if Z==1
    +---------------+

    U_SUBSCRIBER (0x03)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X|  U_SUB  |   ID = 0x03
    +-+-+-+---------+
    %  subs_id:z32  %   VLE - Subscriber entity ID to cancel
    +---------------+
    ~  [decl_exts]  ~   if Z==1 (MUST include WireExpr extension 0x0F)
    +---------------+

    WireExpr extension payload for U_SUBSCRIBER (ZBuf, ID=0x0F, M=true)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |X|X|X|X|X|X|M|N|   N=Named (suffix present), M=Mapping
    +-+-+-+---------+
    % key_scope:z16 %   VLE ExprId
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    ```

??? "D_QUERYABLE / U_QUERYABLE"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

    ```text
    D_QUERYABLE (0x04)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|N|  D_QBL  |   ID = 0x04
    +-+-+-+---------+
    %  qbls_id:z32  %   VLE - Queryable entity ID
    +---------------+
    % key_scope:z16 %   VLE ExprId
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    +---------------+
    ~  [decl_exts]  ~   if Z==1
    +---------------+

    QueryableInfo extension (Z64, ID=0x01, M=false)
    bit 0      : Complete flag
    bits 17:1  : distance (u16)

    encoded z64 value:
    ((u64)distance << 1) | (u64)complete

    U_QUERYABLE (0x05)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X|  U_QBL  |   ID = 0x05
    +-+-+-+---------+
    %  qbls_id:z32  %   VLE - Queryable entity ID to cancel
    +---------------+
    ~  [decl_exts]  ~   if Z==1 (MUST include WireExpr extension 0x0F)
    +---------------+

    WireExpr extension is the same as U_SUBSCRIBER.
    ```

??? "D_TOKEN / U_TOKEN"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

    ```text
    D_TOKEN (0x06)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|M|N|  D_TKN  |   ID = 0x06
    +-+-+-+---------+
    % token_id:z32  %   VLE - Token entity ID
    +---------------+
    % key_scope:z16 %   VLE ExprId
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    +---------------+
    ~  [decl_exts]  ~   if Z==1
    +---------------+

    U_TOKEN (0x07)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X|  U_TKN  |   ID = 0x07
    +-+-+-+---------+
    % token_id:z32  %   VLE - Token entity ID to retract
    +---------------+
    ~  [decl_exts]  ~   if Z==1 (MUST include WireExpr extension 0x0F)
    +---------------+

    WireExpr extension is the same as U_SUBSCRIBER.
    ```

??? "D_FINAL"

    Spec: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

    ```text
    Flags:
      Z  If Z==1, extension chain follows.

    D_FINAL (0x1A)
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |Z|X|X| D_FINAL |   ID = 0x1A
    +-+-+-+---------+
    ~  [final_exts] ~   if Z==1
    +---------------+

    沒有額外 body 欄位。
    它會搭配外層 DECLARE 的 interest_id，表示某個 INTEREST 的 current declarations 已經送完。
    ```
<!-- markdownlint-enable MD046 -->

#### Extensions

許多 Zenoh message 不只靠固定欄位，還會在 `Z=1` 時掛上一串 extension chain。
extension 不是獨立封包，而是依附在某個 message 或 body 後面的附加欄位。

Spec: https://spec.zenoh.io/spec/1.0.0/wire/primitives.html

共通上可以先記住四件事：

- 是否有 extension chain 是由外層 message header 的 `Z` bit 決定
- 每個 extension 自己也有 header，會指出 extension id、payload type、以及是否 mandatory
- `Z` 在 extension header 中表示後面還有沒有下一個 extension
- 各種 message 支援的 extension 並不相同，例如 `PUSH`、`DECLARE`、`PUT` 各自可接受的 extension 類型不同

最重要的一點是：extension 都有 `ID`，但這個 `ID` 通常不是全域唯一。
同一個 `ID` 在不同 message 上，可能代表不同 extension。
因此辨識一個 extension 時，至少要同時看：

- 它掛在哪個 message
- `ID`
- `ENC`
- `M`

也就是說，比較接近下面這種概念：

```text
(message type, ext id, enc) -> extension meaning
```

```text
Extension
+---------------+
| header byte   |   Z / ENC / M / ID
+---------------+
| payload       |   Unit / Z64 / ZBuf
+---------------+
```

##### Extension Header

```text
 7 6 5 4 3 2 1 0
+-+-+-+-+-+-+-+-+
|Z|ENC|M|  ID   |
+-+-+-+-+-------+
```

- `Z`:
    - `1` 代表還有下一個 extension
    - `0` 代表這是最後一個 extension
- `ENC`:
    - payload 的編碼格式
- `M`:
    - `1` 代表 mandatory，receiver 必須理解這個 extension
    - `0` 代表 optional，不理解時可以略過
- `ID`:
    - extension identifier，範圍 `0x0` 到 `0xF`

##### ENC Values

```text
Unit  (ENC=0b00)
[header : u8]

Z64   (ENC=0b01)
[header : u8]
% value : z64 %

ZBuf  (ENC=0b10)
[header : u8]
% length : z32 %
~  [u8 × length] ~
```

- `Unit`:
    - 只有 header，沒有 payload
- `Z64`:
    - payload 是一個 VLE-encoded `u64`
- `ZBuf`:
    - payload 是 `z32` 長度加上原始 bytes
- `ENC=0b11`:
    - reserved，不能使用

##### Extension Chain

```text
[ext1 header | Z=1] [ext1 payload]
[ext2 header | Z=1] [ext2 payload]
[ext3 header | Z=0] [ext3 payload]
[first message body field]
```

也就是說 extension 不是用外層 length 包起來，而是靠每個 extension header 裡的 `Z` bit 串成一條 chain。

##### 常見 Extension 類型

- `QoS`
- `Timestamp`
- `NodeId`
- `ResponderId`
- `SourceInfo`
- `Attachment`
- `Shm`

##### 各 Message 的 Extension 對照

以下是這份文件目前最常碰到的 extension。
建議把它們理解成「某個 message 可接受哪些 extension」，而不是一份全域唯一的 extension 清單。

<!-- markdownlint-disable MD046 -->
??? "FRAME / Transport OAM 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `Y` | `QoS priority lane` | 低 3 bits 是 priority，其餘 reserved |

    參考：

    - Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "Network OAM 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `N` | `QoS` | 與 common network QoS 相同 |
    | `0x2` | `ZBuf` | `N` | `Timestamp` | Timestamp primitive |

    參考：

    - Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "DECLARE 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `N` | `QoS` | common network QoS payload |
    | `0x2` | `ZBuf` | `N` | `Timestamp` | Timestamp primitive |
    | `0x3` | `Z64` | `Y` | `NodeId` | `% node_id : z64 %` |

    參考：

    - Declarations: https://spec.zenoh.io/spec/1.0.0/session/declarations.html
    - Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "PUSH 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `N` | `QoS` | common network QoS payload |
    | `0x2` | `ZBuf` | `N` | `Timestamp` | Timestamp primitive |
    | `0x3` | `Z64` | `Y` | `NodeId` | `% node_id : z64 %` |

    參考：

    - Push: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html
    - Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "REQUEST 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `N` | `QoS` | common network QoS payload |
    | `0x2` | `ZBuf` | `N` | `Timestamp` | Timestamp primitive |
    | `0x3` | `Z64` | `Y` | `NodeId` | `% node_id : z64 %` |
    | `0x4` | `Z64` | `Y` | `QueryTarget` | spec-defined request routing hint |
    | `0x5` | `Z64` | `N` | `Budget` | VLE-encoded remaining budget |
    | `0x6` | `Z64` | `N` | `Timeout` | VLE-encoded timeout |

    參考：

    - Query: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html
    - Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "RESPONSE 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `N` | `QoS` | common network QoS payload |
    | `0x2` | `ZBuf` | `N` | `Timestamp` | Timestamp primitive |
    | `0x3` | `ZBuf` | `N` | `ResponderId` | packed `zid_len` + `ZID` + `eid:z32` |

    參考：

    - Reply: https://spec.zenoh.io/spec/1.0.0/data-plane/reply.html
    - Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "RESPONSE_FINAL 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `N` | `QoS` | common network QoS payload |
    | `0x2` | `ZBuf` | `N` | `Timestamp` | Timestamp primitive |

    參考：

    - Reply: https://spec.zenoh.io/spec/1.0.0/data-plane/reply.html
    - Message Reference: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "PUT 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `ZBuf` | `N` | `SourceInfo` | packed `zid_len` + `ZID` + `eid:z32` + `sn:z32` |
    | `0x2` | `Unit` | `Y` | `Shm` | no payload |
    | `0x3` | `ZBuf` | `N` | `Attachment` | opaque bytes |

    參考：

    - Push: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html

??? "DEL 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `ZBuf` | `N` | `SourceInfo` | packed `zid_len` + `ZID` + `eid:z32` + `sn:z32` |
    | `0x2` | `ZBuf` | `N` | `Attachment` | opaque bytes |

    參考：

    - Push: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html

??? "QUERY 的 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `ZBuf` | `N` | `SourceInfo` | packed `zid_len` + `ZID` + `eid:z32` + `sn:z32` |
    | `0x3` | `ZBuf` | `N` | `QueryBody` | opaque query body bytes |
    | `0x5` | `ZBuf` | `N` | `Attachment` | opaque bytes |

    參考：

    - Query: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

??? "D_QUERYABLE 的額外 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x1` | `Z64` | `N` | `QueryableInfo` | bit `0`=`Complete`, bits `17:1`=`distance` |

    參考：

    - Declarations: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

??? "U_SUBSCRIBER / U_QUERYABLE / U_TOKEN 的額外 Extensions"

    | Ext ID | Type | M | Extension | Payload |
    | --- | --- | --- | --- | --- |
    | `0x0F` | `ZBuf` | `Y` | `WireExpr` | flags byte + `key_scope:z16` + optional `key_suffix` |

    參考：

    - Declarations: https://spec.zenoh.io/spec/1.0.0/session/declarations.html
<!-- markdownlint-enable MD046 -->

##### 常見 Payload 格式

<!-- markdownlint-disable MD046 -->
??? "Timestamp extension 的 ZBuf payload"

    ```text
    % ntp64 : z64  %   64-bit NTP timestamp
    % zid_len: z8 %   ZID byte count
    ~      ZID     ~   zid_len bytes
    ```

    參考：

    - Wire Primitives / Timestamp: https://spec.zenoh.io/spec/1.0.0/wire/primitives.html

??? "SourceInfo extension 的 ZBuf payload"

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |zid_len|X|X|X|X|   actual ZID byte count = 1 + zid_len
    +-+-+-+-+-+-+-+-+
    ~      ZID      ~   source node ZenohID
    +---------------+
    %   eid : z32   %   source entity ID
    +---------------+
    %    sn  : z32  %   source sequence number
    ```

    參考：

    - Push / Common Data Extensions: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html

??? "ResponderId extension 的 ZBuf payload"

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |zid_len|X|X|X|X|   actual ZID byte count = 1 + zid_len
    +-+-+-+-+-+-+-+-+
    ~      ZID      ~   responder ZenohID
    +---------------+
    %   eid : z32   %   responder entity ID
    ```

    參考：

    - Message Reference / RESPONSE: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "NodeId extension 的 Z64 payload"

    ```text
    % node_id : z64 %   16-bit node identifier encoded as z64
    ```

    參考：

    - Message Reference / Common Network Extensions: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "QoS extension 的 Z64 payload"

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |0|r|F|E|D|prio |
    +-+-+-+-+-+-+-+-+
    ```

    - `prio`
        - bits `2:0`
        - priority class `0-7`
    - `D`
        - bit `3`
        - `Don't-drop`，對應 block congestion control
    - `E`
        - bit `4`
        - `Express`，要求這個 message 不參與一般 batching
    - `F`
        - bit `5`
        - `Don't-drop-first`，對應 `BlockFirst`
    - `r`
        - bit `6`
        - reserved
    - bit `7`
        - reserved

    參考：

    - Message Reference / Common Network Extensions: https://spec.zenoh.io/spec/1.0.0/wire/message-format.html

??? "QueryableInfo extension 的 Z64 payload"

    ```text
    encoded_value = ((u64) distance << 1) | (u64) complete
    ```

    參考：

    - Declarations / D_QUERYABLE: https://spec.zenoh.io/spec/1.0.0/session/declarations.html

??? "QueryTarget extension 的 Z64 payload"

    ```text
    0x00   BestMatching
    0x01   All
    0x02   AllComplete
    ```

    - `0x00`
        - 送到單一 best-matching queryable
    - `0x01`
        - 送到所有 matching queryables
    - `0x02`
        - 只送到 `Complete` flag 為真的 matching queryables

    參考：

    - Query / REQUEST Extensions: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

??? "Budget extension 的 Z64 payload"

    ```text
    % budget : z64 %   upper bound on accepted RESPONSE count
    ```

    - 實際語意是 non-zero `u32` 上限
    - 沒帶這個 extension 代表 unlimited

    參考：

    - Query / REQUEST Extensions: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

??? "Timeout extension 的 Z64 payload"

    ```text
    % timeout_ms : z64 %   timeout in milliseconds
    ```

    - 單位是 milliseconds
    - 沒帶這個 extension 代表 querier 不在 wire 上宣告 timeout

    參考：

    - Query / REQUEST Extensions: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

??? "QueryBody extension 的 ZBuf payload"

    ```text
    ~   encoding    ~   Encoding field
    +---------------+
    ~ pl: <u8;z32>  ~   query body payload bytes
    ```

    參考：

    - Query / QueryBody Extension: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

??? "Attachment extension 的 ZBuf payload"

    ```text
    % length : z32 %
    ~  [u8 × length] ~   opaque bytes
    ```

    - 協定本身不定義內部 schema
    - 只保證它是一段 opaque buffer

    參考：

    - Push / Attachment Extension: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html
    - Query / Attachment Extension: https://spec.zenoh.io/spec/1.0.0/data-plane/query.html

??? "Shm extension 的 Unit payload"

    ```text
    [header only]
    ```

    - 沒有 payload
    - presence 本身就表示 shared-memory payload

    參考：

    - Push / Shm Extension: https://spec.zenoh.io/spec/1.0.0/data-plane/push.html

??? "WireExpr extension 的 ZBuf payload"

    ```text
     7 6 5 4 3 2 1 0
    +-+-+-+-+-+-+-+-+
    |X|X|X|X|X|X|M|N|
    +-+-+-+-+-+-+-+-+
    % key_scope:z16 %   VLE ExprId
    +---------------+
    ~  key_suffix   ~   if N==1: <u8;z16>
    ```

    - `N`
        - `1` 代表 suffix 存在
    - `M`
        - `1` 代表 sender mapping space

    參考：

    - Declarations / U_SUBSCRIBER, U_QUERYABLE, U_TOKEN: https://spec.zenoh.io/spec/1.0.0/session/declarations.html
<!-- markdownlint-enable MD046 -->
