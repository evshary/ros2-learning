---
title: Zenoh 協定
description: 介紹 Zenoh 協定內容以及如何用 dissector 觀察
keywords:
  - Zenoh
---

## zenoh-dissector

如果想要學習 Zenoh Protocol，那我們一定需要使用 [zenoh-dissector](https://github.com/eclipse-zenoh/zenoh-dissector) 這個 wireshark plugin 來看。

* 先確保已經有安裝 wireshark

```bash
sudo add-apt-repository ppa:wireshark-dev/stable
sudo apt update
sudo apt install wireshark
```

* 到 [GitHub Release page](https://github.com/eclipse-zenoh/zenoh-dissector/releases) 下載對應自己作業系統的 library
* 解壓縮後會看到 `libzenoh_dissector.so` 這個檔案，要放到對應的位置，這邊是以 `4.6.x` 為例，你要放到對應自己版本的資料夾

```bash
mkdir -p ~/.local/lib/wireshark/plugins/4.6/epan
cp libzenoh_dissector.so ~/.local/lib/wireshark/plugins/4.6/epan/libzenoh_dissector.so
```

* 開啟 wireshark 之後，就可以輸入 zenoh 來查看 protocol
* 我們可以用 Zenoh 最基本的範例來觀察，注意的是目前預設 Zenoh plugin 只會追蹤 port 7447，所以讓我們特別指定 listener 和 connector

```bash
./z_sub -l tcp/127.0.0.1:7447
./z_pub -e tcp/127.0.0.1:7447
```

## Zenoh 協定

基本上 Zenoh 的協定可以分成兩種：

* Scouting：用來發現不同節點的協定，預設會監聽 multicast 的位址 `224.0.0.224:7446`
* Session / Data：用在一般傳輸上面，一般來說我們會使用 port `7447`

如果用流程來理解，大致可以分成這幾段：

* `Scouting`
* `Session Establishment`
* `Session State Synchronization`
* `Data Plane`
* `Transport Maintenance`
* `Multicast`

### Scouting

在 Zenoh 中，不同節點可以不需要輸入固定 IP 位址就能彼此通訊，這就是依靠 Scouting 的機制。
Scouting 會廣播自己的資訊給其他人，其他節點就可以依靠這個資訊來建立 Zenoh 的連線。

Scouting 的流程大概如下：

```text
A                   B                   C
|       SCOUT       |                   |   (multicast/broadcast)
|─────────────────>|                   |
|         \──────────────────────────>|
|                   |                   |
|       HELLO       |                   |   (unicast, if B matches)
|<─────────────────|                   |
|                   |      HELLO        |   (unicast, if C matches)
|<──────────────────────────────────── |
```

A 會透過 multicast 或是 broadcast 發送 SCOUT 的訊息給其他人，
如果 B 或 C 有相對應的 Zenoh 服務，就會用 unicast 回傳 HELLO 的訊息給 A

參考：

* SCOUT: https://spec.zenoh.io/spec/1.0.0/scouting/scout.html
* HELLO: https://spec.zenoh.io/spec/1.0.0/scouting/hello.html

除了回應 `SCOUT` 之外，節點也可以主動週期性送出 `HELLO` 來宣告自己的存在。
這樣即使沒有先收到 `SCOUT`，其他節點也有機會得知目前網路上的 Zenoh 節點。

```text
A                   B                   C
|                   |                   |
|                   |      HELLO        |   (periodic multicast / broadcast)
|<─────────────────|                   |
|                   |                   |
|<──────────────────────────────────── |
```

參考：

* HELLO / Periodic Advertisement: https://spec.zenoh.io/spec/1.0.0/scouting/hello.html

### Session Establishment

Session establishment 指的是 unicast 情境下，雙方怎麼從「知道彼此存在」進入可傳輸資料的狀態。

* A 先發送 `INIT SYN` 給 B，B 回覆 `INIT ACK`，用來確認雙方的 protocol version、ZenohID 與參數
* 接著 A 發送 `OPEN SYN`，B 回覆 `OPEN ACK`
* 到這一步之後，session 才算正式建立

```text
A                           B
|                           |
|  INIT SYN  (A=0)          |   propose version, ZID, parameters
|─────────────────────────>|
|          INIT ACK  (A=1)  |   accept + cookie
|<─────────────────────────|
|  OPEN SYN  (A=0)          |   echo cookie + propose lease / initial_sn
|─────────────────────────>|
|          OPEN ACK  (A=1)  |   confirm; session is now active
|<─────────────────────────|
```

參考：

* Session Establishment (INIT & OPEN): https://spec.zenoh.io/spec/1.0.0/session/open-accept.html

### Session State Synchronization

Zenoh 不只是建立 session 之後就開始送資料，還需要同步目前的 session 狀態。
這一層最重要的是 `DECLARE` 和 `INTEREST`。

#### DECLARE / INTEREST

`DECLARE` 用來宣告 key expression alias、subscriber、queryable、liveliness token。
比較晚加入的節點如果要知道現有狀態，就會送出 `INTEREST`。

常見流程如下：

```text
A                                 B
|                                 |
|  INTEREST                       |   ask for current declarations
|────────────────────────────────>|
|                                 |
|                 DECLARE (I=1)   |   D_KEYEXPR / D_SUBSCRIBER / ...
|<────────────────────────────────|
|                 DECLARE (I=1)   |   more declarations if needed
|<────────────────────────────────|
|                                 |
|                 DECLARE (I=1)   |   D_FINAL
|<────────────────────────────────|
```

可以把它理解成：

* `INTEREST`：我要知道目前有哪些宣告
* `DECLARE(I=1)`：這些是對應的宣告內容
* `D_FINAL`：這一批 current declarations 已經送完

參考：

* Declarations: https://spec.zenoh.io/spec/1.0.0/session/declarations.html
* Interests: https://spec.zenoh.io/spec/1.0.0/session/interests.html

!!! info "為何只 declare subscriber / queryable"
    這裡有一個很值得特別說明的設計：Zenoh 會要求 session 內宣告 `subscriber`、`queryable`、`token`，但不要求 `publisher` 或 `querier` 先 declare。
    因為真正會影響路由決策的只有「誰想接收資料」與「誰能回答 query」，也就是 `subscriber` 與 `queryable`。
    因此我們可以忽略 `publisher` 和 `querier` 來減少 control plane 設計的複雜度。

### Data Plane

真正的資料傳輸主要可以拆成 `PUSH` 和 `QUERY / REPLY` 兩條路徑。

#### PUSH

`PUSH` 是最常見的 publish path，body 會是 `PUT` 或 `DEL`。

```text
Publisher                      Subscriber
|                               |
|  FRAME                        |
|    PUSH                       |   body = PUT or DEL
|──────────────────────────────>|
```

如果是 low-latency unicast session，`PUSH` 也可能不經過 `FRAME`，而是直接以 network message 傳送。

參考：

* Push (PUSH / PUT / DEL): https://spec.zenoh.io/spec/1.0.0/data-plane/push.html
* Frame Format: https://spec.zenoh.io/spec/1.0.0/wire/frame-format.html

#### QUERY / RESPONSE / RESPONSE_FINAL

Query plane 和一般 pub/sub 不同，一個 `REQUEST` 可能對應多個 `RESPONSE`，最後再用 `RESPONSE_FINAL` 收尾。

```text
Querier                         Queryable B              Queryable C
|                               |                        |
|  REQUEST                      |                        |
|──────────────────────────────>|                        |
|───────────────────────────────────────────────────────>|
|                               |                        |
|         RESPONSE              |                        |
|<──────────────────────────────|                        |
|                               |         RESPONSE       |
|<───────────────────────────────────────────────────────|
|                               |                        |
|         RESPONSE_FINAL        |                        |
|<──────────────────────────────|                        |
```

這個流程的重點是：

* 一個 `REQUEST` 可以收到多個 `RESPONSE`
* `RESPONSE` 的 body 會是 `REPLY` 或 `ERR`
* `RESPONSE_FINAL` 表示這次 query 的回覆流結束

參考：

* Query (REQUEST / QUERY): https://spec.zenoh.io/spec/1.0.0/data-plane/query.html
* Reply (RESPONSE / RESPONSE_FINAL / REPLY / ERR): https://spec.zenoh.io/spec/1.0.0/data-plane/reply.html
* Frame Format: https://spec.zenoh.io/spec/1.0.0/wire/frame-format.html

### Transport Maintenance

Session 建好之後，transport 還需要持續維護這個連線，並在必要時處理大封包。

#### KEEP_ALIVE / CLOSE

最基本的維護流程如下：

```text
A                           B
|                           |
|  FRAME / direct network   |   low-latency sessions may skip FRAME batching
|  messages                 |
|<────────────────────────>|
|  KEEP_ALIVE               |   every lease/4
|<────────────────────────>|
|                           |
|  CLOSE (when done)        |
|─────────────────────────>|
```

參考：

* Keep-Alive: https://spec.zenoh.io/spec/1.0.0/transport/keep-alive.html
* Close: https://spec.zenoh.io/spec/1.0.0/session/close.html
* Frame Format: https://spec.zenoh.io/spec/1.0.0/wire/frame-format.html

#### FRAGMENT

如果單一 payload 太大，Zenoh 可能無法把所有內容塞進同一個 `FRAME`，這時就會用 `FRAGMENT` 來切片。

```text
A                           B
|                           |
|  FRAGMENT seq=n           |   first chunk
|─────────────────────────>|
|  FRAGMENT seq=n+1         |   next chunk
|─────────────────────────>|
|  FRAGMENT seq=n+2         |   final chunk
|─────────────────────────>|
```

在 wire format 上，`FRAGMENT` 裡承載的是 raw fragment bytes，而不是完整的 network message 結構。

參考：

* Fragmentation: https://spec.zenoh.io/spec/1.0.0/transport/fragmentation.html
* Batching: https://spec.zenoh.io/spec/1.0.0/transport/batching.html

### Multicast

前面的 session establishment 主要是在講 unicast。
如果是 multicast transport，Zenoh 會使用 `JOIN` 而不是 `INIT` / `OPEN`。

#### JOIN

`JOIN` 可以視為 multicast 情境下的 session announcement / parameter synchronization。

```text
A                           B                   C
|                           |                   |
|          JOIN             |                   |   multicast
|─────────────────────────>|                   |
|           \────────────────────────────────>|
|                           |                   |
```

`JOIN` 會攜帶：

* version
* ZenohID
* lease
* next reliable sequence number
* next best-effort sequence number
* 可選的 resolution / batch size

因此它比較接近「我現在以這組 multicast transport 參數加入這個群組」的訊息，而不是一般 unicast 的 request / ack 握手。

參考：

* JOIN (Multicast): https://spec.zenoh.io/spec/1.0.0/transport/join.html
* Keep-Alive / Relationship to JOIN: https://spec.zenoh.io/spec/1.0.0/transport/keep-alive.html
