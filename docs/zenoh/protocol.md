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
* Transport：用在一般傳輸上面，一般來說我們會使用 port 7447。

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

### Transport

Transport 的部份就是實際 Zenoh 怎麼傳輸資料。

* 一開始 A 會發送 INIT SYN 給 B，B 會回覆 INIT ACK，這是用來確保雙方協定一致
* 再來 A 就會開啟實際的 SESSION，B 一樣會回覆 OPEN ACK，表示確認
* 後續兩者就會透過 FRAME 的 message 來傳輸資料
* 其中也會利用 KEEP_ALIVE message 確保連線暢通
* 結束後 A 就可以發送 CLOSE 把這段通訊關閉

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
