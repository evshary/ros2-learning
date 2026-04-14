---
title: DDS 介紹
description: DDS 簡單介紹
keywords:
  - DDS
---

DDS (Data Distribution Service) 是一種 pub-sub 的通訊規格，希望可以達到可靠、高效能、安全、即時、可擴展、並且以資料為導向的傳輸。為了達成這個目的，DDS 有如下特色功能：

* 以資料為導向：過去的傳輸重視的是「誰連到誰」，所以我們會需要明確知道通訊者的 IP 位址並且開 socket 進行連接。DDS 重視的是「傳什麼資料」，任何人發佈的資料都會指定 topic，有興趣的人可以自行訂閱，而不需要管這資料是誰傳來的。
* 去中心化架構：DDS 並非以往 server-client 的架構，每個通訊節點都是對等的。因此任何人都可以隨意加入或離開通訊網路，而不會有任何通訊上的問題。
* 豐富的 QoS：為了達到可靠、高效能，DDS 有非常多的 QoS(Quality of Service)，使用者可以依照自己想要的通訊品質來進行調整，例如reliability, durability, transport priority等等。

[DDS 的規格](https://www.omg.org/omg-dds-portal/)制定是由 OMG (Object Management Group) 這個組織所負責，OMG 其實也有制定不少跟程式設計師有關的規格，舉例來說 Object Oriented (物件導向)常用到的 UML，就是由 OMG 所制定。

在 [DDS foundation](https://www.dds-foundation.org/omg-dds-standard/) 裡面有許多關於 DDS 的文件，其中比較值得注意的是下面這五份：

* DDS v1.4: 定義 DCPS，主要是 API 的制定
* DDSI-RTPS v2.3: 定義 DDSI，定義 DDS 的行為
* DDS-XTypes v1.2: 用在 topic 的 data type 規範
* DDS-Security v1.1: 定義 security 的規範
* Interface Definition Language (IDL) v4.2: IDL 的定義規範

DDS 最初是用在軍事、航太上，已經有悠久的歷史了，後來 ROS 2 在尋找可利用的通訊層時也看上 DDS 的可靠穩定，所以 DDS 也開始被應用在機器人領域。

DDS 目前有很多種選擇：

* CycloneDDS：ADLINK (後來 spin-off 出去叫做 ZettaScale) 提出的開源版輕量化 DDS。ROS 2 的 Galactic 版本是以 CycloneDDS 為預設通訊層。
* OpenSplice DDS：原本是 Prismtech 的主力產品(後來被 ADLINK 買下後又 spin-off 出去成為 ZettaScale)，有分開源版和商業版。
* FastDDS：Eprosima 的 DDS 開源解決方案，跟 CycloneDDS 一樣被應用在 ROS 2 上。
* Connext DDS：RTI 這家公司所提出的 DDS，並沒有開源。
* OpenDDS：Open source 版本的 DDS。

目前個人推薦的是使用 CycloneDDS，輕量且效能高，主要程式是用 C 撰寫而成。
