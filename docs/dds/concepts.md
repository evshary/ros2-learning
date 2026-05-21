---
title: DDS 核心概念
description: 介紹 DDS 的核心角色、名詞以及資料流
keywords:
  - DDS
---

接下來我們來先探到 DDS 的概念，我們可以想像 DDS 所有參與者都可以看到這個網路中的所有資訊，不論是有哪些 data writer、data reader、topic 等等。
我們稱這些資訊都在同個 GDS (global data space) 之中。
當然實際上並沒有魔法———有一個共通的空間來存這些資訊，這其實是靠著 DDS protocol 彼此交換訊息，在每個參與者本地端建立起自己的 GDS，然後不斷去更新它。

接下來讓我們參考下圖([圖源](https://www.mdpi.com/2218-6581/14/5/63))來介紹 DDS 協定中常見的角色。

![DDS architecture](./images/DDS%20architecture.png)

## DDS 角色

### GDS (DDS Domain)

如同前面提到的 GDS 是一個用來儲存所有 DDS 網路資訊的概念圖，但這邊有個 DDS Domain 的概念值得注意。
有時候我們會希望可以切分不同網域，建立邏輯隔離層，確保某些節點群的資訊不會互相干擾。
這樣的設計在 DDS 中叫做 DDS domain，不同的 domain 的節點彼此是無法通訊的。

在 ROS 中，有 ROS_DOMAIN_ID 的設計，而這個設計有 ROS_DOMAIN_ID 的設計，而這個設計就是使用到 DDS domain 的概念。

### Domain Participant

DDS 節點要參與某個 domain 的通訊，就會需要建立一個 participant 的身份。
這個 participant 內部包含後面我們會提到的 publisher、subscriber 等等，另外也包含 DDS 節點用來發現彼此的 builtin entities。

### Topic

Topic 是 DDS data-centric 通訊的重點，定義了「這筆資料是什麼」。
Topic 包含以下資訊：

* topic name：專屬的名稱
* data type：資料格式應該如何
* QoS：有哪些 quality of service

### Publisher / Subscriber

`Publisher` 和 `Subscriber` 可以看成 writer / reader 的管理容器。
真正送資料與收資料的，通常還是底下的 `DataWriter` 和 `DataReader`。

### DataWriter / DataReader

`DataWriter` 負責將資料寫入某個 topic，`DataReader` 則負責接收對應 topic 的資料。

這一組概念是 DDS 資料流的核心：

* writer 宣告自己會發送某種 topic
* reader 宣告自己想接收某種 topic
* 當 name、type、QoS 相容時，雙方才可能成功配對

## 資料流

### 從 writer 到 reader

跟點對點的傳統通訊不一樣，DDS 不重視要傳給誰，而是重視要發佈什麼東西，也就是我們前面說的 topic。
上層應用程式使用 DDS API 發布資料後，DDS middleware 就會透過 data writer 發布某個 topic 的資料。
對面的應用程式由於也使用 DDS API 說要關注某個 topic 資料，DDS middleware 的 data reader 就會從 writer 那邊收取資料，並且通知上層應用已經有收到資料了。

### topic 配對

如同前面我們說過的 topic 有三個部份 name、type、QoS，writer 和 reader 的 topic 要都對上，才能夠進行連線。
不過值得注意的是 QoS 並不是要百分之百完全一致才行，而是只要符合一定規則就能相通，這個我們稱之為 RxO。

### 一對多、多對多

由於 DDS 使用 topic 的方式來傳輸資料，所以當然可以有多個 publisher 對一個 topic 送資料，或是多個 subscriber 從同一個 topic 收資料。
不過 DDS 他特別有效率的地方是如果發現有多個接收方的時候，它不會使用 unicast 的方式，而是使用 multicast 來傳送，這可以有效減少網路的封包數量。
