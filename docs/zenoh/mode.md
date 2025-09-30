---
title: Zenoh 模式
description: Zenoh 三種常見的模式 (peer, client, router)
keywords:
  - Zenoh
---

Zenoh 的拓樸架構可以說十分彈性，只要能夠連上任意 Zenoh 網路上的節點，就可以與其他節點進行通訊。
可以參考下面的架構圖：

![Zenoh topology](images/full_topology.png)

這邊有三種角色

* Peer：通常在本地端執行，一般設定都是可以直接去中心化通訊
* Router：當要連接到遠端時(跨網域)，會使用 router 來避免 discovery 封包擴散整個網路
* Client：最簡化、少功能的 Zenoh 執行單位，使用的資源最少

可以看得出 Zenoh 的三種角色就是專門為了不同情境來設計。
