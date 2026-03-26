---
title: Eclipse SDV 介紹
description: 介紹 Eclipse SDV 組織以及旗下 project
keywords:
  - SDV
---

Eclipse SDV 是 Eclipse Foundation 旗下其中一個 Working Group，主要專注於 Software-Defined Vehicle，他們的目標是希望可以透過開源讓 OEM 廠商可以共用基礎軟體。

## 著名 project

下面列出常見的 project，完整列表可以參考[官網](https://eclipsesdv.org/integration-projects/)

* [S-CORE](https://github.com/eclipse-score)：符合 safety-critical 的中介軟體堆疊
* [Zenoh](https://github.com/eclipse-zenoh)：知名通訊協定，可以用在機器人、車用、IoT
* [iceoryx](https://github.com/eclipse-iceoryx)：知名的共享記憶體通訊框架
* [uProtocol](https://github.com/eclipse-uprotocol)：提供一個統一的車用通訊框架，整合各個不同協定，最早由 GM 主導開發，後來由社群維護
* [Pullpiri](https://github.com/eclipse-pullpiri)：由 LG 發起，要把 cloud-native 的開發體驗帶入車中，能夠動態佈署各種微服務
* [Eclipse SDV Blueprints](https://github.com/eclipse-sdv-blueprints)：用來展示各個 Eclipse project 的整合，提供給開發者參考
    * [fleet-management](https://github.com/eclipse-sdv-blueprints/fleet-management)：展示如何進行車隊管理
    * [service-to-signal](https://github.com/eclipse-sdv-blueprints/service-to-signal)：展示如何用 uProtocol 透過 Kuksa 來取得 COVESA 的資訊

## 常用連結

* [Eclipse SDV 官網](https://eclipsesdv.org/)：官方網站，有基本介紹以及列出參與的 project
* [slack channel](https://app.slack.com/client/T02MS1M89UH/C02MS1M9BH7)：Eclipse SDV 主要是用 slack 來做社群通訊，想了解動態可以加入
