---
title: Autoware 簡介
description: Autoware 的簡單介紹，以及相關常用連結
keywords:
  - Autoware
  - 自駕車
---

Autoware 本質上是基於 ROS framework 上的自駕車軟體，所以也可以視為一系列的 ROS package 所組成。

由於 Autoware 目前的連結有點分散很難尋找，我創了個 [awesome list](https://github.com/evshary/awesome-autoware/) 來收集，方便尋找。歡迎大家在上面新增有意思的專案。

## 版本

在 Autoware 中有三個版本：

* [Autoware.ai](https://github.com/autowarefoundation/autoware/tree/autoware-ai)：最早期的版本，使用 ROS 1
* [Autoware.auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto)：升級到 ROS 2 後的新版本，值得注意的是這時候的程式碼是放在 GitLab
* [Autoware](https://github.com/autowarefoundation/autoware)：最新版基於 ROS 2 的 Autoware，這個版本主要目的是把 package 區分為 core 和 universe，方便維護
    * [Autoware.core](https://github.com/autowarefoundation/autoware.core)：自駕車中最重要的套件會放在這裡，這邊的程式碼需要經過嚴格的品質驗證
    * [Autoware.universe](https://github.com/autowarefoundation/autoware.universe)：其他第三方社群所開發的套件會放在此

目前 Autoware 沒有明確的版本細分，主要還是跟著 ROS 2 的版本變化，你可以到 [GitHub](https://github.com/autowarefoundation/autoware) 中切換不同 branch，例如 galactic, humble。

## 常用連結

* [Autoware 官網](https://www.autoware.org)：上面有 Autoware 的最基本介紹
* [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)：  最重要的部份，上面有非常詳細的安裝以及使用教學
* [GitHub Discussion](https://github.com/autowarefoundation/autoware/discussions)：Autoware 社群的討論區，有問題的話可以直接在上面發問，甚至也可以發布一些自己的作品
* [Project Wiki](https://github.com/autowarefoundation/autoware-projects/wiki)：對社群的運作有興趣的話可以參考這頁

## Working Group

Autoware 社群有許多 Working Group 所組成，可以選擇自己有興趣的參與，與社群討論各式各樣的議題

* [Working Group List](https://github.com/autowarefoundation/autoware-projects/wiki#working-group-list)：Working Group 的總覽
* [WG meeting minutes](https://github.com/autowarefoundation/autoware/discussions/categories/workgroup-meetings)：每次的 Working Group 會議記錄都會放到 GitHub Discussion 上
* [Calendar](https://calendar.google.com/calendar/u/0/embed?src=autoware.org_6lol0ho5ft0217h8c60pi1fm30@group.calendar.google.com&ctz=Asia/Taipei)：所有 Working Group 的會議時間
