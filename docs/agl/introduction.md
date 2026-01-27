---
title: AGL 介紹
description: AGL 的簡單介紹
keywords:
  - SDV
---

Automotive Grade Linux 是 Linux Foundation 底下的組織。
主要目標是以提供汽車行業 OEM 廠商一個車載環境開發系統，也算是 Software Defined Vehicle (SDV) 的共通平台。
通常會用在車載訊息娛樂系統(In-Vehicle Infotainment, IVI)上。
如果參考[會員名單]([https://www.automotivelinux.org/about/members/](https://www.automotivelinux.org/about/members/))的話可以看到有很多汽車產業公司在裡面，特別是來自日本的公司。

AGL 的所提供的軟體是 Unified Code Base (UCB)，也就是希望各家車廠共同維護的基底程式碼。
UCB 整體架構是以 Yocto 為主，然後上面會有各式各樣跟汽車相關的應用層，例如 Flutter、KUKSA.val 等等，另外也有探討像是 Xen、VirtIO 等等的 hypervisor 的情境。

AGL 的版本名稱是使用一個形容詞加上水生動物，並且兩者要同樣字母開頭。
每次進板也都是按照字母順序依序增加，當然除此之外也會有數字的版號。

例如，目前最新版本就叫做 Terrific Trout (了不起的鱒魚)，版號是 v20.0。

可以在 wiki 頁面查詢過去[所有的 Release Note](https://wiki.automotivelinux.org/agl-distro/release-notes#agl_unified_code_base_release_notes)

## 常用連結

* [AGL 官網](https://www.automotivelinux.org/)：官方網站
* [AGL 官方文件](https://docs.automotivelinux.org/en/trout/)：AGL 的基礎教學
* [AGL Wiki](https://wiki.automotivelinux.org/start)：裡面提一些常用連結和資訊
* [程式碼](https://gerrit.automotivelinux.org/gerrit/admin/repos)：比較特別的是程式碼的部份是放在 gerrit，而不是 GitHub
* [行事曆](https://lists.automotivelinux.org/g/agl-dev-community/calendar)：AGL 的相關社群活動
* [Discord](https://discord.gg/c3tngsS3)：AGL 社群交流的地方
* [Automotive Grade Linux 系统介绍](https://paul.pub/automotive-linux/):一個不錯的 AGL 中文介紹
