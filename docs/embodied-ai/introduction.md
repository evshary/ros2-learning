---
title: Embodied AI 介紹
description: Embodied AI 介紹
keywords:
  - AI
---

隨著人工智慧的發展，有身體的 AI，也就是 Embodied AI 也開始被大量關注。
相對於一般雲端的聊天機器人，Embodied AI 強調物理載體，以及怎麼感知物理世界的資料，並且與其互動。
比起傳統機器人或自駕車的固定演算法，Embodied AI 因為能從物理世界中學習，更加能夠應對日益複雜的情況。

## 端到端 (End-to-End)

傳統的機器人演算法需要人類去撰寫複雜的邏輯和狀態機，例如 ROS 的 navigation 或是 moveit 套件，分別對應 AMR 或是機器手臂的應用。
這些套件的作法通常是把邏輯拆分成多個模組，例如感知模組、定位模組、控制模組等等，每個模組透過定義好的接口（像是 ROS topic）來傳遞資料。
然而面對複雜的真實世界，人類的設計往往無法考量周全，每遇到一個新的情境就要加入新的邏輯，這也是機器人領域過去發展的痛點。

後來有人想到既然人為設計這麼困難，那是否我們可以直接從原始資料產生出對應的行為呢？
那個流派就被稱作端到端。

一般來說端到端的輸入是各種感測器的資料，像是攝影機、光達等等，經過一個我們所訓練的 model，會直接產生出控制指令，像是馬達控制。
這樣的方式很有用，不用想辦法去處理各種 corner case，更容易應對真實世界的挑戰。

但是端到端也不是沒有缺點。第一，他無法進行語言的邏輯推理，例如我請機器人拿玩具，他就必須要了解玩具的定義是什麼。第二，我們無法了解為何 model 會做這樣的決定，例如自駕車決定在某個路口煞車時，我們需要知道他的判斷邏輯是什麼。這兩個情境說明了語言在其中的重要性。

因此 VLA (Vision-Language-Action) 就出現了。Vision 代表輸入接收影像資料，透過語言邏輯的判讀，最終產生出了行動。由於可以使用自然語言，就可以使用網路上的文本資料協助訓練，並且也提供更好的人機界面了。

VLA 目前最知名的專案有 Google 的 [RT-2](https://deepmind.google/blog/rt-2-new-model-translates-vision-and-language-into-action/)，以及 [OpenVLA](https://github.com/openvla/openvla)。

## 學習資源

* [具身智能技术指南](https://github.com/TianxingChen/Embodied-AI-Guide)：簡體中文的各種學習資源
