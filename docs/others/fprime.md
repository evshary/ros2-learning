# F´（F Prime）

F´（讀作 F Prime）是由 NASA 噴射推進實驗室（JPL）開發的開源飛行軟體與嵌入式系統框架，主要使用 C++。它最初為太空任務設計，也適合用於 CubeSat、SmallSat、儀器及其他需要可靠軟體架構的嵌入式系統。

F´ 採用元件化架構，透過定義清楚的介面連接各個元件，並提供執行緒、訊息佇列、作業系統抽象層、遙測、命令處理及事件記錄等常用功能。開發者也能使用 FPP（F Prime Prime）描述元件與系統拓樸，再自動產生部分 C++ 程式碼，減少重複工作。

## 常用資源

* [F´ 官方網站](https://fprime.jpl.nasa.gov/)：功能介紹、最新消息與專案入口。
* [F´ 官方文件](https://fprime.jpl.nasa.gov/latest/docs/)：教學、使用手冊、操作指南與 API 參考資料。
* [安裝指南](https://fprime.jpl.nasa.gov/latest/docs/getting-started/installing-fprime/)：開發環境需求及安裝步驟。
* [F´ GitHub](https://github.com/nasa/fprime)：原始碼、版本發布與問題追蹤。
* [F´ GitHub Discussions](https://github.com/nasa/fprime/discussions)：使用問題與社群討論。
* [FPP 文件](https://nasa.github.io/fpp/)：FPP 建模語言的介紹與使用指南。
* [F Prime cheatsheet](https://fprime.jpl.nasa.gov/cheatsheet.pdf)：F Prime 的小抄，說明一些基本概念。

初次接觸時，建議先完成官方文件中的 HelloWorld 教學，再進一步了解元件、連接埠、拓樸及 Ground Data System（GDS）。
