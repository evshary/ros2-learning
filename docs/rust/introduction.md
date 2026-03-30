---
title: Rust 介紹
description: 簡介 Rust，並列出常用的 Rust 資源
keywords:
  - Rust
---

Rust 是一門越來越熱門的程式語言，因為他語法的設計，特別有安全性，可以避免很多常犯的 memory 問題。
Rust 目前應用的領域有各類系統程式、網路協定設計、區塊鏈合約等等。
而在車用領域，也開開始有很多廠商為了 safety 問題而開始採用 Rust。他們相信使用 Rust 可以寫出更加安全的程式，進而避免安全性的各種問題。
因此這邊也列出一些跟 Rust 相關的資源。

## 常用資源

* [Rust playground](https://play.rust-lang.org/): 線上 compiler，可以用來測試 Rust 語法
* [Compiler Explorer](https://rust.godbolt.org/): 可以用來觀察 Rust 編譯成組合語言會長怎麼樣
* [Rust Community Crate](https://crates.io/): 社群所開發的套件會放在這邊

## 教學

### Rust 本身

* The Rust Programming Language: 官方文件
    * [英文](https://doc.rust-lang.org/book/?search=)
    * [中文](https://rust-lang.tw/book-tw/)
* [Rust语言圣经(Rust Course)](https://course.rs/about-book.html): 中國人寫的教學
    * [Rust By Practice( Rust 练习实践 )](https://zh.practice.rs/why-exercise.html): 附上的練習題
* Rust by Example
    * [英文](https://doc.rust-lang.org/rust-by-example/)
    * [中文](https://rustwiki.org/zh-CN/rust-by-example/index.html)
* [rustlings](https://github.com/rust-lang/rustlings): 有些 Rust 的練習可以確保知道基本概念
* [Rust学习笔记](https://skyao.io/learning-rust/): 這個有點像查詢的資料庫

### 影片教學

* [All Rust features explained](https://youtu.be/784JWR4oxOI): Rust 和其他語言不同的特色

### async

* Asynchronous Programming in Rust: 官方文件
    * [英文](https://rust-lang.github.io/async-book/)
    * [中文](https://huangjj27.github.io/async-book/)
    * [Rust语言圣经 - Rust 异步编程](https://course.rs/async-rust/intro.html): 含官方文件和 tokio 教學，但改寫得比較容易懂
* tokio
    * [Tokio - Async in depth](https://tokio.rs/tokio/tutorial/async)
    * [Rust入门秘籍 - tokio简介](https://rust-book.junmajinlong.com/ch100/00.html)
    * [Tokio 內部機制：從頭理解 Rust 非同步 I/O 框架](https://gist.github.com/weihanglo/4661db374f82fe91e931bab0f50d7a10)
* async 內部分析 (FSM logic)
    * [async/await 如何工作 | Rust学习笔记](https://segmentfault.com/a/1190000024540040)
* async 比較易懂的教學:
    * [Rust Async 1- 異步編程的概念](https://www.readfog.com/a/1666736268025892864)
    * [Rust Async 2- 同步、多線程、異步的例子](https://www.readfog.com/a/1666818509749981184)
    * [Rust Async 3- 繼續理解 Async](https://www.readfog.com/a/1667010704884994048)
    * [Rust Async 4- 理解 Future](https://www.readfog.com/a/1667010728788332544)
    * [Rust Async 5. 最后一个例子](https://mp.weixin.qq.com/s?__biz=MzIxODY5Mzc4Mg==&mid=2247485328&idx=1&sn=87e20836cdb8de10d2de8e5db0fde598&chksm=97e7ee69a090677f262035e3ab1882c9d8bf4c7259d7f5fcc56fcb3fc8b7f6069d631c71f14a&scene=178&cur_album_id=2364704891711045633#rd)
    * [Hello Rust async/await](http://liubin.org/blog/2021/03/25/hello-rust-async/)

### Low-Level Concurrency

* [Rust Atomics and Locks: Low-Level Concurrency in Practice](https://marabos.nl/atomics/): 講解 Rust 底層的 concurrency 如何運作
