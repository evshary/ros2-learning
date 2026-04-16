---
title: Zenoh C++
description: 介紹 Zenoh C++ 的 API 用法
keywords:
  - Zenoh
---

C++ 算是最熱門的程式語言之一，所以 Zenoh 也提供 zenoh-cpp 讓使用者可以透過 C++ API 來使用 Zenoh。

zenoh-cpp 的設計比較特別，他其實只是一群 header 檔案，後面可以接上 zenoh-c (完整版的 Zenoh) 或是 zenoh-pico (嵌入式) 的實作。

## 範例

我創了一個簡單的專案 [zenoh-cpp-example](https://github.com/evshary/zenoh-cpp-example) 來展示怎麼在你的程式中使用 zenoh-cpp。

* 首先我們抓下這個專案，並且同步裡面的 submodule

```bash
git clone https://github.com/evshary/zenoh-cpp-example
cd zenoh-cpp-example
```

* 安裝 library

```bash
sudo apt install libzenohc-dev
sudo apt install libzenohpico-dev
sudo apt install libzenohcpp-dev
```

* 執行 cmake

```bash
mkdir -p build && cd build
cmake -DBUILD_ZENOHC=ON \
      -DBUILD_ZENOHCPP_ZENOHC=ON \
      -DBUILD_ZENOHCPP_ZENOHPICO=ON ..
cmake --build .
```

這邊有三個選項 `BUILD_ZENOHC`、`BUILD_ZENOHCPP_ZENOHC`、`BUILD_ZENOHCPP_ZENOHPICO`，可以去 `CMakeLists.txt` 查看分別對應的編譯選項。

* `BUILD_ZENOHC`：代表編譯 zenoh-c 的 API 的範例
* `BUILD_ZENOHCPP_ZENOHC`：代表編譯 zenoh-cpp 的 API 範例，但是用 zenoh-c 當 backend
* `BUILD_ZENOHCPP_ZENOHPICO`：代表編譯 zenoh-cpp 的 API 範例，但是用 zenoh-pico 當 backend
