---
title: Zenoh Pico
description: Zenoh Pico 介紹
keywords:
  - Zenoh
---

zenoh-pico 是 Zenoh 的輕量級版本，主要用於嵌入式系統。
我們可以把 zenoh-pico 想成是 Zenoh 的用 C 寫成的輕量級實作，擁有大多數的功能，但是不支援共享記憶體 (Shared Memory)、Gossip (分享並接收連接狀態的資訊)。

## 編譯教學

以下我們使用 [zenoh-example/zenoh-pico](https://github.com/evshary/zenoh-example/tree/main/zenoh-pico) 作為範例，說明如何編譯並執行基於 Zenoh Pico 的 C 程式。

### 1. 編譯 zenoh-pico

首先，我們需要編譯 `zenoh-pico` 本身。先建立相對應的資料夾，並透過 CMake 設定編譯選項與安裝路徑：

```bash
mkdir -p zenoh-pico-build zenoh-pico-install
cd zenoh-pico-build
cmake ../zenoh-pico \
      -DCMAKE_INSTALL_PREFIX=../zenoh-pico-install \
      -DCMAKE_BUILD_TYPE=Release \
      -DZ_FEATURE_UNSTABLE_API=1 \
      -DZ_FEATURE_ADVANCED_PUBLICATION=1 \
      -DZ_FEATURE_ADVANCED_SUBSCRIPTION=1
make
make install
cd ..
```

*註：`Z_FEATURE_ADVANCED_PUBLICATION` 和 `Z_FEATURE_ADVANCED_SUBSCRIPTION` 需要開啟 `Z_FEATURE_UNSTABLE_API=1`。此範例所需的其他功能在 `zenoh-pico` 預設皆已開啟。*

### 2. 編譯範例程式

接著，編譯應用程式範例。建立 `build` 資料夾，並透過 `CMAKE_PREFIX_PATH` 指定剛剛安裝好的 `zenoh-pico` 路徑

```bash
mkdir -p build
cd build
cmake .. \
      -DCMAKE_PREFIX_PATH="$(pwd)/../zenoh-pico-install" \
      -DCMAKE_BUILD_TYPE=Release
make
cd ..
```

編譯成功後，會得到以下兩個執行檔：

* `zp_simple_pubsub`
* `zp_simple_advanced_pubsub`

### 3. 執行範例

```bash
# 先執行 zenohd
./build/zp_simple_pubsub
./build/zp_simple_advanced_pubsub
```

### 4. 清理環境

如果需要清除編譯的暫存檔與安裝資料夾，可以執行：

```bash
rm -rf build zenoh-pico-build zenoh-pico-install
```
