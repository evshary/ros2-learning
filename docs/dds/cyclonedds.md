---
title: CycloneDDS
description: 介紹 CycloneDDS 的使用方法
keywords:
  - DDS
  - CycloneDDS
---

[CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) 是 ZettaScale 提出開源解決方案，並且目前託管於 [Eclipse Foundation 底下](https://projects.eclipse.org/projects/iot.cyclonedds)。

## 文件

* [官方文件](https://cyclonedds.io/docs/cyclonedds/latest/)
* [C API](https://cyclonedds.io/docs/cyclonedds/latest/ddsc.html)
* [C++ API](https://cyclonedds.io/docs/cyclonedds-cxx/latest/)
* [Python API](https://cyclonedds.io/docs/cyclonedds-python/latest/)

## 編譯

這邊簡單描述編譯 CycloneDDS 方法，這邊使用的環境是 Ubuntu。

* 安裝相依性

```bash
sudo apt install bison cmake
```

* 從 GitHub 抓程式碼

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
```

* 開始編譯

```bash
mkdir build
cd build
# 要把 example 編譯出來
cmake -DBUILD_IDLC=ON -DBUILD_EXAMPLES=ON ..
cmake --build .
cd ..
```

* 完成編譯後可以跑個 Hello World 來看看

```bash
# Publisher
build/bin/HelloworldPublisher
# Subscriber (另一個 terminal)
build/bin/HelloworldSubscriber
```

* 可以看到 publisher 和 subscriber 可以互相通訊了

## 設定環境變數

當然 CycloneDDS 也是可以做一些簡易環境設定，只要設定好 `CYCLONEDDS_URI` 這個環境變數就可以了。這邊我們以 GitHub 上的設定檔為範例。

* 新創 cyclonedds.xml 的檔案

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>default</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
            <FragmentSize>4000B</FragmentSize>
        </General>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
        <Tracing>
            <Verbosity>config</Verbosity>
            <OutputFile>stdout</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```

* 設定環境變數

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

* 接著重跑 CycloneDDS 的程式就會自動使用新的 config 了。關於更多 config 可以怎麼設定，可以參考 [CycloneDDS 的文件](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)

* 另外也有偷吃步的直接改 config 的方法，假設我們要改 DDS 的 domain ID

```bash
CYCLONEDDS_URI="<CycloneDDS><Domain><Id>1</Id></Domain></CycloneDDS>" build/bin/HelloworldPublisher
```
