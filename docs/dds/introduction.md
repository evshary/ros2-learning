# 簡介

DDS (Data Distribution Service) 是一種 pub-sub 的通訊規格，希望可以達到可靠、高效能、安全、即時、可擴展、並且以資料為導向的傳輸。為了達成這個目的，DDS 有如下特色功能：

* 以資料為導向：過去的傳輸重視的是「誰連到誰」，所以我們會需要明確知道通訊者的 IP 位址並且開 socket 進行連接。DDS 重視的是「傳什麼資料」，任何人發佈的資料都會指定 topic，有興趣的人可以自行訂閱，而不需要管這資料是誰傳來的。
* 去中心化架構：DDS 並非以往 server-client 的架構，每個通訊節點都是對等的。因此任何人都可以隨意加入或離開通訊網路，而不會有任何通訊上的問題。
* 豐富的 QoS：為了達到可靠、高效能，DDS 有非常多的 QoS(Quality of Service)，使用者可以依照自己想要的通訊品質來進行調整，例如reliability, durability, transport priority等等。

[DDS 的規格](https://www.omg.org/omg-dds-portal/)制定是由 OMG (Object Management Group) 這個組織所負責，OMG 其實也有制定不少跟程式設計師有關的規格，舉例來說 Object Oriented (物件導向)常用到的 UML，就是由 OMG 所制定。

在 [DDS foundation](https://www.dds-foundation.org/omg-dds-standard/) 裡面有許多關於 DDS 的文件，其中比較值得注意的是下面這五份：

* DDS v1.4: 定義 DCPS，主要是 API 的制定
* DDSI-RTPS v2.3: 定義 DDSI，定義 DDS 的行為
* DDS-XTypes v1.2: 用在 topic 的 data type 規範
* DDS-Security v1.1: 定義 security 的規範
* Interface Definition Language (IDL) v4.2: IDL 的定義規範

DDS 最初是用在軍事、航太上，已經有悠久的歷史了，後來 ROS 2 在尋找可利用的通訊層時也看上 DDS 的可靠穩定，所以 DDS 也開始被應用在機器人領域。

DDS 目前有很多種選擇：

* CycloneDDS：ADLINK (後來 spin-off 出去叫做 ZettaScale) 提出的開源版輕量化 DDS。ROS 2 的 Galactic 版本是以 CycloneDDS 為預設通訊層。
* OpenSplice DDS：原本是 Prismtech 的主力產品(後來被 ADLINK 買下後又 spin-off 出去成為 ZettaScale)，有分開源版和商業版。
* FastDDS：Eprosima 的 DDS 開源解決方案，跟 CycloneDDS 一樣被應用在 ROS 2 上。
* Connext DDS：RTI 這家公司所提出的 DDS，並沒有開源。
* OpenDDS：Open source 版本的 DDS。

目前個人推薦的是使用 CycloneDDS，輕量且效能高，主要程式是用 C 撰寫而成。下面會稍微介紹一下 CycloneDDS。

## CycloneDDS

如前面所提，[CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) 是 ZettaScale 提出開源解決方案，並且目前託管於 [Eclipse Foundation 底下](https://projects.eclipse.org/projects/iot.cyclonedds)。

### 文件

* [官方文件](https://cyclonedds.io/docs/cyclonedds/latest/)
* [C API](https://cyclonedds.io/docs/cyclonedds/latest/ddsc.html)
* [C++ API](https://cyclonedds.io/docs/cyclonedds-cxx/latest/)
* [Python API](https://cyclonedds.io/docs/cyclonedds-python/latest/)

### 編譯

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

### 設定環境變數

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
