---
title: rmw_zenoh 進階介紹
description: rmw_zenoh 的進階功能如何使用，包含設定檔、debug、嵌入式等等
keywords:
  - Zenoh
  - ROS 2
  - 機器人
  - middleware
---

前面介紹完 rmw_zenoh 基本概念之後，接下來就是一些比較進階的概念了。

## 設定檔內容

首先最重要的是設定的部份 (Configuration)。
Zenoh 有兩種設定檔： Router 和 Session。
Router 代表的是給 Zenoh Router 使用的設定，而 Session 則是給一般 ROS Node 使用。

設定方式也非常簡單，分別使用 `ZENOH_ROUTER_CONFIG_URI` 和 `ZENOH_SESSION_CONFIG_URI` 這兩個環境變數即可。

```bash
# 給 Zenoh Router 使用
export ZENOH_ROUTER_CONFIG_URI=$HOME/MY_ZENOH_ROUTER_CONFIG.json5
# 給 ROS Node 使用
export ZENOH_SESSION_CONFIG_URI=$HOME/MY_ZENOH_SESSION_CONFIG.json5
```

他們預設的設定值，可以參考 rmw_zenoh_cpp 的 GitHub Repository。
複製一份，然後調整自己需要的樣子即可。

* [DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5](https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5)
* [DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5](https://github.com/ros2/rmw_zenoh/blob/rolling/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5)

當然，這樣的使用方式也是有點麻煩。
rmw_zenoh 也提供了 override 的覆蓋單純部份 config 方式，也就是利用 `ZENOH_CONFIG_OVERRIDE` 環境變數。
下面舉幾個例子：

* 要求 Zenoh Router 連線到指定的 IP

```bash
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.0.3:7447", "tcp/192.168.0.4:7447"]'
ros2 run rmw_zenoh_cpp rmw_zenohd
```

* 讓節點可以不用 Zenoh Router 彼此發現

```bash
export ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true'
ros2 run demo_nodes_cpp talker
```

如此一來，當使用者單純只是想修改設定檔中某個欄位而已，就不需要把整個設定檔都複製下來調整，只需要調整環境變數就好。

## 跨機連接

對機器人使用情境來說，最常使用的是從我們的筆電遠端控制機器人，代表著 rmw_zenoh 要跑在兩個不同機器上面。
然而預設 rmw_zenoh 限制流量只會在同台機器內，那這時候我們該怎麼設定呢？

這邊有兩個方法：

1. 兩台電腦都運行 Zenoh Router，然後讓兩者相連
    * 適用情境：兩台機器內部都有跑大量的 ROS application，只有部份流量需要兩者彼此互通
    * 設定方式：假設其中一台電腦 IP 是 192.168.1.1，我們可以在另一台電腦修改 Zenoh Router 的設定檔
        * 設定檔：

        ```json
        {
          connect: {
            endpoints: ["tcp/192.168.1.1:7447"],
          },
        }
        ```

        * `ZENOH_CONFIG_OVERRIDE`:

        ```bash
        export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.1.1:7447"]'
        ```

2. 一台電腦運行 client 模式，然後連接到另一台電腦的 Zenoh Router
    * 適用情境：基本上所有 ROS 流量都要彼此互通，例如，另一台電腦只有跑單一 ROS application，像是 RViz
    * 設定方式：假設跑 Zenoh Router 的電腦 IP 是 192.168.1.1，我們在另一台電腦修改 Session 的設定檔
        * 設定檔：

        ```json
        {
          mode: "client",
          ...
          connect: {
            endpoints: ["tcp/192.168.1.1:7447"],
          },
        }
        ```

        * `ZENOH_CONFIG_OVERRIDE`:

        ```bash
        export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.1.1:7447"]'
        ```

## Debug

Zenoh 本身是用 Rust 寫成，而 Rust 可以使用 `RUST_LOG` 來控制 log level 的顯示，也就是可以決定 log 的細節程度，例如 info、debug 等等。
因此如果使用上遇到問題懷疑是 Zenoh 本身的問題，可以打開 log 來觀察哪邊出錯。

* `export RUST_LOG=zenoh=info`：觀察最基礎的 Zenoh 資訊，包含初始化以及連接哪些節點。
* `export RUST_LOG=zenoh=debug`：顯示所有的 Zenoh debug 資訊，這些資訊相當豐富，但也會讓人眼花撩亂。
* `export RUST_LOG=zenoh=info,zenoh_transport=debug`：顯示最基本的 Zenoh 資訊，但是針對連線部份則是呈現更細緻的 debug 資訊。

`RUST_LOG` 在 Zenoh Router 或是一般的 ROS Node 都是可以使用的。

## Pico-ROS：rmw_zenoh 的嵌入式版本

在之前的 ROS 2 中，要跑 ROS 在嵌入式系統需要用到 [micro-ROS](https://micro.ros.org/)，然而這個只能用在 FastDDS 上。
在 rmw_zenoh 中，則有相對應的替代品 [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software)，主要是基於 Zenoh 的嵌入式版本 [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico)，直接與 rmw_zenoh 相連。

## rmw_zenoh 設計

這邊主要的介紹是提供給對 rmw_zenoh 如何實作有興趣的人參考。

### topic 和 key expression 對應

我們知道 Zenoh 是用 key expression 來串連不同資料流，而 ROS 2 則是 topic 的概念。
那麼他們兩者是怎麼對應的呢？
我們這邊可以參考 rmw_zenoh 的[設計細節](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#topic-and-service-name-mapping-to-zenoh-key-expressions)。

為了避免 ROS 2 topic 在 Zenoh 的實作上會遇到彼此相衝的問題，任何的 ROS 2 topic 都會轉為如下形式：

`<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>`

* `domain_id`：也就是大家熟知的 ROS_DOMAIN_ID 的值
* `fully_qualified_name`：符合 [ROS 2 topic/service 規範](https://design.ros2.org/articles/topic_and_service_names.html#fully-qualified-names)的名稱
* `type_name`：ROS 2 topic 的 message type
* `type_hash`：message type 的雜湊值，這是後來 ROS 2 為了避免 topic message type 進版時彼此還是可以相互通訊索引入的，相關定義參考 [REP-2016](https://github.com/ros-infrastructure/rep/pull/381/files)

讓我們舉些實際例子：

* 假設有個 ROS 2 topic `chatter`，是 String 格式，且 `ROS_DOMAIN_ID` 為 0：

```raw
0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
```

* 如果加上 namespace `/robot1`，`ROS_DOMAIN_ID` 變成 2：

```raw
2/robot1/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
```

### Graph Cache 設計

當我們使用 `ros2 topic` 或是 `ros2 node` 等指令時，我們需要有個方法知道當前 ROS 環境下的 topic 和 node 狀態。
而這個就是 Graph Cache 的功能，它可以幫助 ROS 系統知道所有當下拓樸的狀態。
在 Zenoh 中，我們可以利用 liveliness 這個機制來偵測各種 topic 和 node 的變化。
liveliness 和一般的 key expression 很類似，主要差別就是用來確保當某個節點消失時，其他節點也都還是可以發現。
當 `rmw_zenoh_cpp` 初始化 ROS context 的時候會會透過 liveliness 來取得整個系統狀態，而都有任何節離開或進入 ROS 系統，都會用 liveliness的的機制更新 Graph Cache。

* node 名稱如何對應到 Zenoh liveliness 的名稱

```raw
@ros2_lv/<domain_id>/<session_id>/<node_id>/<node_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>
```

* topic 名稱如何對應到 Zenoh liveliness 的名稱

```raw
@ros2_lv/<domain_id>/<session_id>/<node_id>/<entity_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>/<mangled_qualified_name>/<type_name>/<type_hash>/<qos>
```

說明一下每個欄位的細節：

* `<domain_id>`：對應到實際的 `ROS_DOMAIN_ID`
* `<session_id>`：Zenoh 的 session ID，每個 ROS context 都會對應到一個專門的 session ID
* `<node_id>`：這個 session 下的每個 node 都有獨一無二的 ID
* `<entity_id>`：在這個 node 下，每個 entity，包含 publisher、subscriber、service 等等都有獨一無二的 ID，如果是 node 自己，就填跟 `<node_id>` 一樣
* `<entity_kind>`：這個 entity 的類型，可以是如下欄位
    * `NN`： node
    * `MP`： message publisher
    * `MS`： message subscriber
    * `SS`： service server
    * `SC`： service client
* `<mangled_enclave>`：SROS enclave 名稱，如果沒有就填 `%`
* `<mangled_namespace>`： namespace 名稱，如果沒有就填 `%`
* `<node_name>`： node 名稱
* `<mangled_qualified_name>`：(node 不用填) topic 或是 service 的名稱
* `<type_name>`：(node 不用填) ROS 2 topic 的 message type
* `<type_hash>`：(node 不用填) message type 的雜湊值，這是後來 ROS 2 為了避免 topic message type 進版時彼此還是可以相互通訊索引入的，相關定義參考 [REP-2016](https://github.com/ros-infrastructure/rep/pull/381/files)
* `<qos>`：(node 不用填) 會從 ROS 2 的 QoS 屬性轉換過來，中間用 `:` 分隔，實際轉換方式可以參考 [qos_to_keyexpr](https://github.com/ros2/rmw_zenoh/blob/cf09e854c9df17e0eb7e80ce4ab00e1b122a64e0/rmw_zenoh_cpp/src/detail/liveliness_utils.cpp#L239)
    * 第一個欄位是 Reliability，沒有填代表 reliable，2 代表 best_effort
    * 第二個欄位是 Durability，沒填是 volatile，1 代表 transient_local
    * 第三個欄位是 History，用 `,` 分隔，前半部份沒填是 keep_last，2 代表 keep_all，後半部份代表 History depth 的數字，例如 `,7` 代表 keep_last 且 depth 為 7
    * 第四個欄位是 Deadline，用 `,` 分隔，前半部是 second，後半部是 nanosecond
    * 第五個欄位是 Lifespan，用 `,` 分隔，前半部是 second，後半部是 nanosecond
    * 第六個欄位是 Liveliness，用 `,` 分隔成三部份，第一部份沒填代表 automatic，2 代表 manual_by_topic，第二和第三部份則分別代表 second 和 nanosecond

舉實際例子

* 假設某個 Node 的 Domain ID 是 2，且 node 名稱是 listener，會宣告如下 liveliness

```raw
@ros2_lv/2/aac3178e146ba6f1fc6e6a4085e77f21/0/0/NN/%/%/listener
```

* 假設某個 topic 的 Domain ID 是 2，隸屬於 talker 這個 node，名稱為 chatter，且類型是 String

```raw
@ros2_lv/2/8b20917502ee955ac4476e0266340d5c/0/10/MP/%/%/talker/%chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,7:,:,:,,
```

更詳細資訊可以參考[官方設計文件](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md#graph-cache)。
