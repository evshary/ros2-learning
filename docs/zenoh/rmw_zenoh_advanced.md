# rmw_zenoh 進階介紹

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
