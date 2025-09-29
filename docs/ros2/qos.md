---
title: ROS 2 QoS
description: ROS 2 基本的 QoS 支援
keywords:
  - ROS 2
  - 機器人
  - QoS
---

ROS 2 通訊層使用 DDS 的一大原因就是 DDS 有支援 QoS (Quality of Service)。
QoS 能夠指定通訊相關的設定，像是資料傳輸的可靠性、是否要保留歷史資料等等。
然而 DDS 所支援的 QoS 實在太多了，從官方規範就有 20 幾種，因此 ROS 2 只選擇跟機器人比較相關的其中六種出來：

| QoS         | 功能 |
| ----------- | ---- |
| Reliability | 用來確認通訊的可靠性，有 best effort 和 reliable 兩種 |
| History     | 能夠保留多少通訊的歷史資料，可以選擇 keep_last N 或 keep_all |
| Durability  | 是否要將歷史資料提供給 late-joiner，可以選擇 volatile 或 transient_local |
| Liveliness  | 主動確認 node 是否存活 |
| Lifespan    | 設定資料的存活時間，如果超過時間未被收取將會捨棄，並且跳出警示 |
| Deadline    | 在連續的資料間，最多可以有多長的間隔時間 |

一個很重要的點是，各個 QoS 本身上是獨立運行不會受到其他人影響，舉例來說，reliability 的設定不會影響到 durability，甚至其他 QoS 的行為。
因此我們在看待這六個 QoS 的時候，可以當成六種可以調整的 config 來看帶。

官方有針對 Liveliness, Lifespan 和 Deadline 提供 [ROS 2 demo](https://github.com/ros2/demos/tree/master/quality_of_service_demo)，如果想要實際了解其中內容，可以直接操作看看。

## RxO (Request vs Offered)

Publisher 和 Subscriber 要能夠通訊，兩者的 QoS 必須要先 compatible，也就是符合 RxO 才行。
一言以蔽之，RxO 就是 Publisher 的 QoS 設定必須要高於 Subscriber 才行。
舉個例子來說，Reliability 的 QoS 要能夠通訊，Publisher 和 Subscriber 必須要符合如下邏輯。

|  Publisher  | Subscriber  | Compatible |
| ----------- | ----------- | ---------- |
| Best Effort | Best Effort |      V     |
| Best Effort |   Reliable  |      X     |
|   Reliable  | Best Effort |      V     |
|   Reliable  |   Reliable  |      V     |

由於定義上 Reliable > Best Effort，所以如果 Publisher 是 Best Effort，Subscriber 是 Reliable，兩者就會跳出 incompatible QoS 的警告，並且無法通訊。

看到這邊一定會有個問題：要怎麼知道哪些設定是高，哪些設定是低？
除了靠直覺外 (Reliable 比起 Best Effort 來說要求更高)，可以查詢 [ROS 2 的 QoS Design Compatibility](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/#qos-compatibilities)，或是直接去查詢 DDS 的說明，[OpenSplice DDS 的文件](http://download.prismtech.com/docs/Vortex/apis/ospl/isocpp2/html/a02530.html)就有詳細敘述。

## 實際範例

我放了一些範例在[GitHub](https://github.com/evshary/ROS2_cheatsheet/tree/master/10.QoS)上，可以當作參考。

### C++

C++ 的 QoS API 可以參考 [rclcpp::QoS Class Reference](http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1QoS.html)

基本上作法是先 init QoS，然後在 create publisher / subscriber 的時候把 QoS 帶入即可。

* history, reliability, durability

這三者的設定還蠻容易的，所以就放一起來表示。

```cpp
// 先 init QoS 的物件
rclcpp::QoS qos_settings(10);
// 如果要設定 keep_last 10, reliable, volatile
qos_settings.keep_last(10).reliable().durability_volatile();
// 如果要設定 keep_all, best_effort, transient local
qos_settings.keep_all().best_effort().transient_local();
// 然後再將 QoS 在 create publisher 時帶入即可
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos_settings);
```

* liveliness

可參考 [官方 liveliness demo](https://github.com/ros2/demos/blob/master/quality_of_service_demo/rclcpp/src/liveliness.cpp)

```cpp
qos_settings.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
qos_settings.liveliness_lease_duration(std::chrono::milliseconds(2000));
rclcpp::SubscriptionOptions sub_options;
sub_options.event_callbacks.liveliness_callback =
  [](rclcpp::QOSLivelinessChangedInfo & event)
  {
    printf("Liveliness changed event: \n");
    printf("  alive_count: %d\n", event.alive_count);
    printf("  not_alive_count: %d\n", event.not_alive_count);
    printf("  alive_count_change: %d\n", event.alive_count_change);
    printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
  };
```

* lifespan

可參考 [官方 lifespan demo](https://github.com/ros2/demos/blob/master/quality_of_service_demo/rclcpp/src/lifespan.cpp)

```cpp
qos_settings.lifespan(std::chrono::milliseconds(2000));
```

* deadline

可參考 [官方 deadline demo](https://github.com/ros2/demos/blob/master/quality_of_service_demo/rclcpp/src/deadline.cpp)

```cpp
qos_settings.deadline(std::chrono::milliseconds(2000));
// publisher callback
rclcpp::PublisherOptions pub_options;
pub_options.event_callbacks.deadline_callback =
  [](rclcpp::QOSDeadlineOfferedInfo & event) -> void
  {
    printf("Offered deadline missed: total %d delta %d\n", event.total_count, event.total_count_change);
  };
auto publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos_settings, pub_options);
// subscriber callback
rclcpp::SubscriptionOptions sub_options;
sub_options.event_callbacks.deadline_callback =
  [](rclcpp::QOSDeadlineRequestedInfo & event) -> void
  {
    printf("Requested deadline missed: total %d delta %d\n", event.total_count, event.total_count_change);
  };
auto subscription_ = this->create_subscription<std_msgs::msg::String>("topic", qos_settings, std::bind(&MinimalSubscriber::topic_callback, this, _1), sub_options);
```

### Python

Python 的 QoS API 可以參考 [rclpy Quality of Service](http://docs.ros2.org/latest/api/rclpy/api/qos.html)

## 官方建議設定

官方有些建議的 QoS 設定，可參考 [qos_profiles.h](https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h)
舉個例子來說，sensor data 並不在乎資料的 reliable，所以會設定為 best effort，盡可能快速傳資料。
過去 sensor data 的歷史資料也不重要，所以 durability 的部分用 volatile 即可。

## Reference

* [ROS 2 wiki about QoS](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)
* [ROS 2 對 QoS 的設計理念](https://design.ros2.org/articles/qos.html)
* [ROS 2 對 deadline, liveliness, lifespan 的使用](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
