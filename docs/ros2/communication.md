# ROS 2 通訊方式

ROS 2 承接 ROS 1，同樣有三種常用的傳輸方式：

* topic
* service
* action

接著會依序介紹三者的特色與差異

## topic

ROS 中最基本的通訊模式就是 topic。發佈者(publisher)會指定要把資料丟給某個 topic，需要這個資料的訂閱者(subscriber)則是透過訂閱 topic 來接收資料。topic 的概念其實跟我們訂閱雜誌很像，出版社(publisher)會發行雜誌(也就是 topic)，而對這個雜誌有興趣的讀者(subscriber)會去訂閱，這樣當出版社發售新的更新時，就會把雜誌送給各個有訂閱的讀者來閱讀。

為什麼要設計 topic 這樣的通訊方式呢？因為透過 topic 的方式可以簡化通訊的概念，publisher 和 subscriber 不需要知道彼此的狀況，他們只需要關心 topic 即可。如果沒有 topic 的機制，傳送端就需要知道現在有多少接收端，而且當新增或減少接收端時，傳送端也都還需要做對應的處理，整體的通訊會變得十分複雜。對我們最重要的其實不是傳送端或接收端，而是資料本身，topic 的機制就可以讓開發者專注於資料本身的交換，而不是在通訊上。

![Topic-MultiplePublisherandMultipleSubscriber](images/Topic-MultiplePublisherandMultipleSubscriber.gif)

由上面 ROS 2 官方教學的動畫我們可以知道 topic 有如下的特色：

* 不論有多少 publisher / subscriber，都可以用 topic 來做到資料的交換
* publisher 只需要發送一次資料就可以同時讓多個 subscriber 收到，不需要重複發送多次
* topic 可以分開 publisher / subscriber，讓兩者專注於資料的處理上，而非通訊上

基本上在 ROS 2 中的 topic 觀察方法已經在 ROS 2 指令一章有所提及，這邊就不在重複了。

## service

ROS 2 中 service 的概念類似我們熟知的 client / server 架構。Service server 會提供某種功能的服務，當 service client 需要使用的時候就會發送 request 並等待 server，server 接收到並進行處理後，就會把結果回傳給 client (response)。

Service 能夠補足 topic 的一些不足。舉例來說，當涉及到資料處理而不僅僅是資料傳輸時，我們需要知道資料的處理結果如何，但 topic 的設計只有單向並沒有雙向傳輸，所以我們無法得知接收端處理的狀態，這時就可以用 service 解決這個問題。

![Service-MultipleServiceClient](images/Service-MultipleServiceClient.gif)

上面 ROS 2 官方教學的動畫傳達幾個 service 的特色：

* Service server 只能有一個，但是可以提供 service 給多個 service client
* 使用 service 需要定義 request 和 response，也就是 client 和 server 相互溝通的格式定義

這邊簡單介紹跟 ROS 2 service 相關的操作：

* 首先我們先跑個 service 的 example

```bash
ros2 run examples_rclcpp_minimal_service service_main
```

* 接著可以列出目前有提供的 service，你可以看到會列出多個可使用的 service

```bash
$ ros2 service list
/add_two_ints
/minimal_service/describe_parameters
/minimal_service/get_parameter_types
/minimal_service/get_parameters
/minimal_service/list_parameters
/minimal_service/set_parameters
/minimal_service/set_parameters_atomically
```

* 我們可以用 `ros2 service call` 直接發 request 給 `/add_two_ints` 這個 service
    * 這邊要特別注意我們需要知道 service type，基本上你可以用 tab 來讓 ROS 2 幫你自動補全

```bash
# 指令格式： ros2 service call <service name> <service type> <request>
$ ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 5}"
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=5)

response:
example_interfaces.srv.AddTwoInts_Response(sum=10)
```

## action

最後一個通訊方式是 action，一定會有人好奇為什麼已經有 topic / service，還要 action 做什麼。其實 action 是要強化 service 的不足之處。service 雖然可以處理 request / response 的通訊型態，但如果 server 需要很長時間的處理和運算，client 將需要花很長的時間等待，而且也無法知道目前的處理狀態。而且假設 client 又收到了新的 request 要把舊的覆蓋掉，他也必須等 server 回應後才能發下一次的 request，非常沒效率。

action 為了解決這個問題，除了 request / response 外，還多提供了 feedback 的功能，action client 可以在 action server 處理的過程中去詢問目前處理狀態，甚至可以取消當前的 request，變得更加彈性。這樣的通訊方法對一些需要花長時間處理的機器人應用非常實用，舉例來說 AMR 不可能一瞬間從 A 點到 B 點，所以當 action server 在導航 A 到 B 時，client 可以知道現在 AMR 的狀態，如果有接收到新的目標 C 點，client 也可以立即取消 A 到 B 的 request ，改成往 C 點。

![Action-SingleActionClient](images/Action-SingleActionClient.gif)

從官方的教學動畫，一樣可以整理出幾個重點：

* Action 的設計結構比 service 還要複雜，分為 Goal Service, Feedback Topic, Result Service
* Goal Service 觸發 action server 開始運作，接著再從 Result Service 取得得結果，而等待結果的過程，可以用 Feedback Topic 來得知目前執行狀態
* 上述這些細節都被隱藏在 action 中，我們無法直接看到組成 action 的 topic 和 service

這邊用 ROS 2 的 action example 來講解一下操作方法：

* 先跑一個 Fibonacci 的 action server

```bash
ros2 run examples_rclcpp_minimal_action_server action_server_member_functions
```

* 列出目前有的 action，可以看到 /fibonacci

```bash
$ ros2 action list
/fibonacci
```

* 看看 /fibonacci 這個 action 裡面有幾個 server 和 client

```bash
$ ros2 action info /fibonacci
Action: /fibonacci
Action clients: 0
Action servers: 1
    /minimal_action_server
```

* 看 action 的格式是什麼

```bash
$ ros2 action show action_tutorials/action/Fibonacci
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

* 直接對 action server 發送 request
    * `--feedback` 代表要觀察目前計算狀況

```bash
# 指令格式： ros2 action send_goal <action name> <action type> <request> [--feedback]
$ ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 20}" --feedback
Waiting for an action server to become available...
Sending goal:
     order: 20

Goal accepted with ID: 9130b8fa240740d5866445719d9b6cb2

Feedback:
    sequence:
- 0
- 1
- 1
...
```

## 總結

這邊簡單總結一下 ROS 2 幾種通訊方式的比較：

| | 描述 | 方向 | 同步 | 應用範例 |
| - | - | - | - | - |
| topic | 不間斷傳送資料對接收端 | 單向 | 非同步 | 機器人現在狀態、感測器資料 |
| service | 發送 request 並等待處理結果 | 雙向 | 同步 | 觸發狀態變化、事件 |
| action | 發送 request 但不等待，可以得知或中斷處理過程 | 雙向 | 非同步 | 命令機器人導航、機器手臂移動 |

三種通訊方式都有自己適合的應用場景，並沒有誰優誰劣，使用者必須自行判斷什麼場景適合使用哪種通訊方式才行。

## 參考資料

* [Understanding ROS 2 topics](https://index.ros.org/doc/ros2/Tutorials/Topics/Understanding-ROS2-Topics/)
* [Understanding ROS 2 services](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)
* [Understanding ROS 2 actions](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/)
