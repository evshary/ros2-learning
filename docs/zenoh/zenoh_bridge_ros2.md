---
title: zenoh-bridge-ros2dds：橋接 ROS 2
description: 如何利用 zenoh-bridge-ros2dds 來橋接 Zenoh 和 ROS 2 訊息
keywords:
  - Zenoh
  - ROS 2
  - 機器人
---

對於部份 ROS 2 使用者來說，可能有興趣使用 Zenoh 的各種功能，但是不想動到目前既有系統，這時候就需要能夠有個方法能轉換 ROS 2 和 Zenoh 的訊息。
為此，Zenoh 提供 [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) 來橋接兩種不同協定。

## 使用情境

那到底哪些應用情境會需要使用 `zenoh-bridge-ros2dds` 呢？

1. 跨機通訊

    最常見的使用情境是跨網通訊，ROS 2 使用的 DDS 協定基本上需要多播 (multicast) 來發現其他節點，而多播預設不能在 4G/5G 環境通訊，更無法連到網際網路。
    因此假設使用者要控制的機器人不在同個網域，例如從雲端控制地端的機器人，就無法使用 DDS，需要別的協定來達成。
    在這種情況下，`zenoh-bridge-ros2dds` 可以幫我們轉換 ROS 2 的訊息到 Zenoh，來避免這個問題。

2. 多機控制

    一般來說如果我們有多台機器人同時運行，因為運行同樣的會軟體，所以會遇到有 topic 衝突的問題。
    我們可以先限定 ROS 流量只能在機器人內部運行，然後機器人外則是使用 Zenoh 互相交流訊息。
    兩者之間的轉換就是透過 `zenoh-bridge-ros2dds` 來橋接。
    值得注意的是，我們還可以指定[不同 namespace 給不同機器人](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds?tab=readme-ov-file#easy-multi-robots-via-namespace-configuration)，如此一來就可以指定哪個訊息要傳給特定機器人了。

3. 橋接不同的 ROS Domain

    ROS 使用者常常會用 ROS domain 來避免不同的 ROS 訊息互相混雜。
    不過如果有些時候我們需要部份訊息能穿透到其他 ROS domain 時，`zenoh-bridge-ros2dds` 就可以幫上忙。
    `zenoh-bridge-ros2dds` 有[支援 Allow / Deny list](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/aa75eb007a0ff9c6b882233d0ee33420446e76b2/DEFAULT_CONFIG.json5#L69)，決定只有哪些訊息可以被轉換成 Zenoh 訊息。

## 如何安裝

`zenoh-bridge-ros2dds` 提供了三種安裝的方式，分別是 apt 安裝、docker 安裝、原始碼編譯。

### apt 安裝

如果是使用 Ubuntu 的系統，Zenoh 官方有提供 Debian package 可以直接安裝，這是最方便的安裝方式。

```bash
# 新增 Eclipse Zenoh 的 public key 到 apt keyring
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
sudo apt update
# 安裝 zenoh-bridge-ros2dds
sudo apt install zenoh-bridge-ros2dds
```

### docker 安裝

如果你的環境是以 container 為主的話，直接使用官方提供的 container 會更為方便。

```bash
docker pull eclipse/zenoh-bridge-ros2dds:latest
```

### 原始碼編譯

如果上述都不適合你，或是想要使用最新版本的 `zenoh-bridge-ros2dds`，那也可以選擇直接編譯。
在安裝完 Rust、`llvm`、`clang`以後，執行編譯指令即可。

```bash
git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
cd zenoh-plugin-ros2dds
cargo build --release
```

編譯完成後，你可以在 `./target/release` 找到 `zenoh-bridge-ros2dds` 的執行檔。

## 使用方法

`zenoh-bridge-ros2dds` 的使用方法也相當的簡單，只需要在你的機器人上直接執行即可。

```bash
zenoh-bridge-ros2dds
```

當然 `zenoh-bridge-ros2dds` 有更多好用的功能：

* namespace：所有從 ROS 傳到 Zenoh 的訊息都會加上 namespace，而從 Zenoh 傳到 ROS 的訊息則會被去掉 namespace

```bash
# 如果 ROS topic 是 mytopic，在 Zenoh 的角度看到的訊息會是 mytopic/myrobot
zenoh-bridge-ros2dds -n myrobot
```

* domain：在 ROS 的部份只會接收某個 ROS_DOMAIN_ID 的訊息，預設是 0

```bash
# ROS 部份使用的 ROS_DOMAIN_ID 為 1
zenoh-bridge-ros2dds -d 1
```

* 允許使用 REST API 來看目前的轉換狀態，如有哪些 node 等基本資訊
    * 在別的 terminal 用 curl 抓資料，可以用 jq 來讓 JSON 好看一點
    * 下面範例我們跑一個 ROS 2 talker 來觀察可以取得哪些資訊

```bash
# 在 8000 port 開啟 REST API
zenoh-bridge-ros2dds --rest-http-port 8000
```

<details>
  <summary>REST API：抓目前看到的 ROS Node</summary>

```bash
$ curl http://localhost:8000/@/\*/ros2/node/\*\* | jq
[
  {
    "key": "@/5a7c2bf14f92a2afccb58c7c5c6e59d6/ros2/node/0110bcec8211002a8dbc06f7000001c1/talker",
    "value": {
      "action_clients": [],
      "action_servers": [],
      "publishers": [
        {
          "name": "/rosout",
          "type": "rcl_interfaces/msg/Log"
        },
        {
          "name": "/chatter",
          "type": "std_msgs/msg/String"
        },
        {
          "name": "/parameter_events",
          "type": "rcl_interfaces/msg/ParameterEvent"
        }
      ],
      "service_clients": [],
      "service_servers": [
        {
          "name": "/talker/set_parameters",
          "type": "rcl_interfaces/srv/SetParameters"
        },
        {
          "name": "/talker/set_parameters_atomically",
          "type": "rcl_interfaces/srv/SetParametersAtomically"
        },
        {
          "name": "/talker/describe_parameters",
          "type": "rcl_interfaces/srv/DescribeParameters"
        },
        {
          "name": "/talker/list_parameters",
          "type": "rcl_interfaces/srv/ListParameters"
        },
        {
          "name": "/talker/get_parameters",
          "type": "rcl_interfaces/srv/GetParameters"
        },
        {
          "name": "/talker/get_type_description",
          "type": "type_description_interfaces/srv/GetTypeDescription"
        },
        {
          "name": "/talker/get_parameter_types",
          "type": "rcl_interfaces/srv/GetParameterTypes"
        }
      ],
      "subscribers": []
    },
    "encoding": "application/json",
    "timestamp": null
  }
]
```

</details>

<details>
  <summary>REST API：Zenoh 和 ROS 如何互相轉換</summary>

```bash
$ curl http://localhost:8000/@/\*/ros2/route/\*\* | jq
[
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/topic/pub/chatter",
    "value": {
      "dds_reader": "",
      "local_nodes": [
        "/talker"
      ],
      "priority": 5,
      "publication_cache_size": 0,
      "remote_routes": [],
      "ros2_name": "/chatter",
      "ros2_type": "std_msgs/msg/String",
      "zenoh_key_expr": "chatter"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/service/srv/talker/get_parameter_types",
    "value": {
      "is_active": true,
      "local_nodes": [
        "/talker"
      ],
      "remote_routes": [],
      "rep_reader": "0110ffcc8892357cf4b3605500000d04",
      "req_writer": "0110ffcc8892357cf4b3605500000c03",
      "ros2_name": "/talker/get_parameter_types",
      "ros2_type": "rcl_interfaces/srv/GetParameterTypes",
      "zenoh_key_expr": "talker/get_parameter_types"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/service/srv/talker/set_parameters_atomically",
    "value": {
      "is_active": true,
      "local_nodes": [
        "/talker"
      ],
      "remote_routes": [],
      "rep_reader": "0110ffcc8892357cf4b3605500000b04",
      "req_writer": "0110ffcc8892357cf4b3605500000a03",
      "ros2_name": "/talker/set_parameters_atomically",
      "ros2_type": "rcl_interfaces/srv/SetParametersAtomically",
      "zenoh_key_expr": "talker/set_parameters_atomically"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/service/srv/talker/get_type_description",
    "value": {
      "is_active": true,
      "local_nodes": [
        "/talker"
      ],
      "remote_routes": [],
      "rep_reader": "0110ffcc8892357cf4b3605500000704",
      "req_writer": "0110ffcc8892357cf4b3605500000603",
      "ros2_name": "/talker/get_type_description",
      "ros2_type": "type_description_interfaces/srv/GetTypeDescription",
      "zenoh_key_expr": "talker/get_type_description"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/topic/pub/rosout",
    "value": {
      "dds_reader": "",
      "local_nodes": [
        "/talker"
      ],
      "priority": 5,
      "publication_cache_size": 10000,
      "remote_routes": [],
      "ros2_name": "/rosout",
      "ros2_type": "rcl_interfaces/msg/Log",
      "zenoh_key_expr": "rosout"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/service/srv/talker/set_parameters",
    "value": {
      "is_active": true,
      "local_nodes": [
        "/talker"
      ],
      "remote_routes": [],
      "rep_reader": "0110ffcc8892357cf4b3605500000904",
      "req_writer": "0110ffcc8892357cf4b3605500000803",
      "ros2_name": "/talker/set_parameters",
      "ros2_type": "rcl_interfaces/srv/SetParameters",
      "zenoh_key_expr": "talker/set_parameters"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/service/srv/talker/describe_parameters",
    "value": {
      "is_active": true,
      "local_nodes": [
        "/talker"
      ],
      "remote_routes": [],
      "rep_reader": "0110ffcc8892357cf4b3605500000f04",
      "req_writer": "0110ffcc8892357cf4b3605500000e03",
      "ros2_name": "/talker/describe_parameters",
      "ros2_type": "rcl_interfaces/srv/DescribeParameters",
      "zenoh_key_expr": "talker/describe_parameters"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/topic/pub/parameter_events",
    "value": {
      "dds_reader": "",
      "local_nodes": [
        "/talker"
      ],
      "priority": 5,
      "publication_cache_size": 0,
      "remote_routes": [],
      "ros2_name": "/parameter_events",
      "ros2_type": "rcl_interfaces/msg/ParameterEvent",
      "zenoh_key_expr": "parameter_events"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/service/srv/talker/list_parameters",
    "value": {
      "is_active": true,
      "local_nodes": [
        "/talker"
      ],
      "remote_routes": [],
      "rep_reader": "0110ffcc8892357cf4b3605500001104",
      "req_writer": "0110ffcc8892357cf4b3605500001003",
      "ros2_name": "/talker/list_parameters",
      "ros2_type": "rcl_interfaces/srv/ListParameters",
      "zenoh_key_expr": "talker/list_parameters"
    },
    "encoding": "application/json",
    "timestamp": null
  },
  {
    "key": "@/f262131bc95ff1edff109873b6d49918/ros2/route/service/srv/talker/get_parameters",
    "value": {
      "is_active": true,
      "local_nodes": [
        "/talker"
      ],
      "remote_routes": [],
      "rep_reader": "0110ffcc8892357cf4b3605500001304",
      "req_writer": "0110ffcc8892357cf4b3605500001203",
      "ros2_name": "/talker/get_parameters",
      "ros2_type": "rcl_interfaces/srv/GetParameters",
      "zenoh_key_expr": "talker/get_parameters"
    },
    "encoding": "application/json",
    "timestamp": null
  }
]
```

</details>

* 白名單與黑名單：我們可以創一個給 zenoh-bridge-ros2dds 的設定檔，然後在裡面設定白名單與黑名單。
    * 官方提供的[設定檔範例](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/main/DEFAULT_CONFIG.json5)

```bash
# 假設設定檔為 myconfig.json5
zenoh-bridge-ros2dds -c myconfig.json5
```

!!! warning
    由於 `zenoh-bridge-ros2dds` 的 DDS 部份是使用 CycloneDDS 的 API，因此無法使用其他 DDS 的特殊功能。
    另外，由於 FastDDS 和 CycloneDDS 對於 ROS 2 service 和 action 的處理方式有些微不同，所以 `zenoh-bridge-ros2dds` 在 FastDDS 的情況下不支援 service 和 action。

## 運行原理

ROS 2 中有三種通訊模式： Topic、Service、Action。
`zenoh-bridge-ros2dds` 會有各自的轉換方式：

* ROS 2 Topic：對應到 Zenoh 的 publish 和 subscribe
* ROS 2 Service：對應到 Zenoh 的 query 和 queryable
* ROS 2 Action：因為 Action 本身就是由多個 Topic 和 Service 組成，所以對於 Zenoh 也是同樣的方式

## 和 rmw_zenoh 的比較

對於 ROS 2，還蠻常會有人詢問到底要用 `zenoh-bridge-ros2dds` 還是 `rmw_zenoh`。
如果需要有比較好的效能 `rmw_zenoh` 是比較好的選擇，可以有效避免不同協定轉換的成本，畢竟每次轉換就是一次訊息的複製。
然而 `zenoh-bridge-ros2dds` 則是能在避免需要影響整個系統下，還能嘗試 Zenoh 的功能。
我會建議先使用 `zenoh-bridge-ros2dds` 來評估 Zenoh 是否符合需求，如果符合再考慮全面使用 `rmw_zenoh`。

## 有用連結

* [zenoh-bridge-ros2dds 程式碼](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
