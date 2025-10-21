---
title: rmw_zenoh 實戰
description: rmw_zenoh 如何實際應用到真實機器人上
keywords:
  - Zenoh
  - ROS 2
  - 機器人
  - middleware
---

看到這邊，也許你會有個疑問，到底 rmw_zenoh 可以實際帶來什麼好處呢？
對於 DDS 在無線網路下的問題，如果沒有實際碰到可能感受沒有那麼深，因此我們這邊就來列舉一些 rmw_zenoh 實際獨特的功能。

## 共享記憶體 (Shared Memory)

對於效能要求比較高的使用場景，常常我們會要求機內通訊可以直接利用共享記憶體來傳輸，避免訊息還要經過作業系統的網路層以及多次拷貝。
Zenoh 除了一般的網路以外，也支援共享記憶體的通訊。
Zenoh 會自動偵測是否通訊雙方都能用共享記憶體，如果可以就用共享記憶體，不行的話就切回原來網路。
預設 rmw_zenoh 的共享記憶體是關閉的，我們需要另外將其打開，設定並不複雜。

* 修改 Router 和 Session 內部的設定檔

```json5
shared_memory: {
  // ...
  enabled: true,
  // ...
},
```

* 當然更簡單的方法是直接在所有 terminal 都加上 `ZENOH_CONFIG_OVERRIDE`

```bash
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
```

* 除此之外，rmw_zenoh 也有提供兩個環境變數來做進階設定
    * `ZENOH_SHM_ALLOC_SIZE`: Zenoh 的 shared memory 可以用多大的空間，預設為 48 MiB
    * `ZENOH_SHM_MESSAGE_SIZE_THRESHOLD`: 唯有超過一定 size 的資料才會使用 Shared Memory 傳輸，預設為 512 bytes

## 安全通訊 (Security)

資安的重要性在怎麼強調也不為過，當然 Zenoh 也支援相關功能。
當然 rmw_zenoh 也支援 SROS 功能，但如果只是單純要加密跨機通訊的話，我們可以直接利用 mTLS 的方式來確保流量離開網路都是有加密的。

!!! warning
    請注意這邊我們不考慮中間人攻擊的情況，如果真的要避免這個問題，我們會需要有個專門 CA 來認證憑證才行。

假設我們有一台機器人和一台筆電，我們希望兩者之間的通訊是加密的

* 產生憑證與金鑰

```bash
mkdir -p tls
# 建立 root CA 憑證和金鑰
docker run -it --init -v $PWD/tls:/home/step smallstep/step-cli \
  step certificate create --profile root-ca "Example Root CA" root_ca.crt root_ca.key \
  --no-password --insecure
# 產生給機器人的憑證和金鑰，並用 root CA 簽署
docker run -it --init -v $PWD/tls:/home/step smallstep/step-cli \
  step certificate create robot.local robot.crt robot.key \
  --ca ./root_ca.crt --ca-key ./root_ca.key --no-password --insecure
# 產生給筆電的憑證和金鑰，並用 root CA 簽署
docker run -it --init -v $PWD/tls:/home/step smallstep/step-cli \
  step certificate create control.local control.crt control.key \
  --ca ./root_ca.crt --ca-key ./root_ca.key --no-password --insecure
```

* 修改機器人上的設定檔 (Router 部份)

```json5
// ...
listen: {
  // ...
  endpoints: [
    // 本地端內部還是用 TCP 方式連線
    "tcp/localhost:7447",
    // 遠端連進來的話則是用 TLS 來連線
    "tls/192.168.1.1:7447"
  ],
  // ...
},
// ...
tls: {
  root_ca_certificate: "path/to/tls/root_ca.crt",  // CA 憑證
  listen_private_key: "path/to/tls/robot.key",
  listen_certificate: "path/to/tls/robot.crt",
  enable_mtls: true,  // 啟動 mTLS
  connect_private_key: "path/to/tls/robot.key",
  connect_certificate: "path/to/tls/robot.crt",
  verify_name_on_connect: false, // 要不要驗證憑證上的 hostname，為簡化先不要
  // ...
},
// ...
```

* 修改筆電上的設定檔 (假設我們是用 client mode 來連接到機器人，這邊只需要修改 Session 部份)

```json5
// ...
connect: {
  // ...
  endpoints: [
    // 用 TLS 來連到機器人的 Zenoh Router
    "tls/192.168.1.1:7447"
  ],
  // ...
},
// ...
tls: {
  root_ca_certificate: "path/to/tls/root_ca.crt",  // CA 憑證
  listen_private_key: "path/to/tls/control.key",
  listen_certificate: "path/to/tls/control.crt",
  enable_mtls: true,  // 啟動 mTLS
  connect_private_key: "path/to/tls/control.key",
  connect_certificate: "path/to/tls/control.crt",
  verify_name_on_connect: false, // 要不要驗證憑證上的 hostname，為簡化先不要
  // ...
},
// ...
```

## 權限控管 (Access Control)

既然 rmw_zenoh 的對外通訊需要經過 Zenoh Router，其實我們就可以在 Zenoh Router 上面做一些手腳，例如限制某些 topic 無法對外連接。
這個可以有效保護一些敏感資訊不被洩漏，也可以降低網路上的封包數量，減輕整體系統負擔。

啟動方式主要是修改 Zenoh Router 的 Router 設定檔，假設我們現在要擋掉 `camera/image_raw` 這個 topic

```json5
access_control: {
  enabled: true,
  default_permission: "allow",  // 沒有特別被限制，就是允許
  rules:
  [
    {
      id: "deny_topics",
      permission: "deny",  // 限制符合該 key expression 的規則
      messages: [  // 所有格式都要擋下
        "put", "delete", "declare_subscriber",
        "query", "reply", "declare_queryable",
        "liveliness_token", "liveliness_query", "declare_liveliness_subscriber",
      ],
      flows:["egress", "ingress"],  // 進出都要限制
      key_exprs: [
        "*/camera/image_raw/**",  // 我們要檔下的 topic，注意這邊要用 Zenoh 的 key expression，可以參考我們前面提到的 ROS 2 topic 和 Zenoh key expression 對應方式
        "*/camera/image_raw/**/@adv/**", // 如果該 topic 是 TRANSIENT_LOCAL 的話
      ],
    },
  ],
  subjects:
  [
    {
      id: "ALL",
    },
  ],
  policies:
  [
    {
      id: "policy_id",
      rules: ["deny_topics"],
      subjects: ["ALL"],
    },
  ]
},
```

## Downsampling

另外一個降低網路上封包流量的方法是降低流出的頻率，我們一樣可以在 Zenoh Router 上面進行流量設定。

假設我們想要把 `camera/image_raw` 這這個 topic 降到 2Hz。
一樣要調整 Zenoh Router 上的 Router config：

```json5
downsampling: [
  {
    messages: ["put", "reply"],
    flows: ["egress"],  // 限制對外才要調整
    rules: [
      { key_expr: "*/camera/image_raw/**", freq: 2.0 },  // 限制只需要 2 Hz
      { key_expr: "*/camera/image_raw/**/@adv/**", freq: 2.0 },  // 如果 topic 是使用 TRANSIENT_LOCAL 的話
    ],
  },
],
```

## 跨網際網路連接 (Internet)

Zenoh 跟 DDS 最大的不同在於可以很輕易的跨出網路，我們假設現在機器人和筆電在不同網路下，且都是 private IP，代表兩者無法建立直接連線。
這時候我們可以在另外一個有 public IP 的地方運行獨立的 Zenoh Router，然後讓機器人和筆電可以同時連到它，Zenoh Router 可以幫忙轉發訊息。

* 先運行 Zenoh Router，假設 IP 為 1.2.3.4
    * 值得注意的是，我們不一定需要用 ROS 2 的方式來跑 zenohd，直接用 Zenoh 的方式來跑也可以

* 機器人的 Router 設定檔

```json5
{
  connect: {
    endpoints: ["tcp/1.2.3.4:7447"],
  },
}
```

* 筆電的 Session 設定檔，一樣假設是 client mode

```json5
{
  mode: "client",
  ...
  connect: {
    endpoints: ["tcp/1.2.3.4:7447"],
  },
}
```

## ROSCon 工作坊

ZettaScale 其實在過去幾年 ROSCon 的工作坊都有一步步針對 rmw_zenoh 的練習教學。
有興趣可以嘗試看看，可以幫助我們更加了解 rmw_zenoh 運作：

* [ROSCon 2024](https://github.com/ZettaScaleLabs/roscon2024_workshop)
* [ROSCon 2025](https://github.com/ZettaScaleLabs/roscon2025_workshop)
