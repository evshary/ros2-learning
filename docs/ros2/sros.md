# SROS

ROS 2 跟 ROS 1 相比一個最大差異是有完整實作 security 的功能，我們稱之為 SROS。
ROS 2 的 security 其實是基於 DDS 的 security 來實作的，主要包含下面三種 DDS 功能：

* 加密(encryption)
* 認證(authentication)
* 存取控制(access control)

DDS Security 有三種設定檔案，而這些也同樣被 SROS 所繼承：

* identity: 認證身份的工具，public/private key
* permission: 權限設定，可以存取哪些 topic
* governance： 設定 global 的安全策略，所有 node 都必須要遵守

基於此之上，SROS 有兩個概念來包含這些檔案： keystore 和 enclave

* keystore: 存放 global 的設定和 CA 的 public & private key
* enclave: 「安全飛地」的概念，一個 keystore 內可以有多個 enclave，有獨立的 config 和 key，確保不會受到彼此影響

除此之外，SROS 會大量使用到 `ros2 security` 這個 command

```bash
# 創造存放 keystore 的位置
ros2 security create_keystore ROOT
# 創造出 enclave：可以針對不同 node 存放 key 和設定檔
ros2 security create_enclave ROOT NAME
# 從 identity 和 policy 產生出 key 和 permission 檔案
ros2 security generate_artifacts [-k KEYSTORE_ROOT_PATH] [-e [ENCLAVES ...]] [-p [POLICY_FILES ...]]
# 從 ROS graph data 產生出 policy 檔案
ros2 security generate_policy --no-daemon --spin-time 3 POLICY_FILE_PATH
# 從 policy 產生出 permission
ros2 security create_permission ROOT NAME POLICY_FILE_PATH
# 列出某個 keystore 裡面有哪些 enclaves
ros2 security list_enclaves ROOT
```

## 使用方法

### Talker & Listener

這邊講解初步的 SROS 使用方法

* 先產生 SROS 的工作空間，我們後續的 certificate 和 key 都會放在這裡

```bash
mkdir ~/sros2_demo
cd ~/sros2_demo
```

* 設定 ROS 2 Jazzy 的環境，並且使用 Cyclone DDS

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

* 產生 demo_keys 這個 keystore，值得注意的是 SROS 的操作都是用 CLI 的方式，也就是 `ros2 security`
    * 執行完後會產生如下的目錄架構
        * enclaves: 底下的 xml 是 security 的相關設定(例如哪些功能要加密之類)，而 p7s 則為了確保 xml 不被竄改的簽名。
        * private: 存放 CA 的 private key，之後的憑證產生都需要由 CA 來簽核，驗證身份
        * public: 存放 CA 的憑證(public key)

```bash
$ ros2 security create_keystore demo_keystore
$ tree .
.
└── demo_keystore
    ├── enclaves
    │   ├── governance.p7s
    │   └── governance.xml
    ├── private
    │   ├── ca.key.pem
    │   ├── identity_ca.key.pem -> ca.key.pem
    │   └── permissions_ca.key.pem -> ca.key.pem
    └── public
        ├── ca.cert.pem
        ├── identity_ca.cert.pem -> ca.cert.pem
        └── permissions_ca.cert.pem -> ca.cert.pem

5 directories, 8 files
```

* 產生給 talker 這個 node 所需的資源 (key, certificate等等)
    * 會在 enclaves 下產生出一個相對應的資料夾
        * `cert.pem`: talker 的公開憑證
        * `key.pem`: talker 的 private key
        * `governance.p7s`: 對應到 global 的設定，可以檢查自己的設定使否違反 global 的內容
        * `identity_ca.cert.pem`: 對應到 global 的 CA 憑證，可以用來驗證其他人的合法性
        * `permissions_ca.cert.pem`: 對應到 global 的 CA 憑證，用來驗證其他人 permission.p7s 的身分
        * `permissions.p7s`: 用來防止 permissions.xml 被竄改的簽名
        * `permissions.xml`: 紀錄這個 node 有哪些權限

```bash
$ ros2 security create_enclave demo_keystore /talker_listener/talker
$ tree demo_keystore/enclaves/talker_listener
demo_keys/enclaves/talker_listener
└── talker
    ├── cert.pem
    ├── governance.p7s -> ../../governance.p7s
    ├── identity_ca.cert.pem -> ../../../public/identity_ca.cert.pem
    ├── key.pem
    ├── permissions_ca.cert.pem -> ../../../public/permissions_ca.cert.pem
    ├── permissions.p7s
    └── permissions.xml

2 directories, 7 files

```

* 同理，listener 也要做同樣的設定

```bash
ros2 security create_enclave demo_keystore /talker_listener/listener
```

* 最後就是要設定 SROS 的環境變數
    * ROS_SECURITY_KEYSTORE: SROS 的 keystore 放哪邊，也就是我們前面產生的檔案
    * ROS_SECURITY_ENABLE: 是否要啟動 SROS
    * ROS_SECURITY_STRATEGY: security 的嚴格程度。Enforce 代表一定要有 security 檔案才能執行，其他則是沒有就走一般非安全的方式。

```bash
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

* 開始執行
    * 記住一定要加上 `--ros-args --enclave`，它會去找相對應的 key 和 certificate

```bash
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```

### roscli

如果我們要讓 `ros2 topic list` 之類的 CLI 工具可以連接 SROS，那還需要做如下設定

```bash
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ENCLAVE_OVERRIDE=/talker_listener/listener
```

* 執行 CLI 工具，注意要使用 daemon，因為 daemon 可能不是在有 SROS 的環境下啟動

```bash
# node list
ros2 node list --no-daemon --spin-time 3
# topic list
ros2 topic list --no-daemon --spin-time 3
```

### policy

目前只有 Authenication 和 Encryption 兩種功能，如果要有 Access Control 的話，首先要修改 permission。
SROS 工具提供我們從好讀易懂的 policy 設定檔轉換成 permission 的能力。
詳細可以參考[官方教學](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Access-Controls.html)

* 取得 policy 檔案
    * 這裡有 policy 的範例： https://github.com/ros2/sros2/tree/rolling/sros2/test/policies

```bash
sudo apt update && sudo apt install subversion
cd ~/sros2_demo
svn checkout https://github.com/ros2/sros2/trunk/sros2/test/policies
```

* (Optional) 除了抓現有的 policy 檔案，你也可以從既有的 ROS graph 產生

```bash
# Run your scenario
ros2 security generate_policy --no-daemon --spin-time 3 mypolicy.xml
```

* 使用該 policy 來產生出 `permission.xml`
    * 你可以比較前後在 enclaves 資料夾中的 `permissions.xml` 差異

```bash
ros2 security create_permission demo_keystore /talker_listener/talker policies/sample.policy.xml
ros2 security create_permission demo_keystore /talker_listener/listener policies/sample.policy.xml
```

* 測試

```bash
# 可成功執行
ros2 run demo_nodes_cpp talker --ros-args -e /talker_listener/talker
ros2 run demo_nodes_py listener --ros-args -e /talker_listener/listener
# 會失敗
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=not_chatter -e /talker_listener/talker
ros2 run demo_nodes_py listener --ros-args -r chatter:=not_chatter -e /talker_listener/listener
```

* 你可以在 `policies/talker_listener.policy.xml` 加上 `<topic>not_chatter</topic>` 再試試

### governance

* 先把 `governance.xml` 的 domain_id 改為 1

* 重新產生 governance.p7s

```bash
openssl smime -sign -text -in demo_keystore/enclaves/governance.xml -out demo_keystore/enclaves/governance.p7s --signer demo_keystore/public/identity_ca.cert.pem -inkey ~/sros2_demo/demo_keystore/private/identity_ca.key.pem
```

* 重新在 enclaves 產生 `permisssions.p7s`

```bash
cd demo_keystore/enclaves/talker_listener/talker
# 把 permission.xml 的 domain_id 改為 1
openssl smime -sign -text -in permissions.xml -out permissions.p7s \
  --signer permissions_ca.cert.pem \
  -inkey ~/sros2_demo/demo_keystore/private/permissions_ca.key.pem
cd -
# listener 也要
cd demo_keystore/enclaves/talker_listener/listener
# 把 permission.xml 的 domain_id 改為 1
openssl smime -sign -text -in permissions.xml -out permissions.p7s \
  --signer permissions_ca.cert.pem \
  -inkey ~/sros2_demo/demo_keystore/private/permissions_ca.key.pem
cd -
```

## Limitations

* 之前 Cyclone DDS 的 foxy package 並沒有包含 security 功能，只能從 source code build
    * 參考 [Working with RMW CycloneDDS and SROS2](https://answers.ros.org/question/363020/working-with-rmw-cyclonedds-and-sros2/?answer=363765#post-id-363765)
    * [發給 ROS 2 官方的 issue](https://github.com/ros2/ros2/issues/1051)
* 目前不同的 DDS 實作是無法互相用 SROS 通訊的

## 實際應用

* 實際部署的最佳作法： https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Deployment-Guidelines.html
* Turtlebot 如何用 SROS： https://github.com/ros-swg/turtlebot3_demo

## 可探討問題

* 如何把現在非加密的 ROS 2 轉換成加密
* 如何使用 security log

## 參考資料

* ROS Design:
    * [ROS 2 DDS-Security integration](https://design.ros2.org/articles/ros2_dds_security.html): ROS 官方介紹 ROS security 的設計理念
    * [ROS 2 Access Control Policies](https://design.ros2.org/articles/ros2_access_control_policies.html): 如何設定 Access Control Policies
    * [ROS 2 Security Enclaves](https://design.ros2.org/articles/ros2_security_enclaves.html): security 檔案擺放位置的意義
    * [ROS 2 Robotic Systems Threat Model](https://design.ros2.org/articles/ros2_threat_model.html): 分析甚麼樣的資料需要被保護
* [GitHub SROS](https://github.com/ros2/sros2): SROS 的 source code 和安裝教學
* [Setting up security](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html): ROS 官方教學
* Ubuntu 介紹 SROS
    * [Robotics security: What is SROS 2?](https://ubuntu.com/blog/what-is-sros-2): Ubuntu 介紹 SROS
    * [ROS 2 Foxy Fitzroy and its Enhanced Security Monitoring](https://ubuntu.com/blog/ros-2-foxy-fitzroy-and-its-enhanced-security-monitoring): 如何使用 security log
