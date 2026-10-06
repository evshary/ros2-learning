---
title: Juju 入門
description: 使用 Juju 在本機 LXD 建立 controller 並部署應用程式
keywords:
  - Juju
  - Ubuntu
  - LXD
---

Juju 是 Canonical 的服務編排工具，透過 charm 管理雲端或本機環境中的應用程式。它將部署、設定、擴縮及應用程式之間的整合描述成模型，並由 Juju 持續協調實際環境與模型狀態。這讓相同的操作方式可以用在不同的 cloud，而不必每次都手動登入機器設定服務。

## Juju 的架構：Controller、Model、Charm 與 Bundle

Juju 的資源關係可以簡化成：

```text
Cloud
└── Controller
    └── Model
        ├── Application（例如 nginx）
        │   ├── Unit（nginx/0）
        │   └── Unit（nginx/1）
        └── 其他 applications 與它們的 units
```

* **Controller**：Juju 的管理服務，接收 CLI 操作、保存部署狀態，並協調 cloud 上的資源。通常一個 controller 可以管理多個 models。
* **Model**：一個獨立的操作與部署範圍，包含 applications、units、設定及彼此的關係。可用不同 model 隔離開發、測試與正式環境。
* **Application**：model 中受管理的服務，例如部署後名為 `nginx` 的 Nginx 服務。它不是單一程序；應用程式可以有一個或多個 units。
* **Unit**：application 的一個執行個體，例如 `nginx/0`。在 LXD 或一般 machine cloud 上通常對應一台機器或容器；在 Kubernetes cloud 上則通常對應一個 pod。
* **Charm**：描述如何安裝、設定及維護某類 application 的操作程式。`juju deploy nginx` 會用 Nginx charm 建立名為 `nginx` 的 application；charm 負責「如何管理這個應用程式」。
* **Bundle**：可重複使用的部署描述，列出一個或多個 applications 使用的 charms、設定及彼此的 relations。它描述「要一起部署什麼」，讓整個應用程式組合能以一次部署重建。Bundle 不是整個 cloud 或 model 的完整備份。

## Charmhub 是什麼

[Charmhub](https://charmhub.io/) 是 charms 與 bundles 的線上目錄及發布平台。部署 `nginx` 時，Juju 會從 Charmhub 取得對應的 Nginx charm；Charmhub 頁面也提供 charm 支援的平台、版本及可設定選項。Charmhub 提供的是管理應用程式所需的 charm，不是實際執行 Nginx 的環境。

## 安裝與前置需求

安裝 Juju CLI：

```bash
sudo snap install juju --classic
```

本範例需要已安裝並初始化的 LXD。先確認 Juju 能看到本機 LXD cloud：

如果 LXD container 無法連外，且主機的 `FORWARD` policy 是 `DROP`，可暫時允許封包轉送：

```bash
sudo iptables -P FORWARD ACCEPT
```

這會影響主機上的所有轉送流量，不只 LXD；只建議在個人開發機使用。單獨執行此指令不保證重開機後仍保留設定。

```bash
juju version
juju clouds
```

在 cloud 清單中確認有 `localhost`，且類型為 `lxd`。若沒有，請先安裝並設定 LXD，再重新檢查。

## 建立環境並部署

建立 controller，再新增一個 model：

```bash
# 新增 controllers
juju bootstrap localhost juju-tutorial-controller
# 查看 controllers
juju controllers

# 新增 model
juju add-model tutorial
# 查看新的 model
juju status
```

`juju switch` 只會改變 CLI 後續指令的目標，不會搬移或修改部署。`controller` 是 controller 名稱，不是特殊關鍵字；可以只選 controller，或用 `controller:model` 明確選擇 model：

```bash
juju switch juju-tutorial-controller
juju switch juju-tutorial-controller:tutorial
```

部署 Charmhub 上的 Nginx charm，並查看部署狀態：

```bash
juju deploy nginx --base ubuntu@22.04
# 可能需要一點時間才會完成
juju status --watch 2s
```

`juju status` 會顯示 model、應用程式及 unit 狀態。部署完成所需時間依本機資源與網路而異；再次執行 `juju status` 可查看最新狀態。

等 `juju status` 顯示 `nginx/0` 已就緒後，可以 SSH 進入該 unit 所在的 machine 或 container：

```bash
juju ssh nginx/0
```

輸入 `exit` 可離開 SSH session。

## 設定 Nginx

每個 charm 只接受它所定義的設定選項。Nginx charm 提供 `port` 選項，可以在部署後變更服務的監聽 port：

```bash
juju config nginx
juju config nginx port=5000
juju config nginx port
```

第一個指令列出目前設定，第二個把 Nginx 的監聽 port 改為 `5000`，最後一個查看該選項的值。也可以在部署時設定：

```bash
juju deploy nginx --base ubuntu@22.04 --config port=5000
```

這些是 charm 公開的設定，不代表可以直接傳入任意 `nginx.conf` 指令。完整選項與預設值請查看 [Nginx charm 的設定頁面](https://charmhub.io/nginx/configurations)。

## 用 Bundle 部署兩個應用程式

以下 bundle 會一起部署 Charmed MySQL 和 Self-Signed Certificates，並建立兩者的 TLS certificates relation。Juju 會按照 relation 協調兩個 charms，讓 MySQL 取得憑證；若分別部署，還要另外建立這個 relation。

將以下內容存成 `mysql-tls-bundle.yaml`：

```yaml
default-base: ubuntu@22.04
applications:
  mysql:
    charm: mysql
    channel: 8.0/stable
    num_units: 1
  certificates:
    charm: self-signed-certificates
    channel: 1/stable
    num_units: 1
relations:
  - - mysql:certificates
    - certificates:certificates
```

建立 model，再用一個指令部署 bundle 裡的兩個 applications 和它們的 relation：

```bash
juju add-model tutorial-tls
juju deploy ./mysql-tls-bundle.yaml --model tutorial-tls
juju status --model tutorial-tls --relations
```

`juju status --relations` 可查看 relation 是否建立。MySQL 比 Nginx 範例耗用更多資源；不需要時可移除整個 model。

## 清除資源

不再需要範例時，移除 model 及其中的應用程式：

```bash
juju destroy-model tutorial-tls --destroy-storage
juju destroy-model tutorial
```

這個指令會要求確認。Controller 會保留，可供其他 model 使用；若要移除 controller，請先確認其中沒有要保留的 model，再執行：

```bash
juju destroy-controller juju-tutorial-controller --destroy-all-models --force
```

## 參考資料

* [Juju 3.6 Tutorial](https://canonical.com/juju/docs/juju-cli/3.6/tutorial/)
* [Juju architecture](https://canonical-juju-1.readthedocs-hosted.com/3.6/explanation/juju-architecture/)
* [Configure an application](https://canonical-juju-1.readthedocs-hosted.com/3.6/howto/manage-applications/#configure-an-application)
* [Juju bundle reference](https://canonical-juju-1.readthedocs-hosted.com/3.6/reference/bundle/)
* [Charmed MySQL on Charmhub](https://charmhub.io/mysql)
* [Self-Signed Certificates on Charmhub](https://charmhub.io/self-signed-certificates)
* [Juju CLI reference](https://canonical.com/juju/docs/juju-cli/3.6/reference/juju-cli/)
