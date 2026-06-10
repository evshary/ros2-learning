---
title: ROS 2 環境變數
description: ROS 2 常用的環境變數
keywords:
  - ROS 2
---

在 ROS 2 中可以用環境變數來調整 ROS 2 的行為，這邊列出一些常用的變數並說明功能。

## 常用的環境變數

### `ROS_DOMAIN_ID`

設定 ROS 2 使用的 domain ID，讓不同群組的節點彼此隔離。
如果大家都在同一個網路下開發時，這個功能非常好用，可以避免彼此的 ROS topic 互相衝突。

預設值：`0`

### `ROS_LOCALHOST_ONLY`

限制節點只使用本機 loopback 介面通訊，避免和區域網路上的其他 ROS 2節點互相發現。

這個變數可以視為較舊的控制方式。
現在官方較建議改用 `ROS_AUTOMATIC_DISCOVERY_RANGE` 和 `ROS_STATIC_PEERS`，
因為新作法可以更細緻地控制「只限本機」、「同子網路」或「只連特定 peers」等 discovery 行為。

預設值：`0`，也就是不限制 localhost only

### `ROS_AUTOMATIC_DISCOVERY_RANGE`

控制 ROS 2 自動探索其他節點的範圍。

預設值：`SUBNET`

可用的值包含：

- `SUBNET`：預設值，會自動探索同一個子網路中的節點。
- `LOCALHOST`：只探索同一台機器上的節點。
- `OFF`：不自動探索任何其他節點。
- `SYSTEM_DEFAULT`：不額外修改 discovery 設定，交給底層 middleware 自己決定。

例如如果你只想讓 ROS 2 在本機內互相找到彼此，可以這樣設定：

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### `ROS_STATIC_PEERS`

指定要主動連線的固定 peers 清單，適合搭配 `ROS_AUTOMATIC_DISCOVERY_RANGE` 一起使用。

預設值：未設定

這個變數使用分號 `;` 分隔多個位址，例如 IP 或主機名稱。

```bash
export ROS_STATIC_PEERS='192.168.0.10;robot2.local'
```

常見用法是把自動探索範圍限制在本機，但仍允許連到少數指定主機：

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS='192.168.0.10;robot2.local'
```

### `ROS_DISTRO`

表示目前 shell 載入的是哪一個 ROS 2 發行版，通常在 `source /opt/ros/<distro>/setup.*` 後自動設定。

預設值：沒有固定字串；會跟著你 source 的發行版，例如 `jazzy`、`humble`

### `RMW_IMPLEMENTATION`

指定 ROS 2 要使用哪個 RMW middleware 實作，例如 Cyclone DDS 或 Fast DDS。

預設值：若未設定，使用該 ROS 2 發行版的預設 RMW 實作

## Log 相關

### `RCUTILS_LOGGING_SEVERITY_THRESHOLD`

設定要輸出的最低 log level，低於門檻的訊息不會顯示。

預設值：未設定時通常不額外限制，交由節點或系統原本的 log 設定處理

例如可以這樣設定：

```bash
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=ERROR
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=FATAL
```

像是設成 `WARN` 時，通常只會看到 `WARN`、`ERROR`、`FATAL`。

### `RCUTILS_CONSOLE_OUTPUT_FORMAT`

設定終端機 log 的輸出格式，例如是否顯示時間、logger 名稱與訊息內容。

預設值：`[{severity}] [{time}] [{name}]: {message}`

### `RCUTILS_COLORIZED_OUTPUT`

控制 log 是否使用彩色輸出。

預設值：未設定時自動判斷；若輸出到 TTY，通常會啟用彩色

例如可以這樣設定：

```bash
# 關閉色彩
export RCUTILS_COLORIZED_OUTPUT=0
# 啟用色彩
export RCUTILS_COLORIZED_OUTPUT=1
```

`0` 代表強制關閉彩色輸出，`1` 代表強制開啟彩色輸出。

### `ROS_LOG_DIR`

指定 ROS 2 寫入 log 檔案的目錄。

預設值：未設定時使用 `$ROS_HOME/.log`；若 `ROS_HOME` 也未設定，通常會落在 `~/.ros/.log`
