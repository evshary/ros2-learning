---
title: ROS 2 錄製與重放
description: ROS 2 如何紀錄以及重放封包
keywords:
  - ROS 2
  - 機器人
---

在機器人開發中，錄製封包的功能可以在很多方面幫助我們。
舉例來說，如果某個問題的重現很困難，我們可以透過錄製當下發生的封包來更容易復現。
又或者是說，我們可以錄製實際機器人或是自駕車的感測器資訊，像是相機或是光達，然後讓開發者們可以在沒有實際硬體的情況下開發相對應的模組。

在網路世界中，我們是使用 Wireshark 來錄製封包。
然而在機器人領域中，最小的封包單位是一個個 ROS Message，因此需要有其他更適合的工具，也就是這邊要談到的 ROS bag。

## 使用方式

ROS bag 是 ROS CLI 指令下的其中一個部份，我們可以用 `ros2 bag --help` 來看到更多指令的用法。

* 錄製封包

```bash
# 錄製所有 topic 的封包
ros2 bag record --all-topics
# 錄製特定 topic 的封包，例如 mytopic1 和 mytopic2
ros2 bag record --topics mytopic1 mytopic2
```

* 查看封包資訊：錄完後可以看到類似 `rosbag2_2025_10_11-10_38_16` 的資料夾，可以查看其內容

```bash
$ ros2 bag info rosbag2_2025_10_11-10_38_16
Files:             rosbag2_2025_10_11-10_38_16_0.mcap
Bag size:          5.3 KiB
Storage id:        mcap
ROS Distro:        rolling
Duration:          2.999745724s
Start:             Oct 11 2025 10:38:17.945157770 (1760150297.945157770)
End:               Oct 11 2025 10:38:20.944903494 (1760150300.944903494)
Messages:          4
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 4 | Serialization Format: cdr
Services:          0
Service information: 
Actions:           0
Action information: 
```

* 重放封包：會重放當初錄製的內容

```bash
ros2 bag play rosbag2_2025_10_11-10_38_16
```

## ROS bag 格式

那麼實際上 ROS bag 是用什麼格式錄製呢？
最早是使用 SQLite3 的方式，但在後來 [Foxglove](https://foxglove.dev/) 這家公司提出的[相對更好的 MCAP 格式](https://foxglove.dev/blog/introducing-the-mcap-file-format)。
MCAP 最大的好處在於有較佳的 throughput、能自行包含 metadata 來描述自我資訊、提供更多的功能(如更好的 compression 等等)。
最終在 [ROS 2 Iron 以後](https://discourse.openrobotics.org/t/psa-default-ros-2-bag-storage-format-is-changing-to-mcap-in-iron/28489)，預設的 ROS bag 格式也改為 MCAP。

### MCAP 格式

根據[官方規格文件](https://mcap.dev/spec)，MCAP 封包格式為 `<Magic><Header><Data section>[<Summary section>][<Summary Offset section>]<Footer><Magic>`。
Magic 是 `0x89, M, C, A, P, 0x30, \r, \n`，而剩下部份都是 Records 格式。
Records 有如下資訊：

* 操作碼（Opcode）：一個單一位元組，用於識別記錄的類型（例如：Schema、Channel、Message、Chunk 等）。
* 內容長度（Content Length）：一個 uint64 值，指示記錄內容的長度。
* 記錄內容（Record Content）：實際的資料。

下面列出常用到的 Opcode：

| Opcode (十六進位) | 記錄名稱 (Record Name) | 功能說明 |
| - | - | - |
| 0x01 | Header | 檔案的第一個記錄，包含 MCAP 版本資訊和格式細節 |
| 0x03 | Schema | 儲存訊息類型的結構定義（各個 ROS 訊息的實際定義） |
| 0x04 | Channel | 定義主題名稱、對應到的 schema ID 和 metadata (ROS 實際 topic 名稱、type 格式 (schema ID) 和紀錄 QoS 的 metadata) |
| 0x05 | Message | 儲存帶有 timestamp 和 channel ID 的實際訊息 |
| 0x0A | Metadata | 儲存應用程式定義的 key-value metadata (在 ROS 中會用來紀錄 bag file 基本資訊) |

注意 `0x01` 到 `0x7F` 是保留給標準 MCAP 格式的記錄類型。

### MCAP CLI

除了使用 `ros2 bag` 指令外，我們也可以使用 [MCAP CLI](https://mcap.dev/guides/cli) 來觀察 MCAP 的內容。

* 查看 MCAP 的基本資訊

<details>
  <summary>mcap info *.mcap</summary>

```bash
$ mcap info *.mcap
library:   libmcap 1.4.0                                               
profile:   ros2                                                        
messages:  14                                                          
duration:  4.911906569s                                                
start:     2025-10-08T11:53:47.926364121+08:00 (1759895627.926364121)  
end:       2025-10-08T11:53:52.83827069+08:00 (1759895632.838270690)   
compression:
    : [1/1 chunks] [5.24 KiB/5.24 KiB (0.00%)] [1.07 KiB/sec] 
chunks:
    max uncompressed size: 5.24 KiB
    max compressed size: 5.24 KiB
    overlaps: no
channels:
    (1) /rosout              11 msgs (2.24 Hz)   : rcl_interfaces/msg/Log [ros2msg]                  
    (2) /events/write_split   0 msgs             : rosbag2_interfaces/msg/WriteSplitEvent [ros2msg]  
    (3) /chatter              3 msgs (0.61 Hz)   : std_msgs/msg/String [ros2msg]                     
    (4) /parameter_events     0 msgs             : rcl_interfaces/msg/ParameterEvent [ros2msg]       
channels: 2
attachments: 0
metadata: 2
```

</details>

* 查看 MCAP 的訊息內容

<details>
  <summary>mcap cat *.mcap</summary>

```bash
$ mcap cat *.mcap
1759895627926364121 /rosout [rcl_interfaces/msg/Log] [0 1 0 0 75 224 229 104 155 107]...
1759895627926390649 /rosout [rcl_interfaces/msg/Log] [0 1 0 0 75 224 229 104 80 236]...
...
1759895630838427352 /chatter [std_msgs/msg/String] [0 1 0 0 15 0 0 0 72 101]...
1759895631837926612 /rosout [rcl_interfaces/msg/Log] [0 1 0 0 79 224 229 104 229 149]...
...
```

</details>

* 查看 MCAP 的 metadata

<details>
  <summary>mcap list metadata *.mcap</summary>

```bash
$ mcap list metadata *.mcap
name   offset    length    metadata
rosbag2    42    541   {"serialized_metadata":"version: 9\nstorage_identifier: mcap\nduration:\n  nanoseconds: 0\nstarting_time:\n  nanoseconds_since_epoch: 9223372036854775807\nmessage_count: 0\ntopics_with_message_count:\n  []\ncompression_format: \"\"\ncompression_mode: \"\"\nrelative_file_paths:\n  - rosbag2_2025_10_08-11_53_47_0.mcap\nfiles:\n  - path: rosbag2_2025_10_08-11_53_47_0.mcap\n    starting_time:\n      nanoseconds_since_epoch: 9223372036854775807\n    duration:\n      nanoseconds: 0\n    message_count: 0\ncustom_data: ~\nros_distro: rolling"}
...
```

</details>

* 查看 MCAP 的 schemas

<details>
  <summary>mcap list schemas *.mcap</summary>

```bash
$ mcap list schemas *.mcap
id    name    encoding    data
...
3    std_msgs/msg/String    ros2msg    # This was originally provided as an example message.
                                       ...
                                       string data
...
```

</details>

* 查看 MCAP 的 channels

<details>
  <summary>mcap list channels *.mcap</summary>

```bash
$ mcap list channels *.mcap
id    schemaId    topic    messageEncoding    metadata
...
3    3    /chatter    cdr    {"offered_qos_profiles":"- history: keep_last\n  depth: 7\n  reliability: reliable\n  durability: volatile\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: automatic\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false","topic_type_hash":"RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"} 
...
```

</details>

### MCAP API

我們也可以直接寫程式來解析 MCAP 的內容，官方提供[各種程式語言的 API](https://mcap.dev/reference)。

如果是要解析 ROS bag 的格式，那就需要參考 `ros2 bag` 本身如何寫的：

* 官方 ROS bag 如何解析 MCAP
    * [文件](https://docs.ros.org/en/rolling/p/rosbag2_storage_mcap/)
    * [程式](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_storage_mcap/src/mcap_storage.cpp)
* [Python 的範例](https://mcap.dev/guides/python/ros2)

## 參考資料

* ROSCon talk: MCAP: A Next-Generation File Format for ROS Recording
    * [slides](http://download.ros.org/downloads/roscon/2022/MCAP%20A%20Next-Generation%20File%20Format%20for%20ROS%20Recording.pdf)
    * [video](https://vimeo.com/showcase/9954564?video=767149733)
