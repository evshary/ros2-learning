---
title: Autoware 地圖
description: Autoware 的地圖格式
keywords:
  - Autoware
  - 自駕車
  - 地圖
---

## 常見的自駕車地圖

* OpenDRIVE：副檔名為 `.xodr`，內容為 XML 的格式 (Carla / Apollo 使用)
* Lanelet2: 副檔名為 `.osm`，Lanelet2 是基於 OpenStreetMap 的擴充，也是 XML 的格式 (Autoware 使用)

## Lanelet2

[Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) 是專門為自動駕駛設計的地圖框架，使用 OSM 格式，Autoware 是使用這個

分為三層：

* 物理層：包含能具體觀測的元素，如道路、停車格等等
* 關聯層：物理元素關聯到虛擬規則，如車道、區域、交通規則
* 拓樸層：物理元素如何彼此連接

其他細節可參考[面向自動駕駛的高精度地圖框架解析和實戰](https://ppfocus.com/0/di1e5d759.html)

常用資源：

* OSM 地圖 tag：OSM 地圖上面的各個 tag 涵義，可參考[這邊](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LinestringTagging.md)

### 地圖投影格式

我們需要有一個專門的檔案來描述如何把全球投影座標轉換成 local 的座標

Autoware 支援三種投影格式
https://github.com/autowarefoundation/autoware.universe/blob/main/map/autoware_map_projection_loader/README.md

* MGRS（Military Grid Reference System）
    * NATO 專用
    * 把全球地圖切成多個方格，在每個方格用數字定位
* Local Cartesian UTM
    * 切成較小的平面並設立原點，方便定位
* Transverse Mercator (UTM)
    * 把地球座標投影到圓柱上面來定位

## Autoware

Autoware 有用到兩種地圖：

* 點雲地圖 ([PCD 格式](https://docs.web.auto/en/user-manuals/vector-map-builder/introduction#pcd)， `.pcd` 結尾): 來自 Lidar
* 向量地圖 ([OSM 格式](https://docs.web.auto/en/user-manuals/vector-map-builder/introduction#what-is-lanelet2)，`.osm` 結尾): 提供道路相關的訊息，又稱為 Vector Map
    * 路線: 如車道如何連接，能否變道...
    * 物體形狀: 車道、紅綠燈、斑馬線、停車格...
    * 交通規則: 紅綠燈、停止線、限速、車道方向...

## 建立地圖

可以參考 [Autoware 的教學](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/)

### PCD

如果是現實環境，需要用 [SLAM 的技術](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/open-source-slam/)來建地圖

如果是 Carla，這邊有些[相對應的套件](https://carla.readthedocs.io/projects/ros-bridge/en/latest/pcl_recorder/)可以使用

### Vector Map

* [Vector Map Builder](https://tools.tier4.jp/vector_map_builder_ll2/)： TierIV 出的地圖編輯器，可以吃 pcd 並且在上面創造出 Lanelet2，不過需要註冊帳號才能用，可參考[教學](https://docs.web.auto/en/user-manuals/vector-map-builder/introduction)
* [JOSM](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_maps/README.md)：可以用來編輯 Lanelet2 地圖

## 地圖轉換

常見的 map 是 OpenDrive 格式 (xodr)，像是 Carla 的預設地圖就是 xodr，如果要能讓 Autoware 使用，就需要轉換成 OpenStreetMap 格式 (osm)

我在整合 Carla 和 Autoware 的時候就有遇到地圖轉換的問題，下面找了許多解法，雖然很多都無法正常使用

* xodr 轉 osm (但是是 trial 版本，且無法順利運作): https://github.com/carla-simulator/carla-autoware/issues/117
* [xodr-OSM-Converter](https://github.com/tiev-tongji/xodr-OSM-Converter): xodr 轉 osm 的工具，但我嘗試後轉換的地圖並無法使用
* [opendrive2lanelets-converter](https://github.com/wenlong-dev/opendrive2lanelets-converter): opendrive 轉 lanelets，但我轉換後發現格式還是有問題
* Carla 提供已經轉好的 Autoware Map，但有點過時: https://bitbucket.org/carla-simulator/autoware-contents/src/master/
* 後來找到別人提供的轉好地圖： https://github.com/hatem-darweesh/op_agent/tree/ros2/autoware-contents/maps/vector_maps/lanelet2

## Code Study

注意這邊以 galactic 版本為主，新版可能有改動

理論上只要有 pcd 和 osm 就可以跑 map_loader

Autoware 會去讀 `map_path` 這個 argument 的值，然後從底下找出 pcd 和 osm 兩個檔案

`map.launch.py` 會跑如下幾個 Node

* map_loader 中的 **map_hash_generator**
    * publisher `/api/autoware/get/map/info/hash`: 會把 lanelet 的內容和 pcd 的路徑 hash 起來並發佈出去
    * service `/api/autoware/get/map/lanelet/xml`: 當 client 來詢問時，提供 lanelet 的內容
* Composable:
    * map_loader 中的 plugin **Lanelet2MapLoaderNode**: 讀取 Lanelet2 格式
        * publish `vector_map`: 讀 lanlet2 的檔案，並將其轉成 binary，用 transient_local
    * map_loader 中的 plugin **Lanelet2MapVisualizationNode**: 在 RViz 畫圖
        * subscribe `vector_map`，然後 publish `vector_map_marker` 給 RViz
    * map_loader 中的 plugin **PointCloudMapLoaderNode**: 讀取 PCD 格式
        * publish `PointCloud2` 的 msg 到 `pointcloud_map`
        * service `/map/get_partial_pointcloud_map` 可以提供 client 部份的 PCD  map
        * **問題**：`map.launch.py` 中的 LINE 88 remap，似乎沒用到 `service/get_differential_pcd_map`，懷疑是忘記刪掉
    * map_loader 中的 plugin **VectorMapTFGeneratorNode**: 接收 pointcloud 資訊後進行座標轉換
        * subscriber `vector_map`，然後發布 TF 轉換 (map_frame <=> viewer_frame)
