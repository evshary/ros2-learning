# Autoware 模組

在 Autoware 中可以拆成如下這些 module

* Sensing:
    * 包含了 sensor 的 ROS driver，以及一些初步 prepocessor 的 ROS node
* Localization:
    * 會使用 Sensing 的資料來判斷自己的所在位置，這是很重要的模組，不管是 Perception, Planning, Control 都會使用 Localization 的資料近一步判斷
* Map Data
    * 地圖資訊，Localization(對照自己在哪裡)、Perception(知道紅綠燈位置)、Planning(在地圖畫路徑並判斷交通規則)，都會用到這個
* Perception
    * 包含物件辨識、紅綠燈辨識等等
* Planning
    * 根據定位位置，和 Perception 所偵測到的物件規劃出適合的路線
* Control
    * 根據 Planning 的規劃路徑來實際控制車子的移動
* Vehicle Interface
    * Autoware 和汽車 DbW 連接的界面，通常會隨汽車不同而調整

![Autoware Architecture](images/autoware_architecture.avif)
圖片來源：https://github.com/autowarefoundation/autoware

## Node diagram

上面畢竟是比較粗略的劃分，如果想要更細緻了解 Autoware 中 ROS topic 和 node 的關係，可以參考官方畫的 [node diagram](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/)，這個對 debug 和開發非常有幫助。
