# ROS 2 指令

ROS 2 的很多好用指令都是繼承 ROS 1，基本上用法差不多。這邊我們會介紹常用的幾個 command，讓大家對 ROS 2 操作有基本的概念。

這邊要特別注意的是要使用 ROS 2 的指令前一定要先設定環境，也就是要執行如下指令：

```bash
# 可以把 rolling 改成你目前使用的 ROS 版本
source /opt/ros/rolling/local_setup.bash
```

執行這個指令後，就可以使用/編譯 ROS 2 相關的套件，如果你有下載 ROS 2 套件(`sudo apt install ros-rolling-xxxx`)，基本上也都會存放在該 folder 內。

## 常見指令

下面介紹指令時，都要記得先去 source ROS 2 才可以使用。當然更快的方法就是使用我們之前提到的 ros_menu，每次開啟 terminal 可以直接用選項來執行要用哪個 ROS environment。

另外有一點很方便的小技巧：當你 source ROS 2 後，你不只是可以用 `<tab>` 來補全 ROS 2 指令，也可以用 `<tab>` 來補全 package 名稱、executable 名稱，這可以大大增加效率。

### Run ROS 2 package

```bash
ros2 run <package> <executable>
```

`ros2 run` 的第一個參數是 ROS package，第二個參數則是這個 package 裡的執行檔。

舉例來說，當我們想要跑 talker 和 listener 的時候，

```bash
# 第一個 terminal 視窗
ros2 run demo_nodes_cpp talker
# 第二個 terminal 視窗
ros2 run demo_nodes_cpp listener
```

執行完就可以看到 talker 把資訊丟給 listener 了。

### Launch ROS 2 package

```bash
ros2 launch <package> <launch file>
```

當然要分開執行 ROS 2 的不同節點，還需要開兩個 terminal 並設定兩次環境，其實很不方便，你可以直接用 launch 來取代。

要使用 launch 需要先有 launch file，幸好在 demo_nodes_cpp 的 package 內已經有寫好一次啟動 talker 和 listener 的 launch file。

因此你可以執行如下指令：

```bash
ros2 launch demo_nodes_cpp talker_listener.launch.py
```

### ROS 2 node

```bash
ros2 node list
ros2 node info <node name>
```

我們知道 ROS 的世界中 node 是最基本單位，所以要怎麼得知 node 的資訊就非常重要了，這時候我們可以使用 `ros2 node` 這個 commande。

```bash
$ ros2 node list
/talker
/listener
```

node list 可以幫助我們列出目前有哪些存在的 node，舉個例子來說，如果在剛剛 talker / listener 的例子，你就可以看到有兩個 node 分別是 talker 和 listener。

```bash
$ ros2 node info /talker
  Subscribers:
    ...
  Publishers:
    /chatter: std_msgs/msg/String
    ...
  Services:
    ...
```

node info 則是可以幫助我們看到每個 node 的細節資訊，如果我們觀察 /talker，就會看到這個 node 去 subscribe / publish 哪個 topic (例如 publish 到 /chatter 這個 topic)，以及提供哪些 service 的服務，甚至可以看到該 topic / service 的 type (例如 /chatter 是 std_msgs/msg/String)。

### ROS 2 topic

```bash
ros2 topic list
ros2 topic info <topic name>
ros2 topic echo <topic name>
ros2 topic pub <topic name> <message type> <message>
```

我們這邊知道 ROS 的溝通方式基本上都是靠著 topic，publisher 把資料丟給某個 topic，subscriber 則是從該 topic 取得資料，因此 `ros2 topic` 系列的 command 也是非常容易被使用到的。

```bash
$ ros2 topic list
/chatter
...
```

一樣以 talker / listener 當作範例，我們可以看到使用 ros2 topic list 會有一個 topic 是 /chatter。

```bash
$ ros2 topic info /chatter
Topic: /chatter
Publisher count: 1
Subscriber count: 1
```

如果用 info 來看這個 topic，我們也能看到目前都多少人在 publish / subscribe 這個 topic。

```bash
$ ros2 topic echo /chatter
data: 'Hello World: 1169'
---
data: 'Hello World: 1170'
---
```

可以用 echo 的方式來看這個 topic 收到的資料內容。

```bash
$ ros2 topic pub -1 /chatter std_msgs/msg/String "data: 123"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='123') 
```

當然我們也可以模仿 publisher 把資料直接打給 /chatter 這個 topic。

## 參考

* [Understanding ROS 2 nodes](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Nodes/)
* [Understanding ROS 2 topics](https://index.ros.org/doc/ros2/Tutorials/Topics/Understanding-ROS2-Topics/)
