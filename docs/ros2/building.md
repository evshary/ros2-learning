---
title: ROS 2 編譯
description: 如何開發一個 ROS 2 的套件
keywords:
  - ROS 2
  - 機器人
  - 開發
---

在了解 ROS 2 基本操作後，對開發者而言下一步就是要學習怎麼建構自己的專案。
這邊會介紹一般 ROS 2 的資料夾結構應該是怎樣，以及如何進行開發編譯。

如果開發過 ROS 1，應該會比較習慣 ROS 2 的專案形式。
差異大概是有些資料夾名稱不一樣，而且編譯的工具也不一樣，其他基本上是大同小異。

## 專案結構

通常我們在一個 ROS 的專案資料夾中，會有如下幾個資料夾

* your_workspace: 你的專案資料夾
    * src: 用來放程式碼的地方
    * build: 編譯過程中的中間檔
    * install: 放置編譯後結果的地方
    * log: 編譯或測試的紀錄檔

build, install, log 這三個資料夾都是編譯後自動產生出來的，在編譯前你只需要把程式碼放到 src 資料夾即可。

## 取得程式碼

那麼在 src 資料夾的程式碼應該要怎麼擺放呢？
一般而言在 src 資料夾底下可以放有多個小專案，編譯工具會自動遞迴去找 src 資料夾下的所有小專案並且編譯。
這樣設計的原因是我們可以把一個大功能拆分成多個小專案，而這些小專案就可以輕易地被重複利用。

下面我們分兩個區塊來介紹，一個是取得已有專案，另一個是自己建立新專案。

### 取得已有專案

我們直接舉個實際的例子幫助大家了解，下面是我們下載既有專案的過程：

```bash
# 建立 workspace，並在底下建立 src
mkdir -p your_workspace/src
cd your_workspace
# 取得專案程式碼
wget https://xxxxx/project.repos
vcs import src < project.repos
```

第一步大家應該比較好理解，就是先建立前面所提到的資料夾結構。
重要的是第二步，我們需要把組成專案的所有程式碼抓下來，但如前面所提，一個大專案是由多個小專案組合而成，而這個結構通常會紀錄在 `.repos` 檔中。
我們可以看一下到底裡面紀錄什麼。

```raw
repositories:
  component1:
    type: git
    url: https://github.com/xxx/component1.git
    version: foxy
  component2:
    type: git
    url: https://github.com/yyy/component2.git
    version: commit_ID
...
```

可以看到有這個列表中有 component1 和 component2，而他們也是組成這個專案的 repo。
底下會註明是用什麼版控工具(type: git)，專案位置(url: xxxx)，以及要取這個專案哪個版本或 commit(version: xxx)。用這個列表就可以輕易管理專案的各個元件版本，也比較不會出現當某個小元件更新後會影響到整個專案的狀況。

那要怎麼利用這個列表把程式碼下載下來呢？
這就要用 vcs 這套工具。vcs 是 [vcstool](https://github.com/dirk-thomas/vcstool) 專案的 CLI 工具，他可以輕易把紀錄在 `.repos` 的程式碼下載下來。
以上面範例為例，`vcs import src < project.repos`代表把 project.repos 中的元件下載下來並且放入 src 資料夾。
當然 vcs 還有其他好用功能，但是最常用的還是 import 為主。

### 自己建立專案

自己建立專案的話就比較麻煩一點，需要使用到 ROS 2 的 `ros2 pkg create`，以下面為例。

```bash
# 建立 workspace，並在底下建立 src
mkdir -p your_workspace/src
cd your_workspace/src
# C++ 專案
ros2 pkg create --build-type ament_cmake <project_name>
# Python 專案
ros2 pkg create --build-type ament_python <project_name>
```

執行 `ros2 pkg create` 時，依照你的 `--build-type` 不同，可以決定是 C++ 專案還是 Python 專案。
你可以直接進入專案的資料夾中，會看見底下有 `package.xml` 的資料夾，這個就是 ROS 2 用來辨認專案內容的描述檔，裡面的細節之後再來介紹。

## 編譯

了解如何取得程式碼後，我們就需要了解怎麼編譯。
在 ROS 2 的世界中，進行編譯的重要工具就是 [colcon](https://colcon.readthedocs.io/en/released/index.html)，比較細節的部份可以參考官網。我們先來看看下面的範例：

```bash
cd your_workspace
colcon build
```

執行完成後，你就會發現 `your_workspace` 底下產生了 build, install, log 三個資料夾。

這邊稍微提一下 C++ 的 ROS 2 專案是使用 cmake 來進行編譯管理的。
因此我們舉 C++ 為例說明這三個資料夾的功用，你可以想像 build 資料夾就是放置執行 `make` 所產生的中間檔，而 install 則是用來放置 `make install` 要安裝的編譯結果。
所以如果要撰寫 ROS 2 C++ project 時，要能夠被使用者執行的檔案一定要有 install 才行。

當然 `colcon build` 也是可以帶參數的，下面列出一些常用的執行指令：

```bash
# 指定要編譯某個專案
colcon build --packages-select <certain project>
# install 資料夾不要放 copy 的檔案，而是用 symbolic link 連結到 src 資料夾
colcon build --symlink-install
# 同時用 n 個 thread 進行編譯
colcon build --parallel-workers n
# 預設 install 資料夾下的 component 都會有獨立資料夾，用 merge-install 則是不再用資料夾來歸類
colcon build --merge-install
# 使用 release build，當要減少 code size 時，這個是很重要的指令
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 執行

執行的部份就非常容易了，只要 `source your_workspace/install/local_setup.bash` 後，你就可以輕易使用 `ros2 run` 或 `ros2 launch` 執行你編譯好的專案。這邊細節就不再多提了。

## 參考資料

* [Using colcon to build packages](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
