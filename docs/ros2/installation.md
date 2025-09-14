# ROS 安裝

ROS 2 的官方 support platform 有三個主流平台：Linux, Windows, MAC。不過由於 Linux 還是比較多人在使用，所以初學還是推薦用 Linux 來學習。

Linux 的部份目前官方只有原生支援 Ubuntu，建議使用最新版 ROS 2 LTS 搭配對應的 Ubuntu 版本。例如目前是 jazzy 搭配 Ubuntu 24.04。

## 官方安裝方法

ROS 2 的安裝有很多方法，像是使用 apt 安裝 Debian 套件，或從 source code 自行編譯都可以，可以參考 [Installing ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)。

最一開始還是先用 apt 安裝比較簡單，具體細節請參考 [Installing ROS 2 via Debian Packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。

* 安裝 ROS 2 的 repository

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
```

* 安裝 ROS 的套件 (含開發 library)

```shell
sudo apt update && sudo apt install ros-dev-tools ros-jazzy-desktop
```

## ROS + docker

如果你不是用 Ubuntu 或是 Ubuntu 版本無法跟最新的 ROS 2 搭配起來，可以使用容器 (container) 的方式來運行。
因為我常常需要跨不同 ROS 2 版本來測試，所以建立了我的容器化開發環境。

* 下載程式

```shell
git clone https://github.com/evshary/ros2_build_env
cd ros2_build_env
```

* 進入你想運行的 ROS 版本(第一次跑會需要時間建立容器)

```shell
#./containers/<the environment you want>/run_container.sh
# 假設想要跑 jazzy
./containers/ros2-jazzy-gui/run_container.sh
```

* 完成後會進入容器的環境內，你的 shell 前面會有 `(ros2-jazzy-gui)`，代表是在容器內的 terminal
    * 我是把 ros2_build_env 掛載進容器內，所以在這資料夾做的修改都可以在本機端看到
    * 容器內部我是使用 zsh，所以在 source ROS 2 環境變數時要注意一下
