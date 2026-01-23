---
title: AGL 安裝
description: AGL 的安裝方法
keywords:
  - SDV
---

這邊紀錄一下 AGL 的編譯以及安裝方法，我們假設使用的是 Ubuntu 系統，然後上面跑 Qemu。

## 前置安裝

* 先安裝 repo，AGL 常用的抓程式碼工具

```bash
sudo apt-get install repo
```

* 安裝編譯 Yocto 所需要的套件，可參考[官網](https://docs.yoctoproject.org/ref-manual/system-requirements.html#required-packages-for-the-build-host)

```bash
sudo apt-get install build-essential chrpath cpio debianutils diffstat file gawk gcc git iputils-ping libacl1 locales python3 python3-git python3-jinja2 python3-pexpect python3-pip python3-subunit socat texinfo unzip wget xz-utils zstd liblz4-tool
```

## 編譯

* 先建立編譯環境，可以指定 `AGL_TOP` 這個環境變數來方便使用

```bash
export AGL_TOP=$HOME/workspace/agl_ws
mkdir $AGL_TOP
cd $AGL_TOP
```

* 抓取程式碼

```bash
mkdir -p $AGL_TOP/trout && cd $AGL_TOP/trout
repo init -b trout -u https://gerrit.automotivelinux.org/gerrit/AGL/AGL-repo
repo sync
```

* 接下來我們都會使用 `aglsetup.sh` 這個 script

```bash
# 列出所有可能的選項
source meta-agl/scripts/aglsetup.sh -h
```

* 編譯，這邊我們嘗試編譯 Qt based IVI demo

```bash
# Sample Qt based IVI demo 
# -m 代表平台，-b 是存放的資料夾，後面要接上要啟動的功能
source meta-agl/scripts/aglsetup.sh -f -m qemux86-64 -b qemux86-64 agl-demo agl-devel
echo "# reuse download directories" >> $AGL_TOP/site.conf
echo "DL_DIR = \"$HOME/downloads/\"" >> $AGL_TOP/site.conf
echo "SSTATE_DIR = \"$AGL_TOP/sstate-cache/\"" >> $AGL_TOP/site.conf
ln -sf $AGL_TOP/site.conf conf/
# Build
time bitbake agl-ivi-demo-qt
```

!!! note
    在 Ubuntu 24.04 編譯可能會出現如下錯誤
    `ERROR: User namespaces are not usable by BitBake, possibly due to AppArmor.`
    這個是因為 Ubuntu 引入了 user namespace 的機制，我們可以先暫時關閉
    `sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0`
