---
title: Zenoh 協定
description: 介紹 Zenoh 協定內容以及如何用 dissector 觀察
keywords:
  - Zenoh
---

## zenoh-dissector

如果想要學習 Zenoh Protocol，那我們一定需要使用 [zenoh-dissector](https://github.com/eclipse-zenoh/zenoh-dissector) 這個 wireshark plugin 來看。

* 先確保已經有安裝 wireshark

```bash
sudo add-apt-repository ppa:wireshark-dev/stable
sudo apt update
sudo apt install wireshark
```

* 到 [GitHub Release page](https://github.com/eclipse-zenoh/zenoh-dissector/releases) 下載對應自己作業系統的 library
* 解壓縮後會看到 `libzenoh_dissector.so` 這個檔案，要放到對應的位置，這邊是以 `4.6.x` 為例，你要放到對應自己版本的資料夾

```bash
mkdir -p ~/.local/lib/wireshark/plugins/4.6/epan
cp libzenoh_dissector.so ~/.local/lib/wireshark/plugins/4.6/epan/libzenoh_dissector.so
```

* 開啟 wireshark 之後，就可以輸入 zenoh 來查看 protocol
* 我們可以用 Zenoh 最基本的範例來觀察，注意的是目前預設 Zenoh plugin 只會追蹤 port 7447，所以讓我們特別指定 listener 和 connector

```bash
./z_sub -l tcp/127.0.0.1:7447
./z_pub -e tcp/127.0.0.1:7447
```
