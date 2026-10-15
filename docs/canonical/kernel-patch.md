---
title: 在 Ubuntu kernel 加上 Patch
description: 取得 Ubuntu kernel 原始碼、套用 patch、編譯並開機測試
keywords:
  - Ubuntu
  - Kernel
---

本文整理一個練習流程：將指定 patch cherry-pick 到相符的 Ubuntu kernel 原始碼，建置套件，安裝後重新開機驗證。
自訂 kernel 僅供開發與測試，不應直接用於正式環境。

## 準備環境

先確認 Ubuntu 版本、kernel flavour 和目前版本：

```bash
lsb_release -cs
uname -r
```

以下以 `generic` kernel 為例。將 `<series>` 換成 Ubuntu 代號（例如 `noble`），並選擇與要測試的 kernel 相符的 Ubuntu kernel tag 和 patch commit。
不要把不同 Ubuntu series 或不相容版本的 patch 混用。

### 啟用 `deb-src`

雖然我們待會會用 `git clone` 從 Launchpad 取得 kernel source，不需要啟用 `deb-src`。
但下面的 `apt build-dep` 會透過 APT 查詢 kernel source package 的建置相依套件，因此需要 `deb-src` source repository。

Ubuntu 24.04 及更新版本使用 deb822 格式設定檔，編輯 `/etc/apt/sources.list.d/ubuntu.sources`：

```bash
sudo vi /etc/apt/sources.list.d/ubuntu.sources
sudo apt update
```

在相關區段的 `Types:` 設定中加入 `deb-src`，例如將 `Types: deb` 改成 `Types: deb deb-src`。

修改後更新 APT 套件清單，再安裝建置相依套件：

```bash
sudo apt update
sudo apt build-dep -y linux linux-image-unsigned-$(uname -r)
sudo apt install -y git fakeroot llvm libncurses-dev dwarves
```

RAM 建議至少 8 GB，並預留約 40 GB 磁碟空間。

## 取得原始碼並套用 Patch

從對應 Ubuntu series 的 Launchpad Git repository 複製 kernel source。
假設我們以 noble 為範例：

```bash
git clone https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/noble
cd noble
git tag -l 'Ubuntu-*'
```

選擇要測試的 `Ubuntu-*` tag，建立工作分支後套用 patch：

```bash
git checkout -b kernel-patch-practice Ubuntu-<version>
git cherry-pick <patch-commit>
```

若發生衝突，解決並檢查變更後執行 `git cherry-pick --continue`；若要放棄這次套用，執行 `git cherry-pick --abort`。
套用後可用 `git show --stat --oneline HEAD` 檢查結果。

## 準備並建置

把 kernel ABI 改成 `999`，例如 `6.8.0-999-...`，讓練習用 kernel 與發行版提供的 kernel 區別開來。
Generic kernel 修改 `debian.master/changelog` 第一行的 ABI 欄位；其他 flavour 則修改對應的 `debian.<flavour>/changelog`。

```bash
chmod a+x debian/scripts/* debian/scripts/misc/*
fakeroot debian/rules clean
```

如果有想要修改 kernel 的 config，可以用下面的指令來修改：

```bash
fakeroot debian/rules editconfigs
```

每次修改原始碼或設定後，都先清理再重新建置：

```bash
fakeroot debian/rules clean
fakeroot debian/rules binary
# 如果想要全速來跑的話
#DEB_BUILD_OPTIONS="parallel=$(nproc)" fakeroot debian/rules binary
```

成功後，產生的 `.deb` 套件會在原始碼目錄的上一層。一般需要安裝該次建置的 headers、image 和 modules 套件。

### 建構的 `.deb` 套件

套件名稱會包含 kernel 版本、flavour 和架構；下表以 `generic` 為例。不同 Ubuntu 版本或設定產生的套件可能不同：

| 套件名稱範例 | 內容與用途 | 安裝時機 |
| --- | --- | --- |
| `linux-image-unsigned-<version>-generic_<version>_<arch>.deb` | Kernel image，也就是開機載入的核心。`unsigned` 表示映像未簽署。 | 必裝；Secure Boot 啟用時，未簽署的 image 通常無法開機。 |
| `linux-modules-<version>-generic_<version>_<arch>.deb` | 此 kernel flavour 的主要可載入模組，包含許多常用驅動程式。 | 必裝，且版本要與 image 相同。 |
| `linux-headers-<version>_<version>_all.deb` | 各架構共用的 kernel headers 和建置檔案。 | 建議安裝，尤其需要建置 DKMS 或其他外部 kernel module 時。 |
| `linux-headers-<version>-generic_<version>_<arch>.deb` | 對應架構及 flavour 的 headers。 | 通常與共用 headers 一起安裝；DKMS 需要它來建置外部模組。 |
| `linux-modules-extra-<version>-generic_<version>_<arch>.deb` | 額外的 kernel modules，可能包含特定硬體需要的驅動程式。 | 依硬體需求安裝。 |
| `linux-modules-iwlwifi-<version>-generic_<version>_<arch>.deb` | Intel Wi-Fi（`iwlwifi`）相關的 kernel modules。 | 有使用 Intel Wi-Fi，且該驅動由此套件提供時才需要。 |
| `linux-tools-<version>-generic_<version>_<arch>.deb` | 與 kernel 版本搭配的工具，例如效能分析工具。 | 一般開機不需要；需要使用套件提供的工具時才安裝。 |

通常先安裝 image、主要 modules 和 headers；額外的 modules 套件則依測試機硬體決定。`lsmod` 沒看到某個驅動不代表一定不需要，因為裝置可能尚未使用或驅動尚未載入。若不確定套件內容或相依性，可檢查：

```bash
dpkg-deb -f <package.deb> Package Depends
dpkg-deb -c <package.deb>
```

只選擇目標架構、flavour 和版本相符的套件；若 `Depends` 列出其他同版本套件，也要一併安裝。

## 安裝與開機驗證

從原始碼目錄上一層，以 `dpkg -i` 安裝剛產生、版本相同的 headers、image 和 modules 套件（以及硬體需要的額外 modules），再重新開機：

```bash
sudo dpkg -i <headers-all.deb> <headers-flavour.deb> <image-unsigned-flavour.deb> <modules-flavour.deb>
sudo reboot
```

重新開機後確認正在執行新版本，並測試 patch 預期影響的功能：

```bash
uname -r
```

`uname -r` 應與 `debian.master/changelog` 中設定的 kernel 版本相符。保留原本可用的 kernel，若新 kernel 無法正常啟動，可從開機選單選回舊 kernel。

> **Secure Boot 注意事項：**此流程產生的 kernel 套件未簽署。Secure Boot 啟用時，新 kernel 通常無法開機；請先在停用 Secure Boot 的測試機或虛擬機測試，或先使用已註冊 MOK 簽署 kernel。

## 官方文件

- [Ubuntu Kernel 文件首頁](https://ubuntu.com/kernel/docs/)
- [使用 Git 取得及管理 Ubuntu kernel source](https://canonical-kernel-docs.readthedocs-hosted.com/how-to/source-code/obtain-kernel-source-git/)
- [建置 Ubuntu Linux kernel](https://canonical-kernel-docs.readthedocs-hosted.com/how-to/develop-customise/build-kernel/)
- [啟用 kernel source package repositories](https://canonical-kernel-docs.readthedocs-hosted.com/how-to/source-code/enable-source-repositories/)
