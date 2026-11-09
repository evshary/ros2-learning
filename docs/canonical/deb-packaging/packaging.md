---
title: Debian package 打包入門
description: 使用 dh_make、quilt、sbuild 與 lintian 為 GNU hello 建立並驗證 Debian 原始碼套件
keywords:
  - Ubuntu
  - Debian
  - dh_make
  - sbuild
---

Debian package 打包是把上游原始碼加上 `debian/` 目錄，描述如何建置、相依與安裝，再產生 **source package**（`.dsc`）與 binary package（`.deb`）。
這篇以 GNU hello 2.12.1 為例，鎖定 Ubuntu 26.04（`resolute`）與 `amd64`，一步步完成：

1. 前置設定作業
2. 針對程式進行打包設定 (修改 `debian/`)
3. 驗證修改 (用 `sbuild` 和 `lintian` 確認有沒有問題)

完成後會得到可上傳的 `_source.changes`，接著請參考 [上傳套件到自己的 PPA](ppa.md)。

## 前置設定作業

### 安裝工具

```bash
sudo apt update
sudo apt install build-essential devscripts dh-make debhelper lintian sbuild schroot debootstrap ubuntu-keyring quilt wget
```

### 設定環境變數

先設定維護者資訊，請換成自己的姓名與已在 Launchpad 驗證的 Email：

```bash
export DEBFULLNAME="Your Name"
export DEBEMAIL="you@example.com"
```

建議將這些資訊放入 `~/.bashrc` 或 `~/.zshrc`。

## 針對程式進行打包設定

### 下載原始碼並產生 debian 目錄

`ftp.gnu.org` 在部分網路環境可能連線逾時，以下使用 kernel.org 的 GNU 鏡像站下載同一個版本。

```bash
mkdir -p ppa-work
cd ppa-work
wget https://mirrors.kernel.org/gnu/hello/hello-2.12.1.tar.gz
tar -xzf hello-2.12.1.tar.gz
cd hello-2.12.1
dh_make --single --copyright gpl3 --file ../hello-2.12.1.tar.gz
```

確認提示後，`dh_make` 會產生 `debian/` 樣板，以及上層目錄的 `hello_2.12.1.orig.tar.gz`。
`--single` 表示先產生單一二進位套件，`--copyright gpl3` 選擇 GPLv3 授權樣板；其他專案必須依實際授權調整。

### 修改打包設定

`dh_make` 產生的 `debian/` 是套件的建置與發佈設定。先了解各檔案用途，再修改和這個套件有關的部分：

| 檔案 | 用途 | 這個範例要做的事 |
| --- | --- | --- |
| `debian/control` | 原始碼套件、二進位套件及相依套件的描述 | 填入套件資訊，列出建置所需的 `Build-Depends`。 |
| `debian/changelog` | 套件版本、目標 Ubuntu 發行版與修改紀錄 | 建立 `resolute` 版本項目；每次上傳都要使用新版本號。 |
| `debian/rules` | 定義 debhelper 如何清理、設定、建置與測試 | 使用 `dh $@`，並略過沒有 `Makefile` 時不適用的清理。 |
| `debian/source/format` | 原始碼套件格式 | 保留 `3.0 (quilt)`，讓 dpkg-source 套用 Debian 補丁。 |
| `debian/copyright` | 上游檔案的版權與授權資訊 | 依 GNU hello 的 `COPYING` 填妥，不要保留未完成的樣板。 |
| `debian/patches/series` 與補丁檔 | 記錄要套用到上游原始碼的修改及順序 | 用 Quilt 保存 gettext 版本修正。 |
| `debian/*.ex`、`debian/*.EX` | 可選的範例設定檔 | 不使用的樣板可刪除；要使用則先改成有效設定。 |

#### `debian/control`

將 `debian/control` 調整為以下內容，維護者請換成自己的資訊：

```text
Source: hello
Section: devel
Priority: optional
Maintainer: Your Name <you@example.com>
Build-Depends: debhelper-compat (= 13), texinfo, help2man
Standards-Version: 4.7.3
Homepage: https://www.gnu.org/software/hello/
Rules-Requires-Root: no

Package: hello
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}
Description: GNU Hello demonstration program
 GNU Hello prints a greeting and demonstrates GNU coding conventions.
```

`Build-Depends` 是編譯需要的套件，由 `sbuild` 安裝到建置環境；`Depends` 是使用者安裝時需要的執行相依套件。
`${shlibs:Depends}` 與 `${misc:Depends}` 由打包工具計算，不需手動換成套件名稱。
GNU hello 的文件建置與測試會用到 `makeinfo`，此程式由 `texinfo` 套件提供；請將它列在 `Build-Depends`，而不是只安裝在主機上。
`help2man` 用來產生 man page。

#### `debian/changelog`

在原始碼目錄建立目標 Ubuntu 版本的項目：

```bash
dch -b --newversion 2.12.1-0ubuntu1~resolute1 --distribution resolute
```

`-b` 允許新版號低於 `dh_make` 產生的初始版本。PPA 不允許以相同版本號覆蓋既有套件，因此每次重新上傳都要增加版本號。

`dch` 會開啟編輯器，並直接改寫 `dh_make` 產生的第一個項目。請把樣板中的 `Initial release. (Closes: #nnnn)  <nnnn is the bug number of your ITP>` 改成實際說明，例如 `Initial release.`，否則 Lintian 會回報 `changelog-is-dh_make-template`。儲存後，第一行應類似：

```text
hello (2.12.1-0ubuntu1~resolute1) resolute; urgency=medium
```

Debian 版號格式為 `[epoch:]upstream_version-debian_revision`：

* `epoch`：可選欄位。上游改版導致新版號排序反而較舊時，用它調整排序，例如 `10.0-1` 改成 `1:2.0-1`。
* `upstream_version`：上游版本，例如 `2.12.1`。
* `debian_revision`：套件維護者的修訂版。Ubuntu 直接打包時常用 `0ubuntu1`；若從 Debian 修訂版 `1` 延伸，常見形式是 `1ubuntu1`。
* `~resolute1`：可選的 PPA 後綴，用來標示 Resolute 的 PPA 修訂序號；同一 suite 後續上傳可遞增為 `~resolute2`。目標 suite 本身由 changelog 的 `resolute` distribution 欄位指定。

版號後綴會影響 APT 的版本排序：`~` 排在沒有後綴的版號之前，`+` 則排在它之後。因此不要直接把 `~resolute1` 加到已存在的 `0.7.0-1ubuntu2` 後面，因為 `0.7.0-1ubuntu2~resolute1` 會比官方版舊。若 Resolute 官方版是 `0.7.0-1ubuntu2`，PPA 測試版可用 `0.7.0-1ubuntu2+resolute1`；或提高 Ubuntu 修訂號後使用 `0.7.0-1ubuntu3~resolute1`。PPA 後續更新要遞增自己的序號，不能重用已上傳的同版號。

#### `debian/copyright`

這個檔案使用 DEP-5 格式，逐組記錄檔案的著作權人與授權。

先從 `AUTHORS`、`COPYING` 和各檔案的授權標頭確認資訊，再移除其他檔案內原始的 comment，否則 lintian 會報錯。

```text
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Source: https://ftp.gnu.org/gnu/hello/
Upstream-Name: GNU hello

Files:
 *
Copyright: 1992-2022 Free Software Foundation, Inc.
License: GPL-3.0+

Files: debian/*
Copyright: 2026 Your Name <you@example.com>
License: GPL-3.0+

License: GPL-3.0+
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 .
 On Debian systems, the complete text of the GNU General Public
 License version 3 can be found in "/usr/share/common-licenses/GPL-3".
```

#### `debian/rules`

GNU hello 使用標準的 configure 與 make 流程。
上游的 `GNUmakefile` 在 `./configure` 尚未產生 `Makefile` 時也存在，若直接使用預設的 `dh_auto_clean`，初次清理可能會誤執行 `make distclean` 而失敗。
因此在 `debian/rules` 加上條件：

<!-- markdownlint-disable MD010 -->

```makefile
#!/usr/bin/make -f

override_dh_auto_clean:
	if [ -f Makefile ]; then dh_auto_clean; fi

%:
	dh $@
```

<!-- markdownlint-enable MD010 -->

recipe（`override_dh_auto_clean:` 與 `%:` 下面的指令）前面必須是 Tab，不是空白；用空白會出現 `missing separator` 錯誤。
另外可以把既有的 comments 都刪除，不然後續的 lintian 會有 Error 錯誤出現。

最後確認 `debian/rules` 有執行權限：

```bash
chmod +x debian/rules
```

#### 修正 gettext 範本版本

GNU hello 2.12.1 的 `configure.ac` 宣告 gettext 0.18.1，但封存檔內已產生的 `configure` 與 `po/Makefile.in.in` 使用 0.20。
Debhelper 執行 `dh_autoreconf` 時，會依照 `configure.ac` 重新產生 gettext 檔案，造成版本不一致並在 `po/` 建置失敗。
Quilt 用來管理對上游原始碼的修改，會將修改記錄成 `debian/patches/` 下的補丁；`3.0 (quilt)` 格式會在建置時套用這些補丁。
以下建立一個補丁，將版本宣告修正為 0.20：

```bash
# 建立存放補丁的目錄。
mkdir -p debian/patches
# 指定 Quilt 使用 Debian 套件慣例的補丁目錄。
export QUILT_PATCHES=debian/patches
# 開始建立新的 gettext 版本修正補丁。
quilt new gettext-macro-version.patch
# 告訴 Quilt 接下來要修改 configure.ac。
quilt add configure.ac
# 將 gettext 版本宣告從 0.18.1 改為 0.20。
sed -i 's/AM_GNU_GETTEXT_VERSION(\[0\.18\.1\])/AM_GNU_GETTEXT_VERSION([0.20])/' configure.ac
# 將檔案變更寫入補丁，並更新補丁清單。
quilt refresh
```

確認 `debian/patches/series` 列有 `gettext-macro-version.patch`，且補丁只把 `AM_GNU_GETTEXT_VERSION([0.18.1])` 改成 `AM_GNU_GETTEXT_VERSION([0.20])`。

#### 其他檔案

`debian/` 內還有很多其他範例，例如 `*.ex`（其中 `upstream.ex` 是目錄）或是 `README.*`，如果沒有用到可以先刪除。請在原始碼目錄執行：

```bash
rm -rf debian/*.ex debian/*.EX debian/README.*
```

## 驗證修改

### 設定 sbuild 乾淨環境

以下使用 schroot backend。將使用者加入 `sbuild` 群組：

```bash
sudo sbuild-adduser "$USER"
```

登出再登入，讓群組設定生效。接著建立 Resolute 的建置環境，這個步驟只需做一次，會下載套件並使用磁碟空間：

```bash
sudo mkdir -p /srv/chroot
# 建立 Resolute amd64 的 sbuild chroot tarball。
sudo sbuild-createchroot --arch=amd64 --components=main,universe \
  --keyring=/usr/share/keyrings/ubuntu-archive-keyring.gpg \
  --make-sbuild-tarball=/srv/chroot/resolute-amd64-sbuild.tar.gz \
  resolute /srv/chroot/resolute-amd64-sbuild http://archive.ubuntu.com/ubuntu
# 列出 schroot 已設定的環境，確認 Resolute chroot 已建立。
schroot --list
```

確認清單中有 `resolute-amd64-sbuild` 對應的 chroot。打包與建置操作使用一般使用者，不要執行 `sudo sbuild`。
此範例不涵蓋跨架構本地建置；ARM 主機需使用適當的架構與 Ubuntu ports mirror，不能只照抄 `amd64` 設定。

### 產生原始碼套件並使用 sbuild 測試

重新進入原始碼目錄，先產生尚未簽署的原始碼套件：

```bash
cd ppa-work/hello-2.12.1
# 產生未簽署的原始碼套件(副檔名為.dsc)，包含上游原始碼封存檔。
dpkg-buildpackage -S -sa -us -uc -d
# 要在原始碼的資料夾外執行 sbuild
cd ..
# 在 Resolute chroot 中安裝建置相依套件並編譯來源套件。
sbuild --chroot-mode=schroot -d resolute hello_2.12.1-0ubuntu1~resolute1.dsc
```

* `-S -sa`：產生原始碼套件，並包含第一次上傳需要的上游封存檔。
* `-us -uc`：先不簽署，等本地測試完成後再簽署。
* `-d`：此處略過主機的建置相依檢查；真正的相依安裝與編譯由 `sbuild` 在 chroot 中進行。

`sbuild` 會讀取 `.dsc`，在乾淨環境中安裝 `Build-Depends` 並編譯。
以此指令執行時，產物位於目前的 `~/ppa-work/`，包含 `.deb`、`_amd64.changes` 與 build log。
確認 log 最後顯示 `Status: successful`；失敗時先修正打包設定，再重新產生原始碼套件與測試。

### Lintian 檢查

在 `~/ppa-work/` 執行：

```bash
# 檢查 sbuild 產生的二進位套件與套件資訊。
lintian -I -i --pedantic hello_2.12.1-0ubuntu1~resolute1_amd64.changes
# 檢查 source package、debian/ 設定及原始碼檔案。
lintian -I -i --pedantic hello_2.12.1-0ubuntu1~resolute1_source.changes
```

優先修正 `E:`（錯誤）；逐項檢查 `W:`（警告），修正問題或確認並記錄合理原因。
`I:` 是資訊提示，`P:` 是較嚴格的建議；不一定都要修改，但仍應確認沒有隱藏實際問題。
修改 `debian/` 後，重新產生原始碼套件、執行 `sbuild` 與 Lintian，避免上傳與測試不同的內容。

### 產生的檔案說明

在執行過程中，你會發現程式碼資料夾外有多種產物，下面解釋各個檔案的意義：

| 檔案 | 產生指令 | 用途 |
| --- | --- | --- |
| `hello-2.12.1.tar.gz` | `wget` | 下載的上游原始碼。 |
| `hello_2.12.1.orig.tar.gz` | `dh_make --file` | dpkg source package 使用的上游原始碼封存檔。 |
| `hello_<version>.debian.tar.xz` | `dpkg-buildpackage -S` | Debian 打包設定與補丁。 |
| `hello_<version>.dsc` | `dpkg-buildpackage -S` | Source package 描述檔，列出檔案及校驗資訊；sbuild 以此為輸入。 |
| `hello_<version>_source.changes` | `dpkg-buildpackage -S` | Source package 上傳清單；`dput` 使用此檔上傳。 |
| `hello_<version>_source.buildinfo` | `dpkg-buildpackage -S` | Source package 的建置環境資訊。 |
| `hello_<version>_amd64.deb` | `sbuild` | 本機建出的可安裝套件。 |
| `hello-dbgsym_<version>_amd64.ddeb` | `sbuild` | 除錯符號套件，通常只有除錯時才需要。 |
| `hello_<version>_amd64.changes` | `sbuild` | 本機 binary build 產物清單；不要用它上傳 PPA。 |
| `hello_<version>_amd64.buildinfo` | `sbuild` | Binary build 的建置環境資訊。 |
| `hello_<version>_amd64.build` | `sbuild` | 建置記錄；同名連結通常指向最近一次的 log。 |

#### 清理產物

在原始碼目錄執行以下指令，可清除建置中間檔，但保留 `debian/` 設定：

```bash
fakeroot debian/rules clean
```

若要清除 `ppa-work/` 中某個版本的打包產物，先進入該目錄，再使用精確的版本字串；以下只刪 Resolute 這個版本，會保留上游 tarball 和 `hello-2.12.1/` 原始碼目錄：

```bash
cd ~/ppa-work
rm -f hello_2.12.1-0ubuntu1~resolute1* \
  hello-dbgsym_2.12.1-0ubuntu1~resolute1*
```

## 參考資料

* [GNU hello](https://www.gnu.org/software/hello/)
* [sbuild 使用手冊](https://manpages.ubuntu.com/manpages/resolute/man1/sbuild.1.html)
* [sbuild-createchroot 使用手冊](https://manpages.ubuntu.com/manpages/resolute/man8/sbuild-createchroot.8.html)
