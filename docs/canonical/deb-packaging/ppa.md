---
title: 上傳套件到自己的 PPA
description: 使用 GPG、debsign 與 dput 將 Debian 原始碼套件上傳到 Launchpad PPA
keywords:
  - Ubuntu
  - Launchpad
  - PPA
---

PPA（Personal Package Archive）是 Launchpad 提供的個人套件庫。
我們把簽署過的 **Debian 原始碼套件**上傳，Launchpad 會在指定的 Ubuntu 環境中建置 `.deb`，讓使用者可以透過 APT 安裝。
因此，不能直接把本機編好的 `.deb` 上傳到 PPA。
值得一提的是 PPA 可以同時包含不同版本 Ubuntu 的套件，例如 Resolute 和之後釋出的新版本。

這篇接續 [Debian package 打包入門](packaging.md)，假設已在 `~/ppa-work/` 得到 GNU hello 的 `hello_2.12.1-0ubuntu1~resolute1_source.changes`，一步步教導使用者如何上傳：

1. 建立自己的 PPA
2. 前置設定作業
3. 簽署並上傳到 PPA
4. 安裝與測試
5. 加入最新的 Ubuntu release

以下鎖定 Ubuntu 26.04（`resolute`）與 `amd64` 架構。

## 建立自己的 PPA

1. 登入 [Launchpad](https://launchpad.net/)，確認帳號的 Email 已完成驗證。
2. 依照 [GPG 金鑰管理入門](../gpg.md) 建立金鑰，並將公鑰上傳到 Ubuntu keyserver。
3. 在個人頁面的 **OpenPGP keys** 設定中，填入完整指紋。依確認信指示，用私鑰解密訊息並完成驗證。
4. 回到個人頁面，建立新的 PPA，例如名稱設為 `myppa`。

假設 Launchpad 帳號名稱是 `YOUR_LAUNCHPAD_ID`，PPA 頁面會是：

```text
https://launchpad.net/~YOUR_LAUNCHPAD_ID/+archive/ubuntu/myppa
```

一般個人 PPA 是公開的，請勿上傳機密原始碼，也要確認授權允許公開散佈。
這裡使用的是 Launchpad 帳號名稱，不是 Email 或顯示名稱。

## 前置設定作業

### 安裝工具

```bash
sudo apt update
sudo apt install devscripts dput gnupg
```

上傳時簽署套件的 GPG 金鑰，必須已登錄到你的 Launchpad 帳號。

### 設定 ~/.devscripts

`~/.devscripts` 是 `devscripts` 工具的使用者設定檔，例如 `dch`、`debsign` 會讀取它。
若希望簽署時自動使用固定金鑰，可以建立或編輯這個檔案，加入：

```bash
DEBSIGN_KEYID="YOUR_FINGERPRINT"
```

將 `YOUR_FINGERPRINT` 換成已在 Launchpad 驗證的完整 GPG 指紋。儲存後，下次執行 `debsign` 就會讀取，不需要手動 `source`。
例如在後續簽署步驟中，可以省略 `-k`：

```bash
debsign hello_2.12.1-0ubuntu1~resolute1_source.changes
```

明確指定 `debsign -kYOUR_FINGERPRINT` 則會覆蓋預設金鑰。
這不是所有打包工具共用的設定檔，`sbuild` 的個人設定放在 `~/.sbuildrc`。

## 簽署並上傳到 PPA

將 `YOUR_FINGERPRINT` 換成自己已在 Launchpad 驗證的 GPG 完整指紋：

```bash
debsign hello_2.12.1-0ubuntu1~resolute1_source.changes
# 如果沒有放 GPG key 在 .devscripts 的話
# debsign -kYOUR_FINGERPRINT hello_2.12.1-0ubuntu1~resolute1_source.changes
dput ppa:YOUR_LAUNCHPAD_ID/myppa hello_2.12.1-0ubuntu1~resolute1_source.changes
```

`debsign` 會簽署 `.changes` 及相關的 `.dsc` 等檔案，可能要求輸入私鑰密碼。
`dput` 會依 `.changes` 上傳相關檔案，不需要逐一上傳 `.dsc` 或封存檔。
PPA 只接受原始碼上傳，請選 `_source.changes`，不要選本地測試產生的 `_amd64.changes`。

上傳完成不代表已經建置成功。
Launchpad 處理後會寄送接受或拒絕通知，可以在 PPA 頁面查看套件與建置狀態。
若建置失敗，打開 build log，檢查相依套件、編譯錯誤與目標 Ubuntu 版本，再提高版本號重新上傳。

## 安裝與測試

接著就要等套件建置成功且發布，可以到自己的 PPA 頁面點選 "View package details"。
值得注意的是，第一次發佈可能要等久一點（可能一個多小時），在發佈之前 `apt update` 都會是 403 錯誤，而且 PPA 的金鑰也無法正常匯入。

發佈以後，在對應 Ubuntu 版本的測試機上執行：

```bash
sudo add-apt-repository ppa:YOUR_LAUNCHPAD_ID/myppa
sudo apt update
apt policy hello
sudo apt install hello=2.12.1-0ubuntu1~resolute1
hello
```

這個練習沿用 Ubuntu 已有的 `hello` 套件名稱，會取代測試機上的同名套件，建議使用 VM。
`apt policy` 可確認 PPA 的版本是否已出現；指定版本安裝可避免誤測 Ubuntu 官方來源的套件。
加入 PPA 會讓 APT 從該來源取得套件與後續更新，只加入自己信任的 PPA。
上傳簽章使用你的個人 GPG 金鑰；APT 驗證套件庫使用的是 Launchpad 為 PPA 管理的簽署金鑰，兩者不同。

測試完成後記得要移除。

```bash
sudo apt purge hello
sudo add-apt-repository --remove ppa:YOUR_LAUNCHPAD_ID/myppa
sudo apt update
```

### 新增最新的 Ubuntu release

Ubuntu 每半年釋出新版，PPA 不會自動跟進，必須主動為新的 series 再上傳一次。
先確認要鎖定的 codename：

```bash
# 最新已釋出的版本，以及目前開發中的版本（需要 distro-info 套件）。
ubuntu-distro-info --latest
ubuntu-distro-info --devel
```

以 2026 年 11 月為例，最新釋出的是 `stonking`（26.10），開發中的是 `tenacious`（27.04）。
Launchpad 允許上傳到開發中的 series，但上傳前請先確認 codename 已出現在 [Ubuntu 的 Launchpad 頁面](https://launchpad.net/ubuntu)。

新 series 的流程與 Resolute 相同，只有以下三處不同：

* **`debian/changelog`（必改）**：新增項目，`distribution` 填新的 codename，版號後綴改成新 codename 並從 `1` 開始，例如 `2.12.1-0ubuntu1~stonking1`。Ubuntu 的 codename 依字母順序遞增，因此 `~resolute1` < `~stonking1` < `~tenacious1`，使用者升級 Ubuntu 後 APT 會自然選到較新的版本。
* **sbuild chroot（必建一次）**：為新 series 建立另一個 chroot，把 `resolute` 換成新 codename。
* **`debian/control`、`debian/rules`、`debian/patches/`（有需要才改）**：只有新 release 的相依套件名稱或版本、工具鏈（例如 GCC、Python）造成建置失敗時才調整，一般先沿用原本內容。

```bash
# 建立新 series 的 sbuild chroot；把 stonking 換成要上傳的 codename。
sudo sbuild-createchroot --arch=amd64 --components=main,universe \
  --keyring=/usr/share/keyrings/ubuntu-archive-keyring.gpg \
  --make-sbuild-tarball=/srv/chroot/stonking-amd64-sbuild.tar.gz \
  stonking /srv/chroot/stonking-amd64-sbuild http://archive.ubuntu.com/ubuntu
```

```bash
# 在原始碼目錄新增新 series 的 changelog 項目。
dch --newversion 2.12.1-0ubuntu1~stonking1 --distribution stonking
dpkg-buildpackage -S -sa -us -uc -d
cd ..
sbuild --chroot-mode=schroot -d stonking hello_2.12.1-0ubuntu1~stonking1.dsc
```

sbuild 成功後，照前面的方式對新的 `_source.changes` 執行 Lintian、簽署並上傳。
每個 series 都是獨立的 source upload；上游封存檔 `.orig.tar.gz` 已存在於 PPA，後續上傳可以改用 `-sd`，只送出 `debian/` 的差異。
若不需要改任何打包設定，也可以在 PPA 頁面使用 **Copy packages**，把既有的 source package 複製到新 series 並重新建置，不必重新簽署上傳。

### 多架構

在 PPA 頁面的架構設定確認可用的 architectures。
`Architecture: any` 的套件會依 PPA 啟用且支援的架構安排建置；部分架構可能需要額外申請，不保證所有 PPA 都能使用。
本機以 sbuild 驗證時，也要為對應架構建立 chroot；ARM 主機需使用適當的 Ubuntu ports mirror，不能只照抄 `amd64` 設定。

## 參考資料

* [Launchpad：上傳套件到 PPA](https://help.launchpad.net/Packaging/PPA/Uploading)
