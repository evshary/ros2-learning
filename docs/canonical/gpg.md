---
title: GPG 金鑰管理入門
description: 在 Ubuntu 上建立、查看與備份 GPG 金鑰
keywords:
  - Linux
  - Ubuntu
  - GPG
---

GPG（GNU Privacy Guard）可以用來加密資料與產生數位簽章。
在 Ubuntu 開發工作中，也常用來簽署套件或 Git commit。
GPG 並非 Canonical 專屬工具，這篇先介紹在 Ubuntu 上管理個人金鑰的基本操作。

## 公鑰與私鑰

* **公鑰**：可以分享給別人，用來驗證你的簽章，或加密要傳給你的資料。
* **私鑰**：只能自己保管，用來簽章或解密。不要上傳到 Git repository，也不要傳給別人。
* **指紋（fingerprint）**：用來辨識金鑰的完整識別碼，確認公鑰來源時應比對完整指紋。

以下指令中的 `YOUR_FINGERPRINT` 請換成自己金鑰的完整指紋。

## 安裝與建立金鑰

安裝 GPG：

```bash
sudo apt update
sudo apt install gnupg
```

建立金鑰：

```bash
gpg --full-generate-key
```

依照提示設定：

1. 金鑰類型與大小可以先使用預設值。
2. 有效期限輸入 `1y`，代表一年。
3. 填入姓名與 Email，註解可以留空。
4. 設定一組不易猜測的密碼（passphrase），保護私鑰。

金鑰通常儲存在 `~/.gnupg/`，不要將這個目錄分享出去。

## 查看金鑰

列出公鑰與完整指紋：

```bash
gpg --list-keys
```

列出本機的私鑰：

```bash
gpg --list-secret-keys
```

輸出中的金鑰標記：

* `pub`：主金鑰的公鑰。
* `sec`：主金鑰的私鑰。
* `sub`：子金鑰的公鑰。
* `ssb`：子金鑰的私鑰。

主金鑰用來管理身分與子金鑰；子金鑰則可分別用於加密或簽章。

找到自己姓名與 Email 對應的金鑰，使用 `pub` 或 `sec` 下方顯示的完整指紋。

## 發佈公鑰到 keyserver

可以將公鑰上傳到 Ubuntu keyserver，讓別人透過完整指紋取得，不必直接傳送檔案。
上傳前請確認願意公開金鑰中的姓名與 Email；資料可能被複製或長期保留，不應預期能完全刪除。

上傳自己的公鑰（不會上傳私鑰）：

```bash
gpg --keyserver hkps://keyserver.ubuntu.com --send-keys YOUR_FINGERPRINT
```

對方透過可信的另一個管道確認你的完整指紋後，可以下載並匯入公鑰：

```bash
gpg --keyserver hkps://keyserver.ubuntu.com --recv-keys YOUR_FINGERPRINT
gpg --list-keys --fingerprint YOUR_FINGERPRINT
```

下載後再次比對完整指紋。Keyserver 只是公鑰的存放處，不保證金鑰持有人的身分。
日後若更新有效期限或撤銷金鑰，請再次執行 `--send-keys` 發佈更新。
已匯入公鑰的人可用以下指令取得更新，將指紋換成要更新的金鑰：

```bash
gpg --keyserver hkps://keyserver.ubuntu.com --refresh-keys YOUR_FINGERPRINT
```

## 匯出與匯入公鑰

若不使用 keyserver，也可以將自己的公鑰匯出成文字格式，這個檔案可以分享給別人：

```bash
gpg --armor --output public-key.asc --export YOUR_FINGERPRINT
```

收到別人的公鑰時，先查看內容：

```bash
gpg --show-keys --fingerprint public-key.asc
```

透過可信的另一個管道與對方確認完整指紋，例如當面確認，再匯入：

```bash
gpg --import public-key.asc
```

匯入成功只代表公鑰已加入本機，不代表它一定屬於聲稱的持有人。

## 備份私鑰

以下指令先限制新檔案的權限，再匯出私鑰。請在自己的終端機操作：

```bash
umask 077
gpg --armor --output private-key-backup.asc --export-secret-keys YOUR_FINGERPRINT
```

`umask 077` 會持續影響這個 shell 建立的新檔案；完成後可以關閉該終端機。
將備份存放在受保護的離線儲存裝置，確認備份完成後移除本機暫存副本。
私鑰備份即使有密碼保護，也不能公開或提交到 Git。

在另一台可信任的電腦安裝 GPG 後，可以用以下指令還原：

```bash
gpg --import private-key-backup.asc
gpg --list-secret-keys --fingerprint
```
