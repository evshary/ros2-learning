---
title: 使用 ubuntu-image 建置自訂 gadget 的 Ubuntu Core 映像
description: 修改 pc gadget Snap 的分割區配置，建立並簽署自己的 model，用 ubuntu-image 建置 Ubuntu Core 映像並在 QEMU 驗證
keywords:
  - ubuntu-image
  - Ubuntu Core
  - gadget
  - model assertion
  - snap sign
  - QEMU
---

本文延續 [建置 Core 映像](core.md)，在已完成的 x86-64 Core 26 範例上自訂 gadget。
請先完成該頁的前置設定，並使用相同的工作目錄 `~/ubuntu-core-vm`；文中「前面」的 QEMU 配置與 SSH 金鑰也都指該頁的內容。

若要調整分割區大小或開機資產，需要修改 **`pc` gadget Snap** 的 `meta/gadget.yaml`，重新打包後放入映像。
本節會建立並簽署自己的 model，以允許使用本機 gadget，再建置映像並在 VM 驗證。
範例會將官方 gadget 的 `ubuntu-save` 調整為 64 MiB，重新打包成 `pc-custom.snap`，再使用 `--snap` 取代 Store 的 `pc` 套件。

Ubuntu Core 要求 model 的來源可被驗證，因此透過 Snapcraft 取得帳號 ID，並建立、註冊簽署公鑰。
實際簽署由本機的 `snap sign` 完成；model 不需要上傳，私鑰也留在主機。

model 只要有被修改就需要重新簽署，但是有些情況不需要重新簽署，例如：

1. model 內已經有宣告 optional snap，我們只需要用 --snap 加入就可以
2. model 有用 dangerous，可以用本機同名 snap 替換原套件或是額外加入本機的 application snap

## 查詢 developer ID

先在 [Snapcraft](https://snapcraft.io/account) 登入 Ubuntu One 並接受服務條款，再於主機執行：

```bash
# 安裝簽署工具與 JSON 編輯工具
sudo snap install snapcraft --classic
sudo apt install jq

# 登入並保存憑證在主機的私人設定目錄，不放在專案內
mkdir -p ~/.config/snapcraft
previous_umask=$(umask)
umask 077
snapcraft export-login ~/.config/snapcraft/core-model.credentials
umask "$previous_umask"
export SNAPCRAFT_STORE_CREDENTIALS="$(cat ~/.config/snapcraft/core-model.credentials)"

# 輸出的 id 欄位就是 developer ID，不是 username 或 Email
snapcraft whoami
```

憑證檔與 `SNAPCRAFT_STORE_CREDENTIALS` 都是秘密，不要上傳、提交到 Git 或貼出內容。
後續的登入與金鑰 passphrase 提示，直接在本機終端輸入。

## 建立金鑰與自訂 model

以下將 `YOUR_DEVELOPER_ID` 換成 `whoami` 顯示的 `id`：

```bash
cd ~/ubuntu-core-vm
export DEVELOPER_ID="YOUR_DEVELOPER_ID"

# 建立簽署金鑰，再將公鑰註冊到自己的帳號；已有金鑰時可沿用
snapcraft create-key my-model-key
snapcraft register-key my-model-key
snapcraft keys

# 下載未簽署的 JSON 原稿，不修改 Canonical 已簽署的 .model
curl -fL -o reference-model.json \
  https://raw.githubusercontent.com/canonical/models/master/ubuntu-core-26-amd64.json

# 改成自己的品牌、model 名稱與開發等級，timestamp 設在金鑰註冊之後
jq --arg id "$DEVELOPER_ID" --arg timestamp "$(date -Iseconds --utc)" \
  '."authority-id" = $id | ."brand-id" = $id |
   .model = "my-core26-amd64" | .grade = "dangerous" |
   .timestamp = $timestamp' \
  reference-model.json > my-model.json

# 用自己的金鑰簽署，產生 ubuntu-image 使用的 model assertion
export GPG_TTY="$(tty)"
snap sign -k my-model-key my-model.json > my-model.model
```

`grade: dangerous` 允許未經 Store assertions 驗證的本機 Snap，但 **model 本身仍須簽署**。
這組簽署金鑰不是前面登入 VM 用的 SSH 金鑰；私鑰留在主機，註冊的是公鑰。

## 產生 pc-custom.snap

在 Ubuntu amd64 主機下載官方 Core 26 gadget，保留開機檔案，只修改分割區配置，不需要重新編譯 GRUB：

```bash
# 安裝解包工具與文字編輯器
sudo apt install squashfs-tools
cd ~/ubuntu-core-vm

# 下載官方 gadget，輸出為 pc-original.snap 與 pc-original.assert
snap download pc --channel=26/stable --basename=pc-original

# 解開到新目錄；pc-unpacked 不應是先前測試留下的目錄
unsquashfs -d pc-unpacked pc-original.snap

# 編輯 gadget 的磁碟配置
vim pc-unpacked/meta/gadget.yaml
```

找到 `name: ubuntu-save` 的項目，將其中的 `size: 32M` 改成 `size: 64M`，保留其他欄位與 YAML 縮排。
不要修改套件名稱 `pc` 或開機檔案，儲存後重新打包：

```bash
# 確保 Snap 內容可讀、目錄可進入，保留原有可執行檔的執行權限
chmod -R a+rX pc-unpacked

# 從解開的內容產生自訂 gadget Snap
snap pack pc-unpacked --filename=pc-custom.snap

# 確認後續建置使用的檔案已產生
ls -lh pc-custom.snap
```

修改後的套件不能沿用 `pc-original.assert`，因此搭配前面的 `dangerous` model 使用。
這是全新 VM 的分割區測試，不是調整既有設備分割區的更新流程。

## 放入本機 gadget 並建置

使用剛產生的 `pc-custom.snap` 建置映像：

```bash
# 使用本機 gadget 取代 Store 的 pc，並保留首次設定介面
mkdir -p custom-build
ubuntu-image snap \
  --image-size 8G \
  --output-dir custom-build \
  --snap console-conf \
  --snap ./pc-custom.snap \
  my-model.model
```

`dangerous` 用於開發測試，不應視為正式安全部署。

## 運行自訂映像

在主機的 `~/ubuntu-core-vm` 執行，磁碟副本與 UEFI VARS **只在建立新 VM 時準備一次**：

```bash
# 保留建置結果，另外準備測試磁碟與這台 VM 專用的 UEFI 設定
mkdir -p custom-vm
cp --sparse=always custom-build/pc.img custom-vm/ubuntu-core.img
cp /usr/share/OVMF/OVMF_VARS_4M.ms.fd custom-vm/OVMF_VARS.fd
```

沿用前面的 TPM-free 配置，改用獨立磁碟與 SSH port `8023`：

```bash
# 啟動自訂 VM；UEFI code 唯讀共用，VARS 與磁碟使用獨立副本
qemu-system-x86_64 \
  -enable-kvm -machine q35 -cpu host -smp 2 -m 2048 \
  -global ICH9-LPC.disable_s3=1 \
  -netdev user,id=net0,hostfwd=tcp:127.0.0.1:8023-:22 \
  -device virtio-net-pci,netdev=net0 \
  -drive file=/usr/share/OVMF/OVMF_CODE_4M.secboot.fd,if=pflash,format=raw,unit=0,readonly=on \
  -drive file=custom-vm/OVMF_VARS.fd,if=pflash,format=raw,unit=1 \
  -drive file=custom-vm/ubuntu-core.img,if=none,format=raw,id=disk1 \
  -device virtio-blk-pci,drive=disk1,bootindex=1 \
  -serial mon:stdio
```

完成 `console-conf` 的網路與 Ubuntu One 設定後，在主機另一個終端登入：

```bash
# 登入自訂 VM，使用新 VM 畫面顯示的使用者名稱
ssh -i ~/.ssh/id_ed25519 -p 8023 YOUR_USERNAME@127.0.0.1
```

## 驗證結果

以下在 **自訂 VM 內**執行：

```bash
# 確認 pc 為本機套件，revision 通常顯示 x1 等值
snap list pc

# 確認 model 為 my-core26-amd64、brand-id 為自己的 ID、grade 為 dangerous
snap model --assertion

# 確認已安裝的 gadget 包含自己修改的分割區大小與開機資產設定
cat /snap/pc/current/meta/gadget.yaml

# 比較實際分割區的標籤、大小、檔案系統與掛載位置
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS

# 完成驗證後正常關機
sudo poweroff
```

`x1` 只能確認使用了本機 revision，仍需檢查 gadget 內容與分割區結果。
本例的 `ubuntu-save` 應約為 64 MiB，gadget YAML 中的對應設定應為 `size: 64M`。
`ubuntu-data` 可能利用剩餘磁碟空間擴展，不一定等於 YAML 的初始大小。
下次只需重跑 QEMU 指令，不要覆蓋測試磁碟或 UEFI VARS。

## 參考資料

* [Add custom snaps](https://documentation.ubuntu.com/core/how-to-guides/image-creation/add-custom-snaps/)
* [Access Ubuntu One](https://documentation.ubuntu.com/core/tutorials/build-your-first-image/access-ubuntu-one/)
* [Sign the model](https://documentation.ubuntu.com/core/tutorials/build-your-first-image/sign-the-model/)
