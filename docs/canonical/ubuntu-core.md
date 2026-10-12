---
title: Ubuntu Core 入門
description: 介紹 Ubuntu Core 的 Snap 架構與儲存配置，並建置及使用 QEMU 執行 x86-64 映像
keywords:
  - Ubuntu Core
  - Snap
  - ubuntu-image
  - QEMU
  - Storage
---

Ubuntu Core 是 Canonical 針對 IoT 與嵌入式設備設計的 Ubuntu 發行版。
系統與應用程式以 Snap 管理，支援更新與版本回復，不使用一般 Ubuntu Server 的 APT 安裝流程。
系統與 Snap 本體主要唯讀，設定及應用程式資料則放在可寫入的儲存區。
Snap 的封裝與 sandbox 機制可先參考 [Snap 機制](snap.md)。

本文以 **Ubuntu Core 24、x86-64（`amd64`）** 為例，主機使用 Ubuntu 24.04 amd64，VM 使用 QEMU/KVM。

## 組成系統的 Snap

| 元件 | amd64 Core 24 範例 | 功能 |
| --- | --- | --- |
| Base | `core24` | 提供系統使用的基本 userspace 與函式庫 |
| Kernel | `pc-kernel`，`24/stable` | Linux kernel 與相關模組 |
| Gadget | `pc`，`24/stable` | 定義磁碟配置、bootloader、開機資產及設備設定 |
| snapd | `snapd` | 安裝、更新、驗證 Snap 與管理設備狀態 |
| Application | `console-conf` 等 | 設備服務及應用程式 |

**Model assertion** 是經過簽署的設備規格，指定架構與要安裝的 Snap，`ubuntu-image` 根據它組合磁碟映像
model 檔內會指定實際要用到的 gadget，裡面會紀錄真正的分割區配置。

## 儲存配置

以下是安裝完成後的四個主要分割區：

| 分割區 | 實際內容 | 用途 |
| --- | --- | --- |
| `ubuntu-seed` | 第一階段／recovery bootloader 資產、`snaps/` 中的初始 `.snap` 檔案，以及 `systems/<label>/` 中的 model、assertions 與 recovery 開機資訊 | 啟動安裝與 recovery 環境，提供重建系統所需的 Snap 與驗證資料 |
| `ubuntu-boot` | Run mode bootloader、開機設定，以及從 kernel snap 解出的 kernel 與 initramfs 等資產 | 載入正常運行的 kernel，啟動已安裝的系統 |
| `ubuntu-save` | 設備身分備份，以及 Snap 透過 `$SNAP_SAVE_DATA` 明確保存的少量設備資料，例如憑證與設備設定 | 在 factory reset 或重新安裝時保留指定的設備資料，不是一般使用者資料備份區 |
| `ubuntu-data` | 已安裝的 `.snap` 檔案、snapd 狀態、系統可寫設定、應用程式資料與使用者家目錄 | 保存日常運行的系統狀態與資料，提供唯讀 root filesystem 所需的可寫目錄 |

`ubuntu-seed` 在 UEFI 系統上是 FAT 格式的 EFI System Partition（ESP）。
`ubuntu-data` 和 `ubuntu-save` 是可寫資料區；`ubuntu-save` 在未加密設備上可省略，但加密設備必須有它。
實際大小與順序依 gadget 而定，尚未安裝的映像不一定已包含全部分割區。

## 開機流程

### 首次開機：install mode

1. UEFI 從 `ubuntu-seed` 啟動 bootloader，載入 seed 內 recovery system 的 kernel 與 initramfs。
2. 系統在記憶體中的暫時環境（tmpfs）進入 install mode。
3. snapd 依 gadget 建立缺少的分割區，在 `ubuntu-boot` 準備正常開機資產，在 `ubuntu-data` 準備系統資料，並初始化需要的 `ubuntu-save`。
4. 重新啟動進入 run mode，完成 Snap seeding 與首次使用者設定。

### 日常開機：run mode

```mermaid
flowchart TD
  firmware[UEFI] --> seed["ubuntu-seed：第一階段 bootloader"]
  seed --> boot["ubuntu-boot：run mode bootloader、kernel、initramfs"]
  boot --> bootstrap["initramfs：snap-bootstrap"]
  data["ubuntu-data：已安裝 Snap 與可寫資料"] --> bootstrap
  save["ubuntu-save：設備身分備份與持久資料（若有）"] -.-> bootstrap
  bootstrap --> root["掛載 base snap 作為唯讀 root，接上可寫目錄"]
  root --> services["啟動 systemd、snapd 與應用程式"]
```

`snap-bootstrap` 負責掛載所需分割區、選擇 base 與 kernel Snap；加密配置還需要先解鎖資料區。
`ubuntu-save` 不是另一個 bootloader 階段，而是掛載的輔助資料區。
Root filesystem 來自 base snap，不是直接把 `ubuntu-data` 當成整個 `/`。
若進入 recovery，則改用 `ubuntu-seed` 的 recovery system 與暫時可寫環境，不走一般 run mode 的系統啟動路徑。

## 主機前置設定

以下指令在 **Ubuntu 主機** 執行。本例分配 2 GiB RAM 與 8 GiB 虛擬磁碟，並需要網際網路連線。

```bash
# 更新套件索引，安裝 QEMU、UEFI firmware 與 KVM 檢查工具
sudo apt update
sudo apt install qemu-system-x86 ovmf cpu-checker curl

# 安裝映像製作工具；--classic 允許存取主機檔案與資源
sudo snap install ubuntu-image --classic

# 檢查主機是否可使用 KVM 硬體虛擬化加速
kvm-ok
```

QEMU 提供虛擬機與虛擬裝置，KVM 則透過 Linux kernel 加速 guest CPU 執行。

確認 `kvm-ok` 可使用 KVM，且使用者有 `/dev/kvm` 存取權限。
若缺少 `kvm` 群組權限，可執行 `sudo usermod -aG kvm "$USER"`，再登出並重新登入。
其中 `-aG` 將目前使用者加入額外群組，不移除原有群組；`$USER` 是目前使用者名稱。

另外我們需要先建立 [Ubuntu One 帳號](https://login.ubuntu.com/)，並在 [SSH keys](https://login.ubuntu.com/ssh-keys) 登錄 SSH **公鑰**，供首次設定與 SSH 登入使用。
這個和 Desktop / Server 的邏輯不一樣，因為 Ubuntu Core 通常是大量部屬，利用 Ubuntu One 就可以避免每次安裝都要設定帳號密碼的資訊。

## 建置 x86-64 映像

使用 Canonical 已簽署的參考 model，不需要自己的簽署金鑰：

```bash
mkdir -p ~/ubuntu-core-vm
cd ~/ubuntu-core-vm

# 下載已簽署的 model，也就是前面提過的 model assertion
# -f 檢查 HTTP 錯誤，-L 跟隨重新導向，-o 指定檔名
curl -fL -o ubuntu-core-24-amd64.model \
  https://raw.githubusercontent.com/canonical/models/master/ubuntu-core-24-amd64.model
mkdir -p build

# 依 model 組合 8 GiB 映像到 build/，並加入首次設定用的 console-conf
ubuntu-image snap \
  --image-size 8G \
  --output-dir build \
  --snap console-conf \
  ubuntu-core-24-amd64.model

# 複製成 VM 工作磁碟；--sparse=always 減少連續零值區域的實際占用
cp --sparse=always build/pc.img ubuntu-core.img
```

輸出為 `build/pc.img`，這裡複製成 `ubuntu-core.img` 供 VM 使用。
Core 24 的參考 model 將 `console-conf` 標為 optional，因此需要明確加入。

## 在 QEMU 上執行

### 準備 UEFI firmware

在同一個工作目錄執行，**每個新 VM 各做一次**：

```bash
# 複製唯讀 UEFI code 到目前目錄 (UEFI 程式碼的部份，用來初始化)
cp /usr/share/OVMF/OVMF_CODE_4M.secboot.fd .

# 建立這台 VM 專用的可寫 UEFI variables (UEFI 的設定部份，如開機順序)
cp /usr/share/OVMF/OVMF_VARS_4M.ms.fd .
```

同一個 VM 重啟時沿用原來的 VARS，不要每次重新覆蓋。

### 啟動 VM

以下使用沒有 TPM 的配置，不測試磁碟加密；透過 QEMU 圖形視窗操作首次設定：

```bash
# 使用 KVM、Q35 與主機 CPU 功能，分配 2 個 vCPU、2048 MiB RAM，停用 S3 睡眠
# virtio 網卡使用 NAT，將主機 127.0.0.1:8022 轉送到 VM 的 SSH port 22
# pflash 掛載唯讀 UEFI code 與可寫 variables；virtio 磁碟使用 raw 映像
# serial console 與 QEMU monitor 共用目前終端
qemu-system-x86_64 \
  -enable-kvm \
  -machine q35 \
  -cpu host \
  -smp 2 \
  -m 2048 \
  -global ICH9-LPC.disable_s3=1 \
  -netdev user,id=net0,hostfwd=tcp:127.0.0.1:8022-:22 \
  -device virtio-net-pci,netdev=net0 \
  -drive file=OVMF_CODE_4M.secboot.fd,if=pflash,format=raw,unit=0,readonly=on \
  -drive file=OVMF_VARS_4M.ms.fd,if=pflash,format=raw,unit=1 \
  -drive file=ubuntu-core.img,if=none,format=raw,id=disk1 \
  -device virtio-blk-pci,drive=disk1,bootindex=1 \
  -serial mon:stdio
```

這是 raw 磁碟映像，不是 ISO。同一個映像不能同時交給兩個 VM 使用。

### 首次啟動與 SSH

1. 等待首次安裝與重新啟動完成。
2. 在 QEMU 視窗的 `console-conf` 設定網路，輸入已登錄 SSH 公鑰的 Ubuntu One 帳號 Email。
3. 記下畫面顯示的本機使用者名稱，從主機登入：

```bash
# -i 指定 SSH 私鑰，-p 使用主機轉送到 VM 的 port
ssh -i ~/.ssh/id_ed25519 -p 8022 YOUR_USERNAME@127.0.0.1
```

`YOUR_USERNAME` 是本機使用者名稱，不是 Email；私鑰路徑請依自己的設定調整。

登入後可在 **VM 內**確認版本與配置：

```bash
# 列出已安裝的 Snap 與 revisions
snap list

# 顯示設備使用的完整 model assertion
snap model --assertion

# 列出磁碟與分割區；-o 指定名稱、大小、檔案系統、標籤與掛載位置
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS
```

完成後用 `sudo poweroff` 關機；下次執行相同 QEMU 指令即可。

## 延伸：自訂 gadget Snap

若要調整分割區大小或開機資產，需要修改 **`pc` gadget Snap** 的 `meta/gadget.yaml`，重新打包後放入映像。
本節會建立並簽署自己的 model，以允許使用本機 gadget，再建置映像並在 VM 驗證。
範例會將官方 gadget 的 `ubuntu-save` 調整為 64 MiB，重新打包成 `pc-custom.snap`，再使用 `--snap` 取代 Store 的 `pc` 套件。

Ubuntu Core 要求 model 的來源可被驗證，因此透過 Snapcraft 取得帳號 ID，並建立、註冊簽署公鑰。
實際簽署由本機的 `snap sign` 完成；model 不需要上傳，私鑰也留在主機。

### 查詢 developer ID

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

### 建立金鑰與自訂 model

以下在前面的 `~/ubuntu-core-vm` 工作目錄執行，將 `YOUR_DEVELOPER_ID` 換成 `whoami` 顯示的 `id`：

```bash
cd ~/ubuntu-core-vm
export DEVELOPER_ID="YOUR_DEVELOPER_ID"

# 建立簽署金鑰，再將公鑰註冊到自己的帳號；已有金鑰時可沿用
snapcraft create-key my-model-key
snapcraft register-key my-model-key
snapcraft keys

# 下載未簽署的 JSON 原稿，不修改 Canonical 已簽署的 .model
curl -fL -o reference-model.json \
  https://raw.githubusercontent.com/canonical/models/master/ubuntu-core-24-amd64.json

# 改成自己的品牌、model 名稱與開發等級，timestamp 設在金鑰註冊之後
jq --arg id "$DEVELOPER_ID" --arg timestamp "$(date -Iseconds --utc)" \
  '."authority-id" = $id | ."brand-id" = $id |
   .model = "my-core24-amd64" | .grade = "dangerous" |
   .timestamp = $timestamp' \
  reference-model.json > my-model.json

# 用自己的金鑰簽署，產生 ubuntu-image 使用的 model assertion
export GPG_TTY="$(tty)"
snap sign -k my-model-key my-model.json > my-model.model
```

`grade: dangerous` 允許未經 Store assertions 驗證的本機 Snap，但 **model 本身仍須簽署**。
這組簽署金鑰不是前面登入 VM 用的 SSH 金鑰；私鑰留在主機，註冊的是公鑰。

### 產生 pc-custom.snap

在 Ubuntu amd64 主機下載官方 Core 24 gadget，保留開機檔案，只修改分割區配置，不需要重新編譯 GRUB：

```bash
# 安裝解包工具與文字編輯器
sudo apt install squashfs-tools
cd ~/ubuntu-core-vm

# 下載官方 gadget，輸出為 pc-original.snap 與 pc-original.assert
snap download pc --channel=24/stable --basename=pc-original

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

### 放入本機 gadget 並建置

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

### 運行自訂映像

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

### 驗證結果

以下在 **自訂 VM 內**執行：

```bash
# 確認 pc 為本機套件，revision 通常顯示 x1 等值
snap list pc

# 確認 model 為 my-core24-amd64、brand-id 為自己的 ID、grade 為 dangerous
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

* [Storage layout](https://documentation.ubuntu.com/core/explanation/core-elements/storage-layout/)
* [Inside Ubuntu Core](https://documentation.ubuntu.com/core/explanation/core-elements/inside-ubuntu-core/)
* [How installation works](https://documentation.ubuntu.com/core/explanation/how-installation-works/)
* [Use ubuntu-image](https://documentation.ubuntu.com/core/how-to-guides/image-creation/use-ubuntu-image/)
* [Add custom snaps](https://documentation.ubuntu.com/core/how-to-guides/image-creation/add-custom-snaps/)
* [Access Ubuntu One](https://documentation.ubuntu.com/core/tutorials/build-your-first-image/access-ubuntu-one/)
* [Sign the model](https://documentation.ubuntu.com/core/tutorials/build-your-first-image/sign-the-model/)
* [Test on QEMU](https://documentation.ubuntu.com/core/how-to-guides/manage-ubuntu-core/test-on-qemu/)
