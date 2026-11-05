---
title: 使用 ubuntu-image 建置 Ubuntu Core 映像
description: 使用 ubuntu-image 依 model assertion 建置 Ubuntu Core x86-64 映像，並在 QEMU 驗證
keywords:
  - ubuntu-image
  - Ubuntu Core
  - model assertion
  - QEMU
---

本文使用 `ubuntu-image snap` 建置 Ubuntu Core 映像。
Core 的 Snap 組成、儲存配置與開機流程請先參考 [Ubuntu Core 入門](../ubuntu-core.md)，ubuntu-image 的整體概念請參考 [ubuntu-image 概念](concepts.md)。

本文以 **Ubuntu Core 26、x86-64（`amd64`）** 為例，主機使用 Ubuntu 26.04 amd64，VM 使用 QEMU/KVM。

## 主機前置設定

請先依 [ubuntu-image 概念](concepts.md) 的「前置需求與安裝」安裝 `ubuntu-image`。
以下指令在 **Ubuntu 主機** 執行。本例分配 2 GiB RAM 與 8 GiB 虛擬磁碟，並需要網際網路連線。

```bash
# 更新套件索引，安裝 QEMU、UEFI firmware、KVM 檢查工具與 curl
sudo apt update
sudo apt install qemu-system-x86 ovmf cpu-checker curl

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
curl -fL -o ubuntu-core-26-amd64.model \
  https://raw.githubusercontent.com/canonical/models/master/ubuntu-core-26-amd64.model
mkdir -p build

# 依 model 組合 8 GiB 映像到 build/，並加入首次設定用的 console-conf
ubuntu-image snap \
  --image-size 8G \
  --output-dir build \
  --snap console-conf \
  ubuntu-core-26-amd64.model

# 複製成 VM 工作磁碟；--sparse=always 減少連續零值區域的實際占用
cp --sparse=always build/pc.img ubuntu-core.img
```

輸出為 `build/pc.img`，這裡複製成 `ubuntu-core.img` 供 VM 使用。
Core 26 的參考 model 將 `console-conf` 標為 optional，因此需要明確加入。

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

若要調整分割區大小或開機資產，需要自己簽署 model 並替換 `pc` gadget Snap，請參考 [自訂 gadget 的 Core 映像](core-custom-gadget.md)。

## 參考資料

* [Use ubuntu-image](https://documentation.ubuntu.com/core/how-to-guides/image-creation/use-ubuntu-image/)
* [Test on QEMU](https://documentation.ubuntu.com/core/how-to-guides/manage-ubuntu-core/test-on-qemu/)
