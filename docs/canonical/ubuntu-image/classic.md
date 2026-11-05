---
title: 使用 ubuntu-image 建置 Classic 映像
description: 使用 ubuntu-image classic 依 image definition 建置 Ubuntu 26.04 Server 與 Desktop preinstalled 映像，並在 QEMU 驗證
keywords:
  - ubuntu-image
  - classic
  - image definition
  - Ubuntu Server
  - Ubuntu Desktop
  - seed
  - QEMU
---

本文使用 `ubuntu-image classic` 建置 Ubuntu Server 與 Desktop 的 **preinstalled 磁碟映像**（`.img`）。
Image definition、gadget 與 preinstalled 映像的概念請先參考 [ubuntu-image 概念](concepts.md)，安裝 `ubuntu-image` 的方式也在該頁的「前置需求與安裝」。

本文以 **Ubuntu 26.04（resolute）、x86-64（`amd64`）** 為例。
Preinstalled 映像是已安裝好的完整磁碟，寫入裝置或交給 VM 就能開機，和官網下載的安裝用 ISO 不同。
官方 ISO 由 `livecd-rootfs` 與 `ubuntu-cdimage` 在 Launchpad 上建置，不是用 `ubuntu-image` 產生。

## Server 與 Desktop 的差異

兩者的建置流程一樣，差別只在 image definition 內的 **seed 名稱**：

| 項目 | Server | Desktop |
| --- | --- | --- |
| 子指令 | `ubuntu-image classic` | `ubuntu-image classic` |
| `rootfs.seed.names` | `server`、`minimal`、`standard`、`cloud-image` | `desktop-minimal`、`minimal`、`standard` |
| 預載 Snap | `snapd` | `snapd`，以及 seed 帶入的 `firefox` 等桌面 Snap |
| 映像大小 | 約 4.0 GiB | 約 8.7 GiB |
| 建置時間 | 約 17 分鐘 | 約 22 分鐘 |

Seed 是一份套件清單，`ubuntu-image` 會用 `germinate` 展開相依關係後安裝。
可以在 [ubuntu-seeds](https://git.launchpad.net/~ubuntu-core-dev/ubuntu-seeds/+git/ubuntu/tree/server?h=resolute) 網頁查看各 seed 的內容。
大小與時間是兩個映像同時在一台 amd64 主機上建置所測得，只供參考。

## 主機前置設定

驗證用的 QEMU 與 UEFI firmware：

```bash
sudo apt update
sudo apt install qemu-system-x86 ovmf
```

`ubuntu-image classic` 需要 root 權限，因為它會建立 chroot、掛載 loop device 並寫入分割區。
建置過程需要連到 Ubuntu archive、Launchpad（下載 seed）、GitHub（下載 gadget）與 Snap Store。

## 建置 Server 映像

### Image definition

先建立 `regen-initrd.sh`，原因見下方 `manual` 的說明：

```bash
cat > regen-initrd.sh <<'EOF'
#!/bin/sh
# 建置主機的硬體不應決定 initrd 內容，改用 generic 模式重新產生
echo 'hostonly="no"' > /etc/dracut.conf.d/90-generic.conf
dracut --force --regenerate-all
EOF
chmod +x regen-initrd.sh
```

再建立 `server.yaml`：

```yaml
# 映像的名稱與描述
name: ubuntu-server-amd64
display-name: Ubuntu Server (resolute) for amd64
revision: 1
architecture: amd64
# Ubuntu 版本代號，26.04 是 resolute
series: resolute
# preinstalled：產生已安裝好、可直接開機的磁碟映像
class: preinstalled
kernel: linux-image-generic

# gadget：提供分割區配置與 GRUB 開機資產
# 要用 classic 分支；classic-26.04 分支沒有 Makefile，ubuntu-image 會建置失敗
gadget:
  url: https://github.com/canonical/pc-gadget.git
  type: git
  branch: classic

# rootfs：映像的檔案系統內容，從 Ubuntu archive 安裝
rootfs:
  components:
    - main
    - restricted
    - universe
  archive: ubuntu
  mirror: http://archive.ubuntu.com/ubuntu/
  pocket: updates
  seed:
    urls:
      - git://git.launchpad.net/~ubuntu-core-dev/ubuntu-seeds/+git/
    # Server 與 Desktop 的差別只在這裡：seed 是套件清單，決定映像裝哪些套件
    names:
      - server
      - minimal
      - standard
      - cloud-image
    # seed 的 git 分支，對應 Ubuntu 版本
    branch: resolute
    vcs: true
  sources-list-deb822: true

# customization：映像產生後的額外設定
customization:
  # seed 不含 bootloader 套件，缺少時 setup_bootloader 會失敗：
  # grub-install: error: /usr/lib/grub/x86_64-efi/modinfo.sh doesn't exist
  extra-packages:
    - name: grub-efi-amd64
    - name: grub-efi-amd64-signed
    - name: shim-signed
  # 26.04 的 initrd 由 dracut 依「建置主機」的硬體產生。在 NVMe 主機上建置時只有 nvme 驅動，
  # 到 QEMU 的 SATA 磁碟會卡在 dracut-initqueue: timeout。
  # 這裡複製並執行 regen-initrd.sh，改用 generic 模式重新產生 initrd
  manual:
    copy-file:
      - source: regen-initrd.sh
        destination: /usr/local/sbin/regen-initrd.sh
    execute:
      - path: /usr/local/sbin/regen-initrd.sh
  extra-snaps:
    - name: snapd
  # cloud-init 的 NoCloud datasource 需要 meta-data 才會讀 user-data，
  # 只寫 user-data 時帳號不會建立，登入會顯示 Login incorrect
  cloud-init:
    meta-data: |
      instance-id: ubuntu-image-test
    # ubuntu:ubuntu 只是測試用，正式使用請改用 SSH 公鑰
    user-data: |
      #cloud-config
      chpasswd:
        expire: false
        users:
          - {name: ubuntu, password: ubuntu, type: text}
      ssh_pwauth: true

# artifacts：輸出的檔案
artifacts:
  img:
    - name: server.img
  manifest:
    # 列出 rootfs 內所有套件與版本
    name: server.manifest
```

### 建置

```bash
sudo ubuntu-image classic server.yaml --output-dir out-server --workdir wd-server
```

主要的建置階段如下：

1. `build_gadget_tree`：下載並建置 gadget。
2. `germinate`：依 seed 展開套件清單。
3. `create_chroot`、`install_packages`：用 debootstrap 建立 rootfs 並安裝套件。
4. `prepare_image`、`preseed_image`：預載 Snap。
5. `make_disk`、`setup_bootloader`：建立分割區並安裝 GRUB。
6. `generate_package_manifest`：輸出套件清單。

成功時最後會顯示 `Build successful`。兩個目錄的用途如下：

- `out-server/`（`--output-dir`）：最終成品，有 `server.img` 與 `server.manifest`。`server.manifest` 列出 rootfs 內的所有套件與版本。
- `wd-server/`（`--workdir`）：建置用的暫存目錄，放 chroot、gadget 原始碼等中間產物，建置完成後不需要，可以刪除。

兩個目錄都屬於 root，刪除時需要 `sudo`：

```bash
sudo rm -rf wd-server
```

## 建置 Desktop 映像

Desktop 的 YAML 與 Server 幾乎相同，只修改名稱、seed 與輸出檔名，並使用同一份 `regen-initrd.sh`。
本例用 `desktop-minimal`，比完整的 `desktop` seed 小。

建立 `desktop.yaml`：

```yaml
# 映像的名稱與描述
name: ubuntu-desktop-amd64
display-name: Ubuntu Desktop Minimal (resolute) for amd64
revision: 1
architecture: amd64
# Ubuntu 版本代號，26.04 是 resolute
series: resolute
# preinstalled：產生已安裝好、可直接開機的磁碟映像
class: preinstalled
kernel: linux-image-generic

# gadget：提供分割區配置與 GRUB 開機資產
# 要用 classic 分支；classic-26.04 分支沒有 Makefile，ubuntu-image 會建置失敗
gadget:
  url: https://github.com/canonical/pc-gadget.git
  type: git
  branch: classic

# rootfs：映像的檔案系統內容，從 Ubuntu archive 安裝
rootfs:
  components:
    - main
    - restricted
    - universe
  archive: ubuntu
  mirror: http://archive.ubuntu.com/ubuntu/
  pocket: updates
  seed:
    urls:
      - git://git.launchpad.net/~ubuntu-core-dev/ubuntu-seeds/+git/
    # 與 Server 的差別只在這裡：改用 desktop-minimal seed（比完整的 desktop 小）
    names:
      - desktop-minimal
      - minimal
      - standard
    # seed 的 git 分支，對應 Ubuntu 版本
    branch: resolute
    vcs: true
  sources-list-deb822: true

# customization：映像產生後的額外設定
customization:
  # seed 不含 bootloader 套件，缺少時 setup_bootloader 會失敗：
  # grub-install: error: /usr/lib/grub/x86_64-efi/modinfo.sh doesn't exist
  extra-packages:
    - name: grub-efi-amd64
    - name: grub-efi-amd64-signed
    - name: shim-signed
  # 26.04 的 initrd 由 dracut 依「建置主機」的硬體產生。在 NVMe 主機上建置時只有 nvme 驅動，
  # 到 QEMU 的 SATA 磁碟會卡在 dracut-initqueue: timeout。
  # 這裡複製並執行 regen-initrd.sh，改用 generic 模式重新產生 initrd
  manual:
    copy-file:
      - source: regen-initrd.sh
        destination: /usr/local/sbin/regen-initrd.sh
    execute:
      - path: /usr/local/sbin/regen-initrd.sh
  extra-snaps:
    - name: snapd
  # cloud-init 的 NoCloud datasource 需要 meta-data 才會讀 user-data，
  # 只寫 user-data 時帳號不會建立，登入會顯示 Login incorrect
  cloud-init:
    meta-data: |
      instance-id: ubuntu-image-test
    # ubuntu:ubuntu 只是測試用，正式使用請改用 SSH 公鑰
    user-data: |
      #cloud-config
      chpasswd:
        expire: false
        users:
          - {name: ubuntu, password: ubuntu, type: text}
      ssh_pwauth: true

# artifacts：輸出的檔案
artifacts:
  img:
    - name: desktop.img
  manifest:
    # 列出 rootfs 內所有套件與版本
    name: desktop.manifest
```

```bash
sudo ubuntu-image classic desktop.yaml --output-dir out-desktop --workdir wd-desktop
```

與 Server 比較，`preseed_image` 階段會向 Snap Store 查詢 seed 內的 Snap（例如 `firefox`）。
若出現 `Error getting info for snap firefox: "too many requests"`，這是 Snap Store 的速率限制，稍後重新執行即可。

## 用 QEMU 驗證

以 sudo 建置的映像屬於 root，QEMU 需要寫入權限，所以先改為目前使用者。
每個 VM 要有自己的 UEFI variables 副本。

### Server

```bash
# 改為目前使用者，並複製這台 VM 專用的 UEFI variables
sudo chown "$USER": out-server/server.img
cp /usr/share/OVMF/OVMF_VARS_4M.fd OVMF_VARS_server.fd

# 4 GiB RAM、4 個 vCPU；主機 2222 連接埠轉到 VM 的 SSH
qemu-system-x86_64 \
  -machine q35 -accel kvm -cpu host -m 4G -smp 4 \
  -drive if=pflash,format=raw,readonly=on,file=/usr/share/OVMF/OVMF_CODE_4M.fd \
  -drive if=pflash,format=raw,file=OVMF_VARS_server.fd \
  -drive file=out-server/server.img,format=raw \
  -nic user,model=virtio-net-pci,hostfwd=tcp::2222-:22
```

開機後使用 `ubuntu` / `ubuntu` 登入，或從主機執行 `ssh -p 2222 ubuntu@localhost`。

### Desktop

```bash
# 改為目前使用者，並複製這台 VM 專用的 UEFI variables
sudo chown "$USER": out-desktop/desktop.img
cp /usr/share/OVMF/OVMF_VARS_4M.fd OVMF_VARS_desktop.fd

# 6 GiB RAM、4 個 vCPU；-vga virtio 提供圖形顯示
qemu-system-x86_64 \
  -machine q35 -accel kvm -cpu host -m 6G -smp 4 -vga virtio \
  -drive if=pflash,format=raw,readonly=on,file=/usr/share/OVMF/OVMF_CODE_4M.fd \
  -drive if=pflash,format=raw,file=OVMF_VARS_desktop.fd \
  -drive file=out-desktop/desktop.img,format=raw \
  -nic user,model=virtio-net-pci
```

開機後會進入 GDM 登入畫面，使用 `ubuntu` / `ubuntu` 登入。
`desktop-minimal` seed 沒有 SSH server，所以這裡沒有轉送 SSH 連接埠；需要 SSH 時，請在 `extra-packages` 加入 `openssh-server`。

## 參考資料

- [ubuntu-image classic 使用範例](https://canonical-subiquity.readthedocs-hosted.com/en/latest/howto/ubuntu-image-classic-examples.html)
- [Image definition 欄位說明](https://github.com/canonical/ubuntu-image/blob/main/internal/imagedefinition/README.rst)
- [ubuntu-seeds](https://git.launchpad.net/~ubuntu-core-dev/ubuntu-seeds/+git/ubuntu)
