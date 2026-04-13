---
title: NVIDIA GPU 安裝
description: 介紹 NVIDIA GPU 架構以及如何安裝
keywords:
  - AI
---

NVIDIA GPU 有很多概念，例如 Compute Capability、CUDA Toolkit、cuDNN 等等，為了方便理解，這邊重新整理一下。

## Compute Capability

簡稱為 CC，代表的是這個 GPU 所支援的硬體指令集，所以無法被改變。
如果要查自己手上的 GPU 到底是對應到哪個 CC，可以參考 NVIDIA 的[官網](https://developer.nvidia.com/cuda/gpus)。

```bash
# 查詢自己 GPU 型號的方式
lshw -c display
```

## NVIDIA GPU 驅動程式

如果要能讓 GPU 在作業系統運作，就需要安裝驅動程式，通常版本號格式是 570 / 580 等等。
這個驅動程式版本一般會受限於 Linux 的 kernel 版本，因為其本身會用到 kernel 的 API，所以有版本相依。
如果是太舊的 Ubuntu 無法更新 kernel，那就無法使用新版的 GPU driver 了。

* 新增 NVIDIA repository

```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
```

* 安裝驅動程式

```bash
# 查看目前支援的驅動版本，通常會看到 recommended 的推薦選項
ubuntu-drivers devices
# 自動安裝 recommended 的驅動程式
sudo ubuntu-drivers autoinstall
# 鎖定某個版本安裝
sudo apt-get install nvidia-driver-590
```

* 重開機，這時候可以查看狀態

```bash
# 看 GPU 的狀態，含驅動版號、CUDA版號等等
nvidia-smi
# 看 GPU 設定
sudo nvidia-settings
```

## CUDA Toolkit

如果要讓程式有效運用 GPU 的平行效能，就需要借助 NVIDIA 的 API 界面，也稱為 CUDA。
而把 CUDA API 轉成執行檔就需要依賴 NVIDIA 的 CUDA 編譯器，這個編譯器就是在 CUDA Toolkit 內。
要注意的是由於 CUDA Toolkit 會跟 GPU driver 互動，所以兩者版本也必須要能夠相容才可以。

* 新增 CUDA repository

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
```

* 安裝 CUDA (指定要安裝的版本)

```bash
sudo apt install cuda-toolkit-13-2
```

* 可以將下列環境變數加到 `~/.profile`，這樣每次開 terminal 就都可以直接使用 CUDA

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

* 確認當前的 CUDA 版本

```bash
# 查看 CUDA compiler 版本
nvcc --version
# 也會顯示目前的 CUDA 版本
nvidia-smi
```

## cuDNN (CUDA Deep Neural Network library)

如果是專門用在深度神經網路(DNN, Deep Neural Networks)，NVIDIA 也有提供專門的函式庫可以用來加速，稱為 cuDNN。
要注意的是 cuDNN 版本跟 CUDA 相依，所以要確認版本是否相符。

```bash
# 如果只是要執行
sudo apt install libcudnn9-cuda-13
# 如果要開發用
sudo apt install libcudnn9-dev-cuda-13
```

測試方式(記住要先設定好 CUDA 環境變數)

```bash
# 安裝範例
sudo apt install libcudnn9-samples
# 複製範例
cp -r /usr/src/cudnn_samples_v9 ~/
cd ~/cudnn_samples_v9/conv_sample
# 編譯並且執行
make
./conv_sample
```

## TensorRT

TensorRT 是 NVIDIA 專門用來加速推論的框架，當把 `.onnx` 檔案優化成 `.plan` 格式後，就可以非常有效加速推論結果。
要注意的是，這個優化的過程是綁定該版本 GPU 的，所以輸出的 `.plan` 無法跑在其他平台上面。

* 先從[官網](https://developer.nvidia.com/tensorrt)下載
* 跟著[教學](https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/installing.html)來安裝

## 在容器中使用 GPU

如果要在虛擬化容器使用 GPU，那就還需要安裝 NVIDIA Container Toolkit。

* 安裝 APT repository

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
```

* 安裝並重啟 docker 服務

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## Troubleshooting

* 如果發生環境上面的問題，想要回復乾淨環境

```bash
sudo apt purge 'nvidia-.*'
sudo apt purge 'cuda-.*'
sudo apt purge "libnv*"
```

## 其他

不一定要依賴 NVIDIA 平台的框架

### PyTorch

可以用在訓練以及推論的深度學習框架，不過大多數都是用在訓練和開發，推論會用 ONNX Runtime 或 TensorRT。

* 可以從[官網](https://pytorch.org/get-started/locally/)下載

### ONNX Runtime

專門用來跑模型的推論引擎

* 可以從 [GitHub](https://github.com/microsoft/onnxruntime/releases) 下載
