---
title: Embodied AI 介紹
description: Embodied AI 介紹
keywords:
  - AI
---

隨著人工智慧的發展，有身體的 AI，也就是 Embodied AI 也開始被大量關注。
相對於一般雲端的聊天機器人，Embodied AI 強調物理載體，以及怎麼感知物理世界的資料，並且與其互動。
比起傳統機器人或自駕車的固定演算法，Embodied AI 因為能從物理世界中學習，更加能夠應對日益複雜的情況。

## 端到端 (End-to-End)

傳統的機器人演算法需要人類去撰寫複雜的邏輯和狀態機，例如 ROS 的 navigation 或是 moveit 套件，分別對應 AMR 或是機器手臂的應用。
這些套件的作法通常是把邏輯拆分成多個模組，例如感知模組、定位模組、控制模組等等，每個模組透過定義好的接口（像是 ROS topic）來傳遞資料。
然而面對複雜的真實世界，人類的設計往往無法考量周全，每遇到一個新的情境就要加入新的邏輯，這也是機器人領域過去發展的痛點。

後來有人想到既然人為設計這麼困難，那是否我們可以直接從原始資料產生出對應的行為呢？
那個流派就被稱作端到端。

一般來說端到端的輸入是各種感測器的資料，像是攝影機、光達等等，經過一個我們所訓練的 model，會直接產生出控制指令，像是馬達控制。
這樣的方式很有用，不用想辦法去處理各種 corner case，更容易應對真實世界的挑戰。

但是端到端也不是沒有缺點。第一，他無法進行語言的邏輯推理，例如我請機器人拿玩具，他就必須要了解玩具的定義是什麼。第二，我們無法了解為何 model 會做這樣的決定，例如自駕車決定在某個路口煞車時，我們需要知道他的判斷邏輯是什麼。這兩個情境說明了語言在其中的重要性。

因此 VLA (Vision-Language-Action) 就出現了。Vision 代表輸入接收影像資料，透過語言邏輯的判讀，最終產生出了行動。由於可以使用自然語言，就可以使用網路上的文本資料協助訓練，並且也提供更好的人機界面了。

VLA 目前最知名的專案有 Google 的 [RT-2](https://deepmind.google/blog/rt-2-new-model-translates-vision-and-language-into-action/)，以及 [OpenVLA](https://github.com/openvla/openvla)。

## ACT / VLA / WAM 比較

**ACT**: Action Chunking with Transformers

這種方法類似於 imitation learning，輸入是畫面和機器人狀態，會輸出一連串的機器人動作，但是不會有任何的語意理解。

**VLA**: Vision-Language-Action model

在 ACT 的基礎上，多增加了語言指令，通常是借用 VLM 的語意能力。
輸入除了畫面和機器人狀態外，多增加了語言指令，輸出會根據指令而有不一樣的行動。

**WAM**: World Action Model

最新的研究進展，會加上對物理世界的理解，思考自己的動作會對世界造成什麼樣的變化。
這樣對未來的想像能力，可以有更好的行動策略，例如往前推杯子，杯子會移動多遠，會不會掉下桌子等等。
然而，這需要很高的運算量，也會提高 latency。

## Sim-to-Real

機器人領域之所以不像是大語言模型(LLM)發展那麼迅速，最主要的原因在於「資料瓶頸」(Data Bottleneck)，特別是高品質的資料。
機器人所需要的運動資料，相較於網路上隨手可得的文字資料少上不少，這類資料不只是包含視覺影像，更有複雜的物理特性（如扭矩、摩擦力、感測器回饋等等），且還跟硬體規格高度綁定。
如果要靠自己產生訓練資料，那又會遇到「物理時鐘」的極限：一個十秒的動作，在現實中就必須實打實花費十秒，還伴隨著硬體的磨損和電力損耗。

這時有些人就提出了利用模擬世界來產生出資料，幫助我們在模擬器中並行化訓練機器人模型，大幅提昇學習效率，更是能符合特定硬體設計，達到數位孿生（Digital Twin）的成效。

然而，Sim-to-Real 最大的挑戰在於與現實世界的鴻溝。模擬世界難以完美還原物理世界的一切細節，還需要透過領域隨機化（Domain Randomization）———對各個參數引入隨機性，如外觀、重量等等———來增強模型在不確定環境的泛化性。

## 相關連結

* 學習資源：
    * [具身智能技术指南](https://github.com/TianxingChen/Embodied-AI-Guide)：簡體中文的各種學習資源
    * [现代强化学习实战课程](https://github.com/walkinglabs/hands-on-modern-rl)：簡體中文針對強化學習的課程
    * [Awesome VLA & WAM](https://github.com/DravenALG/awesome-vla-wam)：列出所有 VLA 和 WAM 相關研究和資源
* 模型與 dataset
    * [OpenVLA](https://openvla.github.io/)：經典 open-source VLA
    * [SmolVLA](https://huggingface.co/docs/lerobot/smolvla)：Hugging Face 所提出，輕量級 VLA，和 LeRobot 生態系整合良好
    * [Octo](https://octo-models.github.io/)：目標是通用機器人策略，可以支援多種機器人平台和 sensor 配置
    * [openpi](https://github.com/Physical-Intelligence/openpi)：Physical Intelligence 提出，特色是 flow-matching / action chunking 路線，目標是讓 robot 做更連續、靈活
    * [NVIDIA Isaac GR00T N1 / N1.7](https://github.com/NVIDIA/Isaac-GR00T)：面向 humanoid 的 robot foundation model，和 NVIDIA 生態系緊密綁定
    * [MolmoAct 2](https://github.com/allenai/molmoact2)：Ai2 / Allen Institute for AI 所提出，特色是偏 Open Science：不只開模型，也強調 datasets、training code、robot policies、evaluation。
    * [Xiaomi-Robotics-0](https://github.com/XiaomiRobotics/Xiaomi-Robotics-0)：小米的 open VLA model
    * [Open X-Embodiment](https://robotics-transformer-x.github.io/)：Goodle DeepMind 和全球學術單位合作推出的開源巨型數據集
* 機器人重要研討會：
    * [CoRL (Conference on Robot Learning)](https://www.corl.org/)：Embodied AI 最頂尖的研討會
    * [ICRA (IEEE International Conference on Robotics and Automation)](https://ieee-icra.org)：機器人領域最早也最權威的研討會，討論各種理論與演算法的創新
    * IROS (IEEE/RSJ International Conference on Intelligent Robots and Systems)：由 IEEE RAS 與 日本機器人學會 (RSJ) 共同主辦，更強調實際的應用。
* 自駕車重要研討會
    * [IEEE IV (Intelligent Vehicles Symposium)](http://ieee-iv.org/)：專注於自動駕駛系統的技術實現與測試。
    * CVPR (Conference on Computer Vision and Pattern Recognition)：電腦視覺第一大會，強調如何落地應用
    * ICCV (International Conference on Computer Vision)：兩年一次(基數年)，偏向學術底層理論
    * ECCV (European Conference on Computer Vision)：兩年一次(偶數年)，有強烈歐洲色彩
