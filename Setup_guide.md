# SemuBot speech system setup guide
Tested on setup with Nvidia Jetson Orin NX and Intel Nuc 13

## Setup Jetson Orin NX (Or other Jetson device):

0. Set up [SDK manager](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html) on your host computer (native linux strongly recommended)
1. Flash your Jetson Orin device following [this tutorial](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html) 
1. Clone Semubot speech repository `git clone https://github.com/SemuBot/Unn-Thesis-2024-Semubot-SpeechSystem.git`
1. Install ROS2 Humble
    1. Create workspace
    2. Copy directories
    3. Build packages `colcon build`
1. Install Docker
1. Install Llama.cpp Docker container with Nvidia GPU support 
1. Install [Llammas model GGUF file](https://huggingface.co/AlbertUnn/LlammasGGUF) from Huggingface and place it in Llama.cpp `models` directory




## Setup Intel NUC

0. Follow NUC setup guide for [linux](https://ubuntu.com/download/intel-nuc-desktop) (Ubuntu 22.04)
1. Clone Semubot speech repository `git clone https://github.com/SemuBot/Unn-Thesis-2024-Semubot-SpeechSystem.git`
1. Install ROS2 Humble
    1. Create workspace
    2. Copy directories
    3. Build packages `colcon build`
1. Install Docker
1. Pull [Kiirkirjutaja](https://github.com/alumae/kiirkirjutaja) docker container `docker pull alumae/kiirkirjutaja:latest`
1. Follow TartuNLP [Text-To-Speech](https://koodivaramu.eesti.ee/tartunlp/text-to-speech) setup (including installing [conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html))


TODO: Launch files, Node setups, Usage documentation