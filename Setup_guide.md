# SemuBot speech system setup guide
Tested on setup with Nvidia Jetson Orin NX and Intel Nuc 13

## Setup Jetson Orin NX (Or other Jetson device):

0. Set up [SDK manager](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html) on your host computer (native linux strongly recommended)
1. Flash your Jetson Orin device following [this tutorial](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html) 
1. Clone Semubot speech repository `git clone https://github.com/SemuBot/Unn-Thesis-2024-Semubot-SpeechSystem.git`
1. Install [ROS2 Humble](https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html) for Jetson
    1. Create workspace
    2. Copy packages custom_interfaces and semubot_llm from this repository to the workspace (src folder)
    3. Build packages `colcon build`
1. Install [Docker](https://docs.docker.com/desktop/install/linux-install/)
1. Install [Llama.cpp Docker container with Nvidia GPU support](https://github.com/dusty-nv/jetson-containers/tree/master/packages/llm/llama_cpp)
1. Install [Llammas model GGUF file](https://huggingface.co/AlbertUnn/LlammasGGUF) from Huggingface and place it in Llama.cpp `models` directory




## Setup Intel NUC

0. Follow NUC setup guide for [linux](https://ubuntu.com/download/intel-nuc-desktop) (Ubuntu 22.04)
1. Clone Semubot speech repository `git clone https://github.com/SemuBot/Unn-Thesis-2024-Semubot-SpeechSystem.git`
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
    1. Create workspace
    2. Copy packages custom_interfaces and semubot_speech_recognition from this repository to the workspace (src folder)
    3. Follow audio (microphone) [setup](https://github.com/SemuBot/respeaker_ros/tree/dbb571247928ef39566d2419516f1726b550b425)
    3. Build packages `colcon build`
1. Install [Docker](https://docs.docker.com/desktop/install/linux-install/)
1. Pull [Kiirkirjutaja](https://github.com/alumae/kiirkirjutaja) docker container `docker pull alumae/kiirkirjutaja:latest`
1. Follow TartuNLP [Text-To-Speech](https://koodivaramu.eesti.ee/tartunlp/text-to-speech) setup (including installing [conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html))


### Running the Semubot speech system
NB! Make sure to look at both of the nodes (llm_controller.py on Jetson and speech_controller.py on NUC) and change any filepaths and environment variables that are necessary.
1. Connect Jetson and NUC with a usb-c or ethernet cable (make sure the ROS enviroment variables allow nodes to subscribe and publish to topics between the computers)
1. Connect the microphone and speakers to NUC and start respeaker_ros respeaker_node with `ros2 run respeaker_ros respeaker_node` on NUC
1. Start llm_controller node in Jetson with `ros2 run semubot_llm llm_controller`
1. Start Kiirkirjutaja docker container in NUC with `docker run --shm-size 2GB --name kiirkirjutaja --rm -d -t alumae/kiirkirjutaja:latest`
1. Start speech_controller node in NUC with `ros2 run semubot_speech_recognition client` and wait about 10 seconds for recognition to start
1. Speak to the robot

Robot is searching for a 4 second pause in recognition after which it starts generating a response on the Jetson using a 7B finetuned estonian conversational Llama2 model Lammas. Then the NUC synthesizes said response with a Text-To-Speech model (Same as Neurokõne).


TODO: Launch files, Usage documentation