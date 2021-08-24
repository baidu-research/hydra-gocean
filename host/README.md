# Host
Applications on host to communicate with Jetson


## AIoT Comm
AIoT communication is designed and implemented basing on Kafka
- Bidirectional messaging between cloud and edge
- Security messaging

### Setup
FFmpeg with GPU acceleration requires a system with Linux OS:
- Prerequisites
    ```
    $ sudo apt install default-jre
    Environment setting to add Jetson/Host
    $ sudo vi /etc/hosts
    Uncompress Kafka
    $ cd xos/host/aiot-comm
    $ tar xvfz kafka_2.13-2.8.0.tgz
    $ cd kafka_2.13-2.8.0
    ```
- Execution
    ```
    Start Kafka environment
    Launch Kafka services in the correct order
    $ bin/zookeeper-server-start.sh config/zookeeper.properties
    Open another terminal session
    $ bin/kafka-server-start.sh config/server.properties
    Create "jetson-host" topic to store events
    $ bin/kafka-topics.sh --create --topic jetson-host --bootstrap-server localhost:9092
    $ bin/kafka-topics.sh --describe --topic jetson-host --bootstrap-server localhost:9092
    Write events to topic
    $ bin/kafka-console-producer.sh --topic jetson-host --bootstrap-server localhost:9092
    Read events from the same topic
    Run the consumer client to read the events
    $ bin/kafka-console-consumer.sh --topic jetson-host --from-beginning --bootstrap-server localhost:9092
    ```
- Process
    ```
    Kafka streams: https://kafka.apache.org/documentation/streams
    More develop: https://kafka.apache.org/25/documentation/streams/tutorial
    ```
- Cleanup
    ```
    Stop Kafak
    $ Ctrl-C to stop producer & consumer clients, as well as broker & zookeeper
    $ rm -rf /tmp/kafka-logs /tmp/zookeeper
    ```


## DS
Deploy deepstream on the host to handle the RTSP streaming from Jetson for real-time video analytisc
- Host is also Jetson with ARM SOC
- Host is x86 system with GPU attached


## VLC
Behaving as the player or viewer for RTSP from Jetson


## FFmpeg
FFmpeg is the most popular multimedia transcoding software used extensively
FFplay is a portable media player using the FFmpeg libraries and the SDL library
NVENC & NVDEC can be effectively used with FFmpeg//FFplay to significantly speed up video transcoding for Vidpress

### Setup
FFmpeg with GPU acceleration requires a system with Linux or Windows OS:    
- System                  X86-64 with Ubuntu Ubuntu 18.04.4 LTS 
- GPU                     GP102 [GeForce GTX 1080 Ti]   
- CUDA                    11.3 (11.0 or higher) 
- Driver                  465.19.01 (455.27 or higher)  
- NVIDIA Video SDK Zone   https://developer.nvidia.com/nvidia-video-codec-sdk   
- Video Codec SDK GitLab  https://gitlab.com/nvidia/video/video-codec-sdk   

### Prerequisites
FFmpeg requires separate git repository nvcodec-headers for NV-accelerated ffmpeg build
To compile FFmpeg, the CUDA toolkit must be installed on the system
Before using FFmpeg, it is recommended to refer to the FFmpeg documentation, note the version of the Video Codec SDK it uses, and ensure that the minimum driver required for that version of the Video Codec SDK is installed.

### Installation
FFmpeg with GPU acceleration is supported on all Linux platforms.
- CUDA Update
    ```
    In case need to remove old version
    $ sudo apt clean                      
    $ sudo apt update
    $ sudo apt purge nvidia-* 
    $ sudo apt autoremove
    Install cuda-11.3 local deb on Ubuntu 18
    $ wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
    $ sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
    $ wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda-repo-ubuntu1804-11-3-local_11.3.0-465.19.01-1_amd64.deb
    $ sudo dpkg -i cuda-repo-ubuntu1804-11-3-local_11.3.0-465.19.01-1_amd64.deb
    $ sudo apt-key add /var/cuda-repo-ubuntu1804-11-3-local/7fa2af80.pub
    $ sudo apt-get update
    $ sudo apt-get -y install cuda
    Install cuda local run on Ubuntu 18 in text mode
    $ wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda_11.3.0_465.19.01_linux.run
    $ sudo sh cuda_11.3.0_465.19.01_linux.run
    ```
- Install Packages
    ```
    Basic
    $ sudo apt-get install build-essential yasm cmake libtool libc6 unzip wget libnuma1 libc6-dev libnuma-dev \
    libopus-dev libvpx-dev libx264-dev libx265-dev libfribidi-dev libmp3lame-dev libfontconfig-dev
    Install OpenCL sw packages
    $ sudo apt-get install nvidia-opencl-dev ocl-icd-opencl-dev opencl-headers clinfo
    ```
- Clone ffnvcodec
    ```
    $ cd codec-headers                    
    Use above tested code, OR
    $ git clone https://git.videolan.org/git/ffmpeg/nv-codec-headers.git && cd nv-codec-headers
    ```
- Install ffnvcodec
    ```
    $ make && sudo make install && cd ..
    ```
- Extract FFmpeg
    ```
    $ unzip ../video-codec-sdk/Video_Codec_SDK_11.0.10/Samples/External/FFmpeg/src/ffmpeg-4.3.zip
    $ cd ffmpeg-4.3                       
    Use above tested code, OR
    $ git clone https://git.ffmpeg.org/ffmpeg.git ffmpeg && cd ffmpeg && git checkout release/4.3
    ```
- Configure
    ```
    Depend on the GPU architecture to set "-gencode arch=compute_xx,code=sm_xx"
    For example, GeForce GTX 1080 Ti is -gencode arch=compute_61,code=sm_61
    $ cp ../reference/configure-ffmpeg-4.3 configure
    Configure with OpenCL
    $ ./configure --toolchain=hardened --enable-nonfree --enable-cuda-nvcc --enable-libnpp --enable-opencl \
    --disable-stripping --enable-avresample --enable-libfontconfig --enable-libfreetype --enable-libfribidi --enable-libmp3lame \
    --enable-libopus --enable-gpl --enable-libvpx --enable-libx264 --enable-libx265 \
    --extra-cflags=-I/usr/local/cuda/include --extra-ldflags=-L/usr/local/cuda/lib64
    ```
- Compile
    ```
    $ make -j 8
    ```
- Install Libraries
    ```
    $ sudo make install
    ```
- Execution
    ```
    $ ffplay rtsp://192.168.86.1699:8555/ds-test
    ```
 
