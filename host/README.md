# Host
Applications on host to communicate with Jetson
```
1. AIoT Comm
2. DeepStream
3. FFmpeg
4. VLC
```

## 1. AIoT Comm
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


## 2. DeepStream
Deploy deepstream on the host to handle the RTSP streaming from Jetson for real-time video analytisc
- Target is Jetson with ARM SOC and CUDA
- Host is x86 system with GPU attached


## 3. FFmpeg
FFmpeg is the most popular multimedia transcoding software used extensively
FFplay is a portable media player using the FFmpeg libraries and the SDL library
NVENC & NVDEC can be effectively used with FFmpeg//FFplay to significantly speed up video transcoding

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

### Testing
Once the FFmpeg binary with GPU acceleration support is compiled, hardware-accelerated video transcode should be tested to ensure everything works well. To automatically detect GPU accelerated video codec and keep video frames in GPU memory for transcoding, the ffmpeg cli option "-hwaccel cuda -hwaccel_output_format cude" is used in further code snippets.
- Transcode without Scaling
    ```
    // Read input.mp4 and transcodes to output.mp4 with H.264 video at the same resolution with the same audio codec
    ffmpeg -y -vsync 0 -hwaccel cuda -hwaccel_output_format cuda -i input.mp4 \
    -c:a copy -c:v h264_nvenc -b:v 5M output.mp4
    ```
- Transcode with CUDA Scaling
    ```
    // Read input.mp4 and transcodes to output.mp4 with H.264 video at 720p resolution and with the same audio codec
    The scale_cuda and cuvid decoder built in resizer is used 
    ffmpeg -y -vsync 0 -hwaccel cuda -hwaccel_output_format cuda -i input.mp4 \
    -vf scale_cuda=1280:720 -c:a copy -c:v h264_nvenc -b:v 5M output.mp4
    ```
- Transcode with NPP Scaling
    ```
    // Read input.mp4 and transcodes to output.mp4 with H.264 video at 720p resolution and with the same audio codec
    The scale_npp and cuvid decoder built in resizer is used 
    ffmpeg -y -vsync 0 -hwaccel cuda -hwaccel_output_format cuda -i input.mp4 \
    -vf scale_npp=1280:720 -c:a copy -c:v h264_nvenc -b:v 5M output.mp4
    ```
- Transcode with Scaling for Multi-output at Diff-resolution
    ```
    // Transcodes input.mp4 to different H.264 videos at various output resolutions and bit rates
    // scale_npp & cuvid decoder built in resizer are used 
    ffmpeg -y -vsync 0 -hwaccel cuda -hwaccel_output_format cuda -i input.mp4 \
    -vf scale_cuda=3840:2160 -c:a copy -c:v h264_nvenc -b:v 5M video1.mp4 \
    -vf scale_cuda=1920:1080 -c:a copy -c:v h264_nvenc -b:v 8M video2.mp4 \
    -vf scale_cuda=1280:720 -c:a copy -c:v h264_nvenc -b:v 10M video3.mp4
    ```
- Basic Decode
    ```
    // Decode an input bitstream from input.mp4 to output.yuv
    ffmpeg -y -vsync 0 -c:v h264_cuvid -i input.mp4 output.yuv 
    ```
- Complex Decode Transcode
    ```
    // Decode multiple bitstreams concurrently within single FFmpeg process
    ffmpeg -y -vsync 0 -hwaccel cuda -hwaccel_output_format cuda -i video1.mp4 \
    -hwaccel cuda -hwaccel_output_format cuda -i video2.mp4 \
    -hwaccel cuda -hwaccel_output_format cuda -i video3.mp4 -filter_complex \
    "[0:v]hwdownload,format=nv12[o0];[1:v]hwdownload,format=nv12[o1];[2:v]hwdownload,format=nv12[o2]" \
    -map "[o0]" -f rawvideo output1.yuv -map "[o1]" -f rawvideo output2.yuv -map "[o2]" -f rawvideo output3.yuv 
    ```
- OpenCL Transcode
    ```
    // Upload to 1st GPU device and apply boxblur_opencl filter before downloading to the system
    ffmpeg -init_hw_device opencl=gpu:0.0 -filter_hw_device gpu -i input.mp4 -vf "hwupload, boxblur_opencl, hwdownload" -y output.mp4
    ```

### Measurement
Perform the video encode/decode transcoding on CPU and GPU to compare the performance quality
- Description
    ```
    ffmpeg aceleration options are switched for the measurement
    real - esrtimated average execution time taken
    user - time taken by the program in user mode ***
    sys - time taken by the program in Kernel mode
    ```
- CPU Profiling
    ```
    // Encode input.mp4 to different H.264 videos using CPU
    time ffmpeg -y -vsync 0 -i input.mp4 \
    -c:a copy video1.mp4 \
    -c:a copy video2.mp4 \
    -c:a copy video3.mp4
    
    // Decode multiple bitstreams concurrently within single FFmpeg process on CPU
    time ffmpeg -y -vsync 0 -i video1.mp4 -i video2.mp4 -i video3.mp4 \
    -filter_complex "[0:v]yadif[o0];[1:v]yadif[o1];[2:v]yadif[o2]" \
    -map "[o0]" -f rawvideo output1.yuv -map "[o1]" -f rawvideo output2.yuv -map "[o2]" -f rawvideo output3.yuv
    
    // Normal boxblur application
    time ffmpeg -vsync 0 -i input.mp4 -vf "boxblur=10:10" -y dog_blur_0_5.00_scroll.mp4
    ```
    ```
    Encode
        real    0m13.980s
        user    1m54.527s
        sys     0m2.480s

    Decode
        real    1m7.616s
        user    1m50.414s
        sys     0m22.077s

    Application
        real    4m35.398s
        user    8m9.805s
        sys     0m2.811s
    ```
- GPU Profiling
    ```
    // Encode input.mp4 to different H.264 videos using GPU
    time ffmpeg -y -vsync 0 -hwaccel cuda -hwaccel_output_format cuda -i input.mp4 \
    -c:a copy -c:v h264_nvenc video1.mp4 \
    -c:a copy -c:v h264_nvenc video2.mp4 \
    -c:a copy -c:v h264_nvenc video3.mp4
    
    // Decode multiple bitstreams concurrently within single FFmpeg process on GPU
    time ffmpeg -y -vsync 0 -hwaccel cuda -hwaccel_output_format cuda -i video1.mp4 \
    -hwaccel cuda -hwaccel_output_format cuda -i video2.mp4 \
    -hwaccel cuda -hwaccel_output_format cuda -i video3.mp4 \
    -filter_complex "[0:v]hwdownload,format=nv12[o0];[1:v]hwdownload,format=nv12[o1];[2:v]hwdownload,format=nv12[o2]" \
    -map "[o0]" -f rawvideo output1.yuv -map "[o1]" -f rawvideo output2.yuv -map "[o2]" -f rawvideo output3.yuv
    
    // OpenCL hw accelerated boxblur application
    time ffmpeg -vsync 0 -hwaccel opencl -init_hw_device opencl=gpu:0.0  -i input.mp4 -vf "hwupload,boxblur_opencl=10:10,hwdownload" -y input_blur.mp4
    ```
    ```
    Encode
        real    0m4.785s
        user    0m1.522s
        sys     0m0.803s

    Decode
        real    0m54.296s
        user    0m11.479s
        sys     0m14.483s

    Application
        real    0m25.913s
        user    5m28.857s
        sys     0m3.294s
    ```
- Benchmark Summary
    ```
    Encode
        real    13.98 / 4.785 = 2.92
        user    114.527 / 1.522 = 75.25
        sys     2.48 / 0.803 = 3.09
    
    Decode
        real    67.616 / 54.296 = 1.25
        user    110.414 / 11.479 = 9.62
        sys     22.077 / 14.483 = 1.52
    
    Application
        real    275.398 / 25.913 = 10.63
        user    489.805 / 328.857 = 1.49
        sys     2.811 / 3.294 = 0.85
    
    During above hw acceleration, some encode/decode/application compuatation tasks are offloaded from CPU to GPU
    GPU usage jumps while CPU usage drops, which release and provide more CPU resource for other tasks
    ```
Over all, the system performance get improved through heterogeneous computing by offloading FFmpeg tasks from CPU to GPU.

### Reference
More reference information for decode, encode, filter and configuration


## 4. VLC
Behaving as the player or viewer for RTSP from Jetson
