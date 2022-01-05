# Multimedia
The Multimedia provides application development by enabling flexibility for better control over the underlying hardware blocks to leverage custom frameworks. Here are the topics currently included in our XCORE architecture design
- Codec
- GStreamer


## Codec
The NVIDIA codec package supports hardware-accelerated decode on Jetson platforms using FFmpeg, application can use accelerated decode to read video files in the following elementary/container formats and dump them in YUV 420 format
- H.264/AVC/265
- VP8/9
- MPEG2/4
- HEVC

### Basic Setup
- Install binary & source
    ```
    sudo apt install ffmpeg
    apt source ffmpeg
    ```
- Include ffmpeg libs in L4T builds
    ```
    echo "deb https://repo.download.nvidia.com/jetson/ffmpeg main main" |  sudo tee -a /etc/apt/sources.list
    echo "deb-src https://repo.download.nvidia.com/jetson/ffmpeg main main" |  sudo tee -a /etc/apt/sources.list
    ```
- Update & Make
    ```
    sudo apt update
    make && sudo make install
    ```

### Optimized Setup
- Build & Install
    ```
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig
    ```
- Patch ffmpeg to rebuild
    ```
    git clone git://source.ffmpeg.org/ffmpeg.git -b release/4.2 --depth=1
    cd ffmpeg
    wget https://github.com/jocover/jetson-ffmpeg/raw/master/ffmpeg_nvmpi.patch
    git apply ffmpeg_nvmpi.patch
    ./configure --enable-nvmpi
    make
    ```

### Examples
Examples on Jetson GPU platform
```
ffmpeg -c:v h264_nvmpi -i input_file -f null -
ffmpeg -i input_file -c:v h264_nvmpi <output.mp4>
ffmpeg -i <input.mp4> -pix_fmt yuv420p -c:v rawvideo -an -s 2880x1860 -y <output.yuv>
```


## GStreamer
GStreamer is a pipeline-based open source multimedia framework that links together a wide variety of media processing systems to complete complex workflows, which can be used together with NvGstPlayer utility for testing multimedia local playback and HTTP/RTSP streaming playback use cases:

- Multimedia Playback
- Camera Capture

GStreamer supports a wide variety of media-handling components, including simple audio playback, audio and video playback, recording, streaming and editing. 
The pipeline design serves as a base to create many types of multimedia applications such as video editors, transcoders, streaming media broadcasters and media players, while above listed features are mainly used on Jetson GPU platforms.

