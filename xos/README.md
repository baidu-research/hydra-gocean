# XOS
XOS consists four components:
```
1. OS
2. ROS
3. DS
4. Host
```


## 1. OS
Ubuntu 18 LTS comes with Jetson platforms from Nvidia


## 2. ROS
There are different versions of ROS available:
```
May 23rd, 2020 - May, 2025     Noetic Ninjemys      Ubuntu 20.04 (Focal) release
May 23rd, 2018 - May, 2023     Melodic Morenia      Ubuntu 18.04 (Bionic) release
May 23rd, 2017 - May, 2019     Lunar Loggerhead     Ubuntu 17.04 (Zesty) release
......
```
We are taking Melodic Morenia for now because of Ubuntu OS compatibility


## 3. DS
Build and deploy AI powered intelligent video analytics apps and services, deepstream offers a multi-platform scalable framework with TLS security to deploy on the edge and connect to any cloud.

### Setup
The following development packages are installed:
    ```
    GStreamer-1.0
    GStreamer-1.0 Base Plugins
    GStreamer-1.0 gstrtspserver
    X11 client-side library
    ```

- Prerequisites
    ```
    sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgstrtspserver-1.0-dev libx11-dev libjson-glib-dev
    ```

- Package 
    ```
    download ds4.bz2 or ds5.bz2 to ~/deepstream/dsX
    more detail at dsX/README
    cd ~/deepstream/dsX     // dsX could be ds4, ds5 or ds51
    tar xvfj dsX.bz2
    ```

- Build
    ```
    cd ~/deepstream/ds4/sources/apps/sample_apps/deepstream-app
    make
    ```
 
- Execute
    ```
    ./deepstream-app -c <config-file>
    ```

### Testing
There are different testing configurations available at:
    ```
    ~/deepstream/dsX/samples/configs/deepstream-app
    ```

### Applications

- BRI RAL Pangu 360 Camera

- ACG EasyMonitor Prototype

- Vidpress


## 4. Host
Applications on host to communicate with Jetson

