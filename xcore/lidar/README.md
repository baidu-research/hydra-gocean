# LiDAR Projects
LiDAR is a method for determining ranges by targeting an object with a laser and measuring the time for the reflected light to return to the receiver, which can be used to make digital 3-D representations of areas on the earth's surface and ocean bottom, due to differences in laser return times, and by varying laser wavelengths.


# Livox LiDAR
There are 4 sections covered which includes:
```
1. Livox SDK
2. Livox ROS Driver
3. Livox Mapping
4. Livox Viewer
```
The installation should following above order to be deployed.

## 1. Livox SDK
Livox SDK is the software development kit designed for all Livox products. It is developed based on C/C++ following Livox SDK Communication Protocol, and provides easy-to-use C style API. With Livox SDK, users can quickly connect to Livox products and receive point cloud data. Livox SDK consists of Livox SDK communication protocol, Livox SDK core, Livox SDK API, Linux sample, and ROS demo. 

### Prerequisites
* Ubuntu 14.04/Ubuntu 16.04/Ubuntu 18.04, both x86 and ARM (Nvidia TX2)
* Windows 7/10, Visual Studio 2015 Update3/2017/2019
* C++11 compiler

### Dependencies
Livox SDK requires [CMake 3.0.0+](https://cmake.org/) as dependencies. You can install these packages using apt:
```
$ cd ~/livox
$ sudo apt install cmake
```

#### Build
Download Livox SDK directory to compile the project:
```
$ cd ~/livox
$ git clone https://github.com/Livox-SDK/Livox-SDK.git
$ cd Livox-SDK
$ cd build && cmake ..
$ make
$ sudo make install
```

#### Reference
https://github.com/Livox-SDK/Livox-SDK

## 2. Livox ROS Driver
livox_ros_driver is a new ROS package, specially used to connect LiDAR products produced by Livox. The driver can be run under ubuntu 14.04/16.04/18.04 operating system with ROS environment (indigo, kinetic, melodic) installed. Tested hardware platforms that can run livox_ros_driver include: Intel x86 cpu platforms, and some ARM64 hardware platforms (such as nvida TX2 / Xavier, etc.).

### Prerequisites
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. 
[ROS Installation](http://wiki.ros.org/ROS/Installation)
```
sudo apt-get install ros-melodic-perception-pcl
```

### Dependencies
Before running livox_ros_driver, ROS and Livox-SDK must be installed.

### Build 
```
Get livox_ros_driver:
$ cd ~/livox
$ git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src

Use the following command to build livox_ros_driver:
$ cd ws_livox
$ catkin_make

Use the following command to update the current ROS package environment:
$ source ./devel/setup.sh
```

#### Reference
https://github.com/Livox-SDK/livox_ros_driver

## 3. Livox Mapping
Livox_mapping is a mapping package for Livox LiDARs. The package currently contains the basic functions of low-speed mapping.

### Prerequisites
Ubuntu 64-bit 16.04 or 18.04
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
```
sudo apt-get install ros-melodic-perception-pcl
```
- Follow [PCL Installation](https://pointclouds.org/downloads/#linux)
- Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- Follow [openCV Installation](https://opencv.org/releases/)
- Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver)

### Dependencies
Before running livox_ros_driver, ROS and Livox-SDK must be installed.

### Build
Clone the repository and catkin_make:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Livox-SDK/livox_mapping.git
$ cd ..
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

#### Reference
https://github.com/Livox-SDK/livox_mapping

## 4. Livox Viewer
https://www.livoxtech.com/

### Setup
Only supports x86 64-bit computers with Linux

### Install
Please open a terminal console and then enter into this directory to run the following command:
```
$ cd ~/livox/viewer
Download data.tar.bz2
$ tar xvfj data.tar.bz2
Download so.tar.bz2
$ tar xvfj so.tar.bz2
```

### Execute
Please issue the following command:
```
$ cd ~/livox/viewer
$ ./livox_viewer.sh
```


# Velodyne


# Hesai


# Innovusion


