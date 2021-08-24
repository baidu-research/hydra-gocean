===============================================================================
eCAM_Argus_MultiCamera API
===============================================================================

Included Components
-------------------

 - Argus API headers ('include/')
   These headers are required to compile applications against the Argus API.

 - Utility Source ('utility/')
   The included utility directory handles libargus/V4L2/Tegra MMAPI calls handled in the eCAM_Argus_MultiCamera application.

Building Samples And Apps
=========================

Requirements
------------

- cmake
  sudo apt-get install cmake

- build-essential
  sudo apt-get install build-essential

- pkg-config
  sudo apt-get install pkg-config

- X11
  sudo apt-get install libx11-dev

- GTK3
  sudo apt-get install libgtk-3-dev

- expat
  sudo apt-get install libexpat1-dev

- JPEG
  sudo apt-get install libjpeg-dev

- gstreamer-1.0
  sudo apt-get install libgstreamer1.0-dev

Building and Installing
-----------------------

Start in the source directory (e.g. '~/ArgusSDK/').

  1) Create and go into a build directory:
       'mkdir build && cd build'

  1) Create the makefiles:
       'cmake ..'
     Additional options:
       - If CMake cannot find an include path for any dependencies, it may be
         required to provide them explicitly. Eg:
           'cmake -DOPENGLES_INCLUDE_DIR=/path/to/khronos/includes ..'
       - The DISABLE_MULTIPROCESS option may be provided to use the single-process
         Argus implementation (ie. does not require nvargus-daemon service):
           'cmake -DDISABLE_MULTIPROCESS=ON ..'

  2) Build all apps (optional flag -jN for parallel build where N is the job count):
       'make [-jN]'

  3) Install:
       'sudo make install'
