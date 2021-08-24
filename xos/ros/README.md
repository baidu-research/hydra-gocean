# Install ROS
Install Robot Operating System (ROS) on NVIDIA Jetson Nano Developer Kit

These scripts will install Robot Operating System (ROS) on the NVIDIA Jetson Nano Developer Kit.

The script is based on the Ubuntu ARM install of ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu

Maintainer of ARM builds for ROS is http://answers.ros.org/users/1034/ahendrix/

There are two scripts:

<strong>install-ros.sh</strong>
<pre>
Usage: ./install-ros.sh  [[-p package] | [-h]]
 -p | --package &lt;packagename&gt;  ROS package to install
                               Multiple Usage allowed
                               The first package should be a base package. One of the following:
                                 ros-melodic-ros-base
                                 ros-melodic-desktop
                                 ros-melodic-desktop-full
 </pre>
 
Default is ros-melodic-ros-base if no packages are specified.

Example Usage:

$ ./install-ros.sh -p ros-melodic-desktop -p ros-melodic-rgbd-launch

This script installs a baseline ROS environment. There are several tasks:

<ul>
<li>Enable repositories universe, multiverse, and restricted</li>
<li>Adds the ROS sources list</li>
<li>Sets the needed keys</li>
<li>Loads specified ROS packages, defaults to ros-melodic-base-ros if none specified</li>
<li>Initializes rosdep</li>
</ul>

You can edit this file to add the ROS packages for your application. 

<strong>setup-catkin-workspace.sh</strong>
Usage:

$ ./setup-catkin-workspace.sh [optional workspace name]

where [optional workspace name] is the name of the workspace to be used. The default workspace name is catkin_ws. This script also sets up some ROS environment variables. Refer to the script for details.


## History
<strong>October 2019</strong>
* vL4T32.2.1
* L4T 32.2.1 (JetPack 4.2.2)

<strong>July 2019</strong>
* vL4T32.2
* L4T 32.2 (JetPack 4.2.1)

<strong>June 2019</strong>
* vL4T32.1.0
* L4T 32.1.0 (JetPack 4.2)
* Update ROS server GPG key

<strong>March 2019</strong>
* Initial Release
* L4T 32.1.0 (JetPack 4.2)

