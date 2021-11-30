#!/bin/bash

sudo apt update
sudo apt-get install python3-catkin-tools
sudo apt-get install ros-melodic-ros-controllers ros-melodic-gazebo* ros-melodic-moveit* ros-melodic-industrial-core
sudo apt-get install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench*
sudo apt-get install ros-melodic-robotis-manipulator
sudo apt-get install ros-melodic-rgbd-launch ros-melodic-libuvc-camera
sudo apt-get install ros-melodic-usb-cam

sudo apt-get install ros-melodic-ar-track-alvar ros-melodic-ar-track-alvar-msgs ros-melodic-image-proc

sudo apt install ros-$ROS_DISTRO-flexbe-behavior-engine
mkdir $HOME/rospackages_ws
mkdir $HOME/rospackages_ws/src
cd $HOME/rospackages_ws/src
git clone https://github.com/FlexBE/flexbe_app.git
cd $HOME/rospackages_ws
catkin b
echo "source $HOME/rospackages_ws/devel/setup.bash" >> ~/.bashrc
echo "source $HOME/open_manipulator_ws/devel/setup.bash" >> ~/.bashrc
sudo adduser $USER dialout
