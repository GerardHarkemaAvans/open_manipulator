#!/bin/bash

sudo apt update
sudo apt-get install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench*
sudo apt-get install ros-melodic-robotis-manipulator

cd $HOME/open_manipulator_ws/
catkin b

echo "source $HOME/open_manipulator_ws/devel/setup.bash" >> ~/.bashrc
sudo adduser $USER dialout
