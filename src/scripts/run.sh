#! /bin/sh

clear
catkin build -j4
source ../devel/setup.bash
roslaunch controller demo.launch