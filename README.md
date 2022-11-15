# CS637A-Project

This repository contains code that is an implementation of a model free approach to Qlearning inorder to tune the **Proportional**, **Integrator**, **Deriative** constants of a general **Altitude PID Controller**.

## Installation

- Installing Ros (if not present)
```sh 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
sudo apt-get install -y ros-noetic-catkin python3-catkin-tools
sudo apt-get install -y ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox ros-noetic-mavros
```

- Cloning the repository
```sh
git clone --recurse-submodules git@github.com:Jadit19/CS637-Project.git
```

- Initializing the workspace
```sh
cd CS637-Project
catkin init
```

- Building and running
```sh
catkin build -j$(nproc)
source devel/setup.bash
roslaunch controller demo.launch
```