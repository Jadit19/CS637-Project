# CS637A-Project

This repository contains code that is an implementation of a model free approach to Qlearning inorder to tune the **Proportional**, **Integrator**, **Deriative** constants of a general **Altitude PID Controller**.

## Installation

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
catkin build -j4
source devel/setup.bash
roslaunch controller demo.launch
```