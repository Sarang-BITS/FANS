# FANS : Flying Ad-hoc Network Simulator (main branch)

## Content Description
* [mavad](https://github.com/Sarang-BITS/airborne_networks/tree/main/mavad) : Networking + Planning stack of the cosimulator codebase
* [pci](https://github.com/Sarang-BITS/airborne_networks/tree/main/pci) : (Planner - Control Interface) Interface between **mavad** and **MAVROS** to further connect to PX4-SITL in Gazebo

## Setup and demo
* To use this project, you need to install ROS, MAVROS, PX4-SITL and NS3 (using CMake). Note that this project has been tested on Ubuntu 18.04 OS and ROS Melodic distribution. All the instructions to install these dependencies are given in the [SETUP.md](https://github.com/Sarang-BITS/airborne_networks/blob/main/SETUP.md) file


## Contributors
* [Sarang Dhongdi](https://github.com/Sarang-BITS)
* [Ojit Mehta](https://github.com/ojitmehta123)
* [Mihir Dharmadhikari](https://github.com/MihirDharmadhikari)
* [Vaibhav Agarwal](https://www.github.com/agvab0811)
* [Vishal Singh](https://www.github.com/vishalbhsc)
* [Shambhavi Singh](https://www.github.com/28shambhavi)
* [Aditya Bidwai](https://www.github.com/adbidwai)

## Branches
* **main** : Contains the entire co-simulator setup which integrates NS3 (networking) and PX4-SITL in Gazebo (robot simulator)
* **no_ros** : Contains the networking + planner stack with visualizations in NetAnim
