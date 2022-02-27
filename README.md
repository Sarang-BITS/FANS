# FANS : Flying Ad-hoc Network Simulator (no_ros branch)

## Content Description
* [mavad](https://github.com/Sarang-BITS/airborne_networks/tree/main/mavad) : Networking + Planning stack of the co-simulator codebase
* [pci](https://github.com/Sarang-BITS/airborne_networks/tree/main/pci) : (Planner - Control Interface) Interface between **mavad** and **MAVROS** to further connect to PX4-SITL in Gazebo


## Setup and demo
* To use this project, you need to install ROS, MAVROS, PX4-SITL and NS3 (using CMake). Note that this project has been tested on Ubuntu 18.04 OS and ROS Melodic distribution. All the instructions to install these dependencies are given in the [SETUP.md](https://github.com/Sarang-BITS/airborne_networks/blob/main/SETUP.md) file
* Clone this repo in your systems and copy [mavad](https://github.com/Sarang-BITS/FANS/tree/main/mavad) to `ns3-all-in-one/NS3`
```bash
cd ~
git clone https://github.com/Sarang-BITS/FANS.git
git checkout no_ros
cp -r ~/FANS/mavad <path_to_ns3-all-in-one>/NS3/
```
* Edit the `CMakeLists.txt` file of `ns3-all-in-one/NS3` to build `mavad`
```bash
cd <path_to_ns3-all-in-one>/NS3/
gedit CMakeLists.txt 
```
* Add the following contents in the CMakelists.txt file and enable the option to build using emulation support
```bash
# Build mavad scripts
add_subdirectory(mavad)

option(NS3_EMU "Build with emu support" ON)
set(NS3_EMU ON)
```
* Build `mavad`. After the build is successful, you would find an executable `mavad_main` inside `<path_to_ns3-all-in-one>/NS3/build`.
```bash
cd <path_to_ns3-all-in-one>/NS3/cmake-cache
make
```
* **Running the simulation demo**
    ~~* Launch the the drones with PX4 autopilot and MAVROS in Gazebo (in terminal 1). You should see 8 unarmed, landed drones in the Gazebo simulator window~~
    ```bash
    source <path_to_sop_ws>/devel/setup.bash
    roslaunch px4 sitl8drones.launch
    ```
    ~~* Launch the planner control interface for the drones (in terminal 2). You should see the 8 drones armed and hovering at a certain height in the Gazebo simulator window~~
    ```bash
    source <path_to_sop_ws>/devel/setup.bash
    roslaunch pci pci8drones.launch
    ```
    * Launch the network simulator and planner stack (in terminal ~3~ 1). You should see the IP initialization and message communication logs in the terminal and the formation motion of drones in the Gazebo simulator window
    ```
    cd <path_to_ns3-all-in-one>/NS3/build
    ./mavad_main
    ```
    * Fire up the NetAnim visualizer to visualize message communication between nodes and their positions and open the XML trace file `<path_to-ns3-all-in-one>/NS3/build/planner_ns3_anim.xml` in it
    ```bash
    cd <path_to-ns3-all-in-one>/netanim
    ./NetAnim
    ```


## Contributors
* [Sarang Dhongdi](https://github.com/Sarang-BITS)
* [Ojit Mehta](https://github.com/ojitmehta123)
* [Mihir Dharmadhikari](https://github.com/MihirDharmadhikari)
* [Vaibhav Agarwal](https://www.github.com/agvab0811)
* [Vishal Singh](https://www.github.com/vishalbhsc)
* [Shambhavi Singh](https://www.github.com/28shambhavi)
* [Aditya Bidwai](https://www.github.com/adbidwai)


## Acknowledgement
We would like to extend our special thanks of gratitude to [Dr. Mohit P. Tahiliani](https://github.com/mohittahiliani) for his valuable guidance throughout the course of this project


## Branches
* **main** : Contains the entire co-simulator setup which integrates NS3 (networking) and PX4-SITL in Gazebo (robot simulator)
* **no_ros** : Contains the networking + planner stack with visualizations in NetAnim
