# Autonomous Exploration
## Description
This project is part of the research that aims to enable robot to explore unknown environment autonomously and interact with the environment. The main tasks of the robot are to patrol indoor and grab objects from the scene using its arm. While patroling, obstacles should be avoided automatically and interactive navigation is also enabled. Since no priori about the scene is required, certain exploration strategies are employed and a global occupancy grid map and pointcloud (and/or octomap) are built for navigation.

The main field of the research is self-learning intelligent unmaned technology, while this project only focus on three smaller and specific domain, i.e. **SLAM**, **navigation** and **exploration**. Because autonomous exploration is one of our main task and it simultaneously utilizes the other two technologies, attention will be largely given to it. In fact, the corporation of all three parts is important, so improvement may occur in any of them.

The effects can be seen both through simulation and real robot experiment. The development of this project is on the basis of packages **rtabmap**, **move_base** and **explore-lite**. Gazebo is used for simulation and turtlebot2 is used for experiment. The sensors include Intel(R) realsense D435i depth camera and Slamtec(R) Rplidar S1 laser rangefinder.

## Installation
There are three stacks you need: a turtlebot2 (with depth camera and laser rangefinder), a controller for turtlebot2 and a remote computer. In this project, we used Nvidia (R) Jeston TX2 as the controller on robot and a PC as a remote computer. If you want wireless control, a WIFI router is also needed.

If you just want simulation, you can only focus on the steps for the remote PC. But if you want to conduct experiment on real robot, both PC and TX2 need to be configured.

**Note** that for simulation, we have added a rplidar laser rangefinder to the robot instead of the original from-depth method to acquire laser scan data. You need to use the modified files from this repo to replace the original ones.

### Remote PC (Ubuntu 16.04 ROS Kinetic)
Since we need ROS, so a computer with Ubuntu is needed (tested on Ubuntu 16.04 LTS). The steps are as follows:
1. Install ROS Kinetic desktop full version.
2. Install gazebo models (models are available in ./data folder).
3. Install turtlebot2 related packages:
   ```bash
   ~$ sudo apt install ros-kinetic-turtlebot  ros-kinetic-turtlebot-bringup  ros-kinetic-turtlebot-capabilities  ros-kinetic-turtlebot-description  ros-kinetic-turtlebot-gazebo  ros-kinetic-turtlebot-navigation  ros-kinetic-turtlebot-rviz-launchers  ros-kinetic-turtlebot-teleop
   ```
4. Copy the packages `turtlebot-description`, `turtlebot-gazebo` and `turtlebot-navigation` in this repo to replace the installed ones (remember to backup the original ones):
   ```bash
   ~$ sudo cd autonomous_exploration
   ~/autonomous_exploration$ cp -rf turtlebot-description turtlebot-gazebo turtlebot-navigation /opt/ros/kinetic/share
   ```
5. Copy the main packages to catkin folder:
   ```bash
   ~/autonomous_exploration$ cp -r explore-lite rtabmap_ros    ~/catkin_ws/src
   ```
6. Install rtabmap SDK library. Please refer to [rtabmap_wiki](https://github.com/introlab/rtabmap_ros#installation)
7. Compile the ros packages:
   ```bash
   ~$ cd ~/catkin_ws
   ~/catkin_ws$ catkin_make
   ```

### PC/NUC/Laptop (Ubuntu 18.04 ROS Melodic)
Since we need ROS, so a computer with Ubuntu is needed. If you don't need some module or function (such as simulation in gazebo), just skip the corresponding steps. We assume that you do the following steps from a **newly installed** Ubuntu.
1. Upgrade system packages.
   ```bash
   # configure the source first if you want faster speed
   ~$ sudo apt update
   ~$ sudo apt upgrade
   ```
2. Install ROS. Please refer to [website](http://wiki.ros.org/melodic/Installation/Ubuntu) for instruction. Install the `desktop full` version to have access to all functions. If you don't want simulation, `desktop` version is also OK.
3. Install OpenCV 4.1.1 with extra modules (opencv_contrib). Remember to enable `Non-Free` function.
4. Some dependent package.
   ```bash
   ~$ sudo apt install ros-melodic-rtabmap ros-melodic-rtabmap-ros
   ~$ sudo apt remove ros-melodic-rtabmap ros-melodic-rtabmap-ros
   ```
5. Install Ceres. 
   ```bash
   ~$ sudo apt install libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
   ~$ cd ceres-1.14.0/build
   ~/ceres-1.14.0/build$ make -j8
   ~/ceres-1.14.0/build$ sudo make install
   ```
6. Install g2o. Use `g2o-master.zip`. Option `BUILD_WITH_MARCH_NATIVE` should be **OFF**.
7. Install gtsam. Option `GTSAM_USE_SYSTEM_EIGEN, GTSAM_WITH_EIGEN_MKL, GTSAM_WITH_EIGEN_MKL_OPENMP` should be **ON**
8. Install libnabo-1.0.7 and libpointmatcher-1.3.1.
9. Install Rtabmap standalone library. Option `-DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel` to set install folder.
   ```bash
   ~$ sudo apt remove ros-melodic-libg2o
   ~$ cd rtabmap-master/build
   ~/rtabmap-master/build$ cmake .. -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel
   ~/rtabmap-master/build$ make -j8
   ~/rtabmap-master/build$ make install
   ```
10. 


### TX2 (JetPack 3.1 ROS Kinetic)
Since we need ROS, so a computer with Ubuntu is needed (tested on Ubuntu 16.04 jetpack). The steps are as follows:
1. Install ROS Kinetic desktop.
2. Install turtlebot2 related packages:
   ```bash
   ~$ sudo apt install ros-kinetic-turtlebot    ros-kinetic-turtlebot-bringup
   ```
3. Install realsense SDK 2.0 and realsense_ros. Please refer to [SDK installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) and [realsense_ros installation](https://github.com/IntelRealSense/realsense-ros). **Note** that the source codes of tested version are available in the Tsinghua Cloud.
4. Compile other ros packages:
   ```bash
   ~/autonomous_exploration$ cp -r realsense-ros rplidar_ros    ~/catkin_ws/src
   ~/autonomous_exploration$ cd ~/catkin_ws
   ~/catkin_ws$ catkin_make
   ```
5. install time synchronize tool `ntpdate` because the time of TX2 is reset when it shutdowns. Please refer to [general_command.md](./docs/general_command.md)


### TX2 (JetPack 4.4 ROS Melodic)
In Ubuntu 18.04, ROS Melodic is used. However, some packages related to turtlebot are not available in this ros distro. Two additional packages (i.e. `kobuki_desktop` and `kobuki-melodic`)need to copy to `catkin_ws/src` first. Then follow the steps below:
1. Install some required packages through 'apt-get':
```bash
~$ sudo apt install ros-melodic-ecl-exceptions ros-melodic-ecl-threads ros-melodic-ecl-geometry ros-melodic-ecl-streams ros-melodic-kobuki-msgs ros-melodic-yocs-controllers ros-melodic-yocs-cmd-vel-mux ros-melodic-yocs-velocity-smoother ros-melodic-kobuki-dock-drive ros-melodic-kobuki-driver ros-melodic-compressed-image-transport ros-melodic-compressed-depth-image-transport
```
2. catkin_make the src folder:
```bash
~/catkin_ws$ catkin_make
```
3. create udev rules for rplidar and kobuki:
```bash
~/catkin_ws/src/rplidar_ros/scripts$ ./create_udev_rules.sh # no sudo!!
~$ sudo apt install ros-melodic-kobuki-ftdi
~$ rosrun kobuki_ftdi create_udev_rules
```
4. install realsense sdk and ros packages. See [realsense_ros](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md) github.
5. install time synchronize tool `ntpdate` because the time of TX2 is reset when it shutdowns. Please refer to [general_command.md](./docs/general_command.md)


## Usage
Demos and some common commands are in [general_command.md](./docs/general_command.md).

## World files
For simulation in gazebo, some test world files (.world) are available in `./turtlebot_gazebo/worlds`. Description of them can be found at [world_description.md](./docs/world_description.md).

## Dataset
Rosbag dataset is a good way to compare the effect of different SLAM methods or different parameters of the same method. We provide some online public dataset and self recorded dataset in `./data/rosbag_dataset`. Please refer to [dataset_description.md](./docs/dataset_description.md) for more information.
