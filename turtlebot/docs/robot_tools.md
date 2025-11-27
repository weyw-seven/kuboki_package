# Description
This document explains some tools in the package `robot_tools`.


# Usage
## robot_tools.py
A self-written tool that provides convenient GUI for some frequently used command. This tool is for turtlebot2 with kobuki base and RTAB-Map SLAM.
```bash
~$ roslaunch robot_tools robot_tools.launch
```
---

## /tf filter
This tool can filter/remap frames(links) in /tf that you specified in the launch file.
```bash
~$ roslaunch robot_tools remap_tf_frames.launch
```
---

## upgraded robot keyboard controller
The original keyboard controller in package `turtlebot_teleop` is not proper for safety control, because robot will not accept move_base command while it is running.

To realize that function, we modified the original script. Furthermore, the initial velocity can be specified in the launch file.
```bash
~$ roslaunch robot_tools keyboard_teleop.launch # single robot
# OR
~$ roslaunch robot_tools keyboard_teleop.launch namespace:=robot2 # multi-robot
```
---

## URDF/xacro robot modal visualizer
Robot modals (including shapes, links, joints etc.) are usually defined in urdf.xacro file in ROS. This tool provide you chances to visualize it quickly.
```bash
~$ roslaunch robot_tools visualize_robot.launch
# if you want to see the tf tree, use rqt.
```
---

## generate ArUco Markers
This tool generates ArUco markers according to the params specified in the launch file. The generated markers will be saved as images in the folder.
```bash
~$ roslaunch robot_tools generate_aruco_markers.launch
```
