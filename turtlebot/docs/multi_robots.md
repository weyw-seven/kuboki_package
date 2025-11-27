# Multi-robot Project
## Description
- Namespace: we use robotx as namespace (where x is the number started from 1). For example, topic /robot1/odom, /robot2/scan. For tf frames, no leading slash is presented, such as robot1/odom, robot2/laser_link.

- URDF file: Defferent robots have their own URDF file in `$(find turtlebot_description)/robot/[gazebo]`. For simulation, the name of camera sensors in `$(find turtlebot_description)/urdf/[gazebo]` in different robots should be different. Their naming rule is **prefix + <colorSensorName/depthSensorName/ired1,2SensorName>**. For example, if `prefix` = "camera_", `<colorSensorName>` = "color_robot1", then the name in tag should be "camera_color_robot1". The default values for tag `<colorSensorName>/<depthSensorName>/<ired1,2SensorName>` are "color/depth/ired1,2", if these tags are not defined.

## Usage
- Launch in Simulation: 
    ```bash
    ~$ roslaunch turtlebot_gazebo two_robots.launch world_file:=/home/derek/catkin_ws/src/autonomous_exploration/turtlebot_gazebo/worlds/my_simple_closure2.world
    ~$ roslaunch robot_tools keyboard_teleop.launch namespace:=robot2
    ```

