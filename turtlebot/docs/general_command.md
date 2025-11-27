# change owner and group of all files/sub-dirs in the dir
- change owner: `chown -R owner file/dir`
- change group: `chgrp -R group file/dir`
- change mode: `chmod xxx file/dir`
- change owner and group: `chown -R owner:group file/dir`  
**Note** : '-R' means recursively.


# connect to remote computer
```bash
ssh username@ip_addr
sudo mount -t nfs 192.168.2.51:/ /mnt -o nolock
---------------------------------
sudo umount /mnt
```

# time synchronize between computers
** Remember to do it everytime start up TX2!!!!**
If you are using ROS across computers, it is best for you to synchronize their system time to avoid different timestamps. Make sure **both** computers have installed `ntpdate`:
```bash
~$ sudo apt install ntpdate
```
Then. on the computer that need to **be synchronized**:
```bash
~$ sudo ntpdate -u 10.42.0.140
~$ sudo hwclock -w
```



# when gazebo unsuccessfully closed (black screen)
```bash
ps -aux|grep gz
kill xxxx_pid
```


# add Hokuyo laser to turtlebot_gazebo
see this for reference:
> "https://blog.csdn.net/Changer_sun/article/details/79264388"


# world files
for more choices of world files, please refer to [world_description.md](docs/world_description.md)


# show a certain tf transform
```bash
rosrun tf tf_echo base_link laser
```


# rtabmap author's demo
This demo uses tf to get odom, and odom information is used directly instead of visual odometry.
```bash
roslaunch rtabmap_ros demo_robot_mapping.launch
rosbag play --clock rtab_mapping.bag
```


# rtabmap, rosbag or direct sensors
- (self-recorded) rosbag  
  if *no* static tf is provided, which you need to first run
  ```bash
  rosrun tf static_transform_publisher 0 0 0 0 0 1 0 camera_link laser 500
  ```
  before you play the dataset. 

  Then you can run  
  ```bash
  roslaunch rtabmap_ros rgbd_mapping_rosbag.launch
  rosbag play --clock xxx.bag
  ```

- direct sensors  
  TF relation of each sensor must be properly config in launch file [rgbd_mapping_camera.launch](./rtabmap_ros/launch/my_work/rgbd_mapping_camera.launch)
  ```xml
  <node pkg="tf" type="static_transform_publisher" name="static_tf_pub" args="0 0 0 0 0 1 0 camera_link laser 100" output="screen"/>
  ```
  Then you can run
  ```bash
  ~$ roslaunch realsense2_camera rs_camera.launch
  ~$ roslaunch rplidar_ros rplidar_s1.launch
  ~$ roslaunch rtabmap_ros rgbd_mapping_camera.launch
  ```


# gazebo keyboard control turtlebot mapping
```
~$ roslaunch turtlebot_gazebo my_turtlebot_world.launch world_file:=/home/derek/catkin_ws/src/autonomous_exploration/turtlebot_gazebo/worlds/my_simple_closure2.world
~$ roslaunch robot_tools keyboard_teleop.launch --screen
~$ roslaunch rtabmap_ros turtlebot_mapping.launch simulation:=true rviz_explor_method:=none
```


# gazebo + rtabmap + move_base + rrt_explore / explore_lite
- **rrt_explore**
  ```bash
  ~$ roslaunch turtlebot_gazebo my_turtlebot_world.launch world_file:=/home/derek/catkin_ws/src/autonomous_exploration/turtlebot_gazebo/worlds/my_simple_closure2.world
  ~$ roslaunch rtabmap_ros turtlebot_mapping.launch simulation:=true rviz_explor_method:=rrt
  ~$ roslaunch rrt_exploration simple.launch
  #~$ roslaunch robot_tools keyboard_teleop.launch --screen
  ```
- **explore_lite**
  ```bash
  ~$ roslaunch turtlebot_gazebo my_turtlebot_world.launch world_file:=/home/derek/catkin_ws/src/autonomous_exploration/turtlebot_gazebo/worlds/my_small_maze.world
  ~$ roslaunch rtabmap_ros turtlebot_mapping.launch simulation:=true rviz_explor_method:=explore_lite
  ~$ roslaunch explore_lite explore.launch
  #~$ roslaunch robot_tools keyboard_teleop.launch --screen
  ```


# real turtlebot mapping + navigation + exploration
First, ssh connect to turtlebot **TX2**, then:
```bash
# in TX2
~$ roslaunch turtlebot_bringup minimal.launch
~$ roslaunch realsense2_camera rs_camera.launch
~$ roslaunch rplidar_ros rplidar_s1.launch
```

Then, on the remote **PC**, choose one:
- for only keyboard control mapping and navigation  
  ```bash
  # remote PC
  ~$ roslaunch rtabmap_ros turtlebot_mapping.launch rviz_explor_method:=none
  ~$ roslaunch robot_tools keyboard_teleop.launch --screen
  ```

- for rrt exploration
  ```bash
  # remote PC
  ~$ roslaunch rtabmap_ros turtlebot_mapping.launch rviz_explor_method:=rrt
  ~$ roslaunch rrt_exploration simple.launch
  ```

- for explore-lite exploration
  ```bash
  # remote PC
  ~$ roslaunch rtabmap_ros turtlebot_mapping.launch rviz_explor_method:=explore_lite
  ~$ roslaunch explore_lite explore.launch
  ```


# lower obstacle avoidance
This function is utilized in pkg `depthimage_to_laserscan`, you can:
- just test `laser_from_depth`:
  ```bash
  ~$ roslaunch depthimage_to_laserscan laser_from_depth.launch
  ```

- test it with mapping and costmap:
  ```bash
  # real robot
  ~$ roslaunch rtabmap_ros turtlebot_mapping.launch rviz_explor_method:=none obstacle_avoidance:=true
  
  # simulation
  ~$ roslaunch rtabmap_ros turtlebot_mapping.launch simulation:=true rviz_explor_method:=none obstacle_avoidance:=true
  ```
