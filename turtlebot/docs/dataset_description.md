# Record command
The following commands publish static tf and record all needed topics to a bag file. The RGB and depth images are in compressed format. You may need package `image_transport` to convert to raw when using it.  

**Tip for image_transport**  
You can use in a launch file:  
```xml
<node if="$(arg compressed)" name="republish_rgb" type="republish" pkg="image_transport" args="$(arg rgb_image_transport) in:=$(arg rgb_topic) raw out:=$(arg rgb_topic_relay)" />
```

Or use command line:  
```bash
rosrun image_transport republish compressed in:=xxxx/image_raw out:=xxxx/xxxx
rosrun image_transport republish compressedDepth in:=xxxx/image_raw out:=xxxx/xxxx
```

## Record your own rosbag
- **before** recording, additional /tf must be published  
    ```bash
    rosrun tf static_transform_publisher 0 0 0 0 0 1 0 camera_link laser 500
    ```
    OR  
    describe the transform relationship in an **URDF** file and use `robot_state_publisher` to publish static tf.

- record command  
    ```bash
    ~$ roslaunch realsense2_camera rs_camera.launch
    ~$ roslaunch rplidar_ros rplidar_s1.launch
    ~$ rosbag record /camera/aligned_depth_to_color/image_raw/compressedDepth /camera/color/camera_info /camera/color/image_raw/compressed /scan /tf /tf_static -O living_room
    ```


# Bag description
Name                          |     Brief Description                         | Has TF  |  3D
------------------------------|-----------------------------------------------|---------|-----  
elevator_hall_1.bag           | going forwardly                               |    No   |  No
elevator_hall_2.bag           | going backwardly                              |    No   |  No
elevator_hall_3_2rounds.bag   | forward,turn,forward,turn, forward,turn       | **Yes** |  No
living_room.bag               | the living room, somewhere will lose odometry.| **Yes** |  No
multi_1_living_room           | trilogy 1/3, for living room                  |    No   |  No
multi_2_connection_hall.bag   | trilogy 2/3, connect hall                     |    No   |  No
multi_3_kitchen.bag           | trilogy 3/3, kitchen                          |    No   |  No
rtab_mapping.bag              | the author's Z-type hall                      | **Yes** |  No
rtab_multi-session1~3.bag     | the author's multi-sessions mapping demo      | **Yes** |  No
MIT_2012-04-06-11-15-29.bag   | MIT Stata Center, has **groundtruth**         | **Yes** |  No
casual_walk.bag               | LIO-SAM's author, see readme in that package  |    No   | **Yes**
park.bag                      | LIO-SAM's author, see readme in that package  |    No   | **Yes**
heqing_building_1~3.bag       | go around the playground and heqing building  |    No   | **Yes**
circle_road_1~2.bag           | return to start of outdoor road, loop closure |    No   | **Yes**


# Detail for Bags
## > elevator_hall_1.bag
This is a self-recorded dataset on a hand-push wheel car. It goes forward along the elevator hall. To provide TF information, run:
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 1 0 camera_link laser 500
```
**Sensor Types**: RGBD-Image, LaserScan

---
## > elevator_hall_2.bag
This is a self-recorded dataset on a hand-push wheel car. It goes backward along the elevator hall. To provide TF information, run:
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 1 0 camera_link laser 500
```
**Sensor Types**: RGBD-Image, LaserScan

---
## > elevator_hall_3_2rounds.bag
This is a self-recorded dataset on a hand-push wheel car. It goes two rounds along the elevator hall (forward, turn, forward, turn).  
**Sensor Types**: RGBD-Image, LaserScan

---
## > living_room.bag
This is a self-recorded dataset on a hand-push wheel car. It goes around the living room.  
**Sensor Types**: RGBD-Image, LaserScan

---
## > multi_1/2/3_{living_room/connection_hall/kitchen}.bag
This is a self-recorded dataset on a hand-push wheel car. It goes around the living room, connection hall and kitchen of the house. This trilogy is  mainly used to test the multi-session mapping. To provide TF information, run:
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 1 0 camera_link laser 500
```
**Sensor Types**: RGBD-Image, LaserScan

---
## > rtab_{mapping/multi-session1~3}.bag
They are datasets provided by the author of RTAB-MAP. rtab_mapping.bag is a quick start up demo used in `rtabmap_ros/launch/demo/demo_robot_mapping.launch`. The trilogy rtab_multi-session1~3.bag are mainly used to test the multi-session mapping in `rtabmap_ros/launch/demo/demo_multi-session_mapping.launch`.  
**Sensor Types**: RGBD-Image, LaserScan, Odometry

---
## > MIT_2012-04-06-11-15-29.bag
MIT Stata Center, has groundtruth data (timestamp,x,y,z) and has WheelImu. Relatively large indoor indoor dataset captured by a PR2 robot.  
**Sensor Types**: RGBD-Image, LaserScan, Odometry 

- **rosbag play**  
    ```bash
    rosbag play MIT_2012-04-06-11-15-29.bag /base_scan:=/scan /camera/rgb/image_raw:=/camera/color/image_raw /camera/depth/image_raw:=/camera/aligned_depth_to_color/image_raw /camera/rgb/camera_info:=/camera/color/camera_info --clock
    ```

- **rtab, using WheelImu**  
    ```bash
    roslaunch rtabmap_ros rgbd_mapping_rosbag.launch compressed:=false frame_id:=base_footprint visual_odometry:=false odom_frame_id:=odom_combined
    ```

- **rtab, using visual odom**  
Note: the effect of Visual and  Visual+ICP is different!  
    ```bash
    roslaunch rtabmap_ros rgbd_mapping_rosbag.launch compressed:=false frame_id:=base_footprint
    ```

- **rtab, using wheel odom**  
    ```bash
    roslaunch rtabmap_ros rgbd_mapping_rosbag.launch compressed:=false frame_id:=base_link visual_odometry:=false odom_topic:=/base_odometry/odom
    ```

---
## > casual_walk/park.bag
This dataset is provided by the author of LIO-SAM. For more information, see readme in that package.
For quick start, run:
```bash
~$ roslaunch lio_sam author_demo.launch
```

---
## > heqing_building_1~3.bag
This dataset is record in a hand-held device. The first part goes around the square in front of the building. The second and third part go around the Heqing building. To convert the messages to stardard ROS format and coordinate, run:
```bash
~$ roslaunch robot_tools dataset_conversion.launch dataset_name:=heqing
```

To provide TF, use `robot_state_publisher` with a URDF file:
```xml
<?xml version="1.0"?>
<robot name="roboat" xmlns:xacro="http://roboat.org">
  <link name="base_link"></link>
  
  <link name="imu_link"> </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link" />
    <child link="imu_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <link name="rslidar"> </link>
  <joint name="rslidar_joint" type="fixed">
    <parent link="base_link" />
    <child link="rslidar" />
    <origin xyz="0.573 0 0.83463" rpy="-0.01986 -0.03356 -0.01188" />
  </joint>

  <link name="navsat_link"> </link>
  <joint name="navsat_joint" type="fixed">
    <parent link="base_link" />
    <child link="navsat_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
</robot>
```
**Sensor Types**: 9 axis IMU, 32 lines PointCloud, GPS


---
## > circle_road_1~2.bag
This dataset is recorded in a car. It goes along the outdoor road and returns to the start point after a loop. It provides GPS odometry that in UTM coordinate and velocity w.r.t. North/East. To convert the messages to stardard ROS format and coordinate, run:
```bash
~$ roslaunch robot_tools dataset_conversion.launch dataset_name:=circle
```

To provide TF, use `robot_state_publisher` with a URDF file:
```xml

```
**Sensor Types**: 9 axis IMU, 32 lines PointCloud, GPS, UTM odom, North/East velocity, RGB-image

