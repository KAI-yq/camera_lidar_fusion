# camera_lidar_fusion
## 1. Install：catkin_make

## 2. Preparation：
### 2.1 modify intrinsic and extrinsic in /data/parameters/intrinsic.txt and /data/parameters/extrinsic.txt
### 2.2 modify the name and path of the bag in /launch/fusion.launch
### 2.3 modify the lidar_topic and camera_topic in /launch/fusion.launch

## 3. Usage:
'source devel/setup.bash'
'roslaunch camera_lidar_fusion fusion.launch'
