# NXROBO scorpio
<img src="https://raw.githubusercontent.com/NXROBO/scorpio/master/src/scorpio/scorpio_description/pic/scorpio.jpg" width="600">

## 说明 Description
- This is a tutorial for beginners.
- 本说明为初学者体验版。

## 列表 Table of Contents

* [功能包说明packages-overview](#功能包说明packages-overview)
* [使用usage](#使用usage)
* [视频展示Video](#视频展示Video)

## 功能包说明packages-overview

* ***src*** : scorpio的源代码，包括底层配置，硬件驱动，和各个应用功能包等。
* ***doc*** : 软硬件依赖包。

## 使用usage

### 系统要求 Prequirement

* System:	Ubuntu 22.04+
* ROS2 Version:	humble(Desktop-Full Install) 

### 下载安装 Download and install

* 下载工作空间 Download the workspace:
```yaml
git clone https://github.com/NXROBO/scorpio_ros2.git
```
* 安装依赖库 Install libraries and dependencies:
```yaml
cd scorpio_ros2
```

### 编译运行 compile and run
```yaml
colcon build
```
* 如果编译一切正常，可根据提示运行相关例程。If everything goes fine, test the examples as follow:
```yaml
./onekey.sh
```

## 视频展示Video

1.scorpio跟随 scorpio-Follower

<a href="https://www.youtube.com/embed/UrD2AEQ3VkI" target="_blank"><img src="http://img.youtube.com/vi/UrD2AEQ3VkI/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd ~/scorpio_ros2
source install/setup.bash
ros2 launch scorpio_teleop teleop.launch.py camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6 
```

2.scorpio建图 scorpio-SLAM-Mapping

<a href="https://www.youtube.com/embed/Yt9Sld-EX0s" target="_blank"><img src="http://img.youtube.com/vi/Yt9Sld-EX0s/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd ~/scorpio_ros2
source install/setup.bash
ros2 launch scorpio_slam_transfer start_build_map_gmapping.launch.py camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
```

3.scorpio导航 scorpio-Navigation

<a href="https://www.youtube.com/embed/3RP11sZKfJg" target="_blank"><img src="http://img.youtube.com/vi/3RP11sZKfJg/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd ~/scorpio_ros2
source install/setup.bash
ros2 launch scorpio_navigation2 start_scorpio_navigation2.launch.py camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
```

4.scorpio-RtabMap建图 scorpio-RtabMap-Mapping

<a href="https://www.youtube.com/embed/K5wvlWb-2uQ" target="_blank"><img src="http://img.youtube.com/vi/K5wvlWb-2uQ/0.jpg" 
alt="follow-person" width="240" height="180" border="10" /></a>
```yaml
cd ~/scorpio_ros2
source install/setup.bash
ros2 launch scorpio_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6 localization:='false'
```

