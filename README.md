# Awesome-SLAM
SLAM code, paper, project collections




# LIDAR SLAM

## 2019

### hdl_graph_slam
3D LIDAR-based Graph SLAM, real-time 6DOF SLAM using a 3D LIDAR
2019, Advanced Robotic Systems, [A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, Advanced Robotic Systems

[PAPER](https://www.researchgate.net/publication/331283709_A_portable_three-dimensional_LIDAR-based_system_for_long-term_and_wide-area_people_behavior_measurement)

### LIO-mapping
ICRA 2019,A Tightly Coupled 3D Lidar and Inertial Odometry and Mapping Approach

[PAPER](https://arxiv.org/abs/1904.06993),
[CODE](https://github.com/hyye/lio-mapping)

### SuMa++
IROS 2019, SuMa++: Efficient LiDAR-based Semantic SLAM

[PAPER](https://www.semanticscholar.org/paper/SuMa-%2B-%2B-%3A-Efficient-LiDAR-based-Semantic-SLAM-Chen-Palazzolo/546137c71dc3c00c8a259de721281930f6574086),
[CODE](https://github.com/PRBonn/semantic_suma)

### A-LOAM
Advanced implementation of LOAM, uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and LOAM_NOTED

[CODE](https://github.com/HKUST-Aerial-Robotics/A-LOAM)


## 2018

### LeGO-LOAM
IROS 2018, LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain

[PAPER](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/Shan_Englot_IROS_2018_Preprint.pdf),
[Video demo](https://www.youtube.com/watch?v=O3tz_ftHV48),
[Gif demo](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/launch/demo.gif)

### loam_velodyne
Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar

[CODE](https://github.com/laboshinl/loam_velodyne)

### lidar_slam_3d
a ROS package for real-time 3D slam. It is based on NDT registration algorithm. With loop detection and back-end optimization, a map with global consistency can be generated.

[CODE](https://github.com/ningwang1028/lidar_slam_3d)



## 2016
Google Cartographer
 Real-time 2D and 3D SLAM across multiple platforms and sensor configurations.
 
 Real-Time Loop Closure in 2D LIDAR SLAM, in Robotics and Automation (ICRA), 2016 IEEE International Conference on. IEEE, 2016

[PAPER](https://research.google/pubs/pub45466/),  [CODE](https://github.com/googlecartographer/cartographer),
[PROJECT](https://google-cartographer.readthedocs.io/en/latest/)

### 2D LaserSLAM
W.Hess, D.Kohler, H.Rapp and D.Andor. Real-Time Loop Closure in 2D LIDAR SLAM. ICRA, 2016

[CODE](https://github.com/meyiao/LaserSLAM),
[VIDEO](https://v.qq.com/x/page/q0363h0i1ej.html)

## 2014
### LOAM
Laser Odometry And Mapping (LOAM) SLAM ROS package for 3D Velodyne VLP-16 laser scanner

J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time. Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014

[PAPER](http://www.roboticsproceedings.org/rss10/p07.pdf),
[VIDEO](https://www.youtube.com/watch?feature=player_embedded&v=8ezyhTAEyHs),
[CODE](https://github.com/daobilige-su/loam_velodyne),
[中文注释代码](https://github.com/cuitaixiang/LOAM_NOTED)


## ORB-SLAM2 related codes

Raúl Mur-Artal and Juan D. Tardós. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics,2017

[PAPER](https://arxiv.org/abs/1610.06475),
[CODE](https://github.com/raulmur/ORB_SLAM2)

### 改进方法
[ORB-YGZ-SLAM, average 3x speed up and keep almost same accuracy v.s. ORB-SLAM2, use direct tracking in SVO to accelerate the feature matching](https://github.com/gaoxiang12/ORB-YGZ-SLAM)

[YGZ-stereo-inertial SLAM, LK optical flow + sliding window bundle adjustment](https://github.com/gaoxiang12/ygz-stereo-inertial)


[ORB-SLAM-Android, test on Sony Xperia Z](https://github.com/castoryan/ORB-SLAM-Android)

[Save and load orb-slam2 maps](https://github.com/AlejandroSilvestri/osmap)



[Viewer for maps from ORB-SLAM2 Osmap](https://github.com/AlejandroSilvestri/Osmap-viewer)



