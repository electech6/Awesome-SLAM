# Awesome-SLAM
SLAM code, paper, project collections



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


[Add line feature based ORB-SLAM2](https://github.com/atlas-jj/ORB_Line_SLAM)

[RGBD-SLAM with Point and Line Features, developed based on ORB_SLAM2](https://github.com/maxee1900/RGBD-PL-SLAM)

## Line feature based SLAM

### PL-SLAM
Stereo visual SLAM using both point and line segment features

2017,PL-SLAM: a Stereo SLAM System through the Combination of Points and Line Segments

[PAPER](https://arxiv.org/abs/1705.09479),
[CODE](https://github.com/rubengooj/pl-slam)

2016 IROS, PL-SVO: Semi-direct monocular visual odometry by combining points and line segments.

[PAPER](http://mapir.isa.uma.es/rgomez/publications/iros16plsvo.pdf)

2016 ICRA, Robust stereo visual odometry through a probabilistic combination of points and line segments. In Robotics and Automation 

[PAPER](http://mapir.isa.uma.es/rgomez/publications/icra16plsvo.pdf)

### Monocular ORB-SLAM with Line Features

[CODE](https://github.com/lanyouzibetty/ORB-SLAM2_with_line)

### ORB_Line_SLAM
Add line feature based ORB-SLAM2

[CODE](https://github.com/atlas-jj/ORB_Line_SLAM)

### RGBD-PL-SLAM
RGBD-SLAM with Point and Line Features, developed based on the famous ORB_SLAM2

[CODE](https://github.com/maxee1900/RGBD-PL-SLAM)

### Line feature based RGBD-SLAM v2

[CODE](https://github.com/yan-lu/LineSLAM)

### 3D Line Semi-dense SLAM
ICPR 2018, Incremental 3D Line Segment Extraction for Surface Reconstruction from Semi-dense SLAM

[PAPER](https://arxiv.org/abs/1708.03275),
[](http://webdocs.cs.ualberta.ca/~vis/thesis_shida/),
[CODE](https://github.com/shidahe/semidense-lines)

### 3D Line-based Stereo SLAM
2015 IEEE Transactions on Robotics, Building a 3-D Line-Based Map Using Stereo SLAM

[CODE](https://github.com/slslam/slslam),
[论文解读](https://www.cnblogs.com/tweed/p/11353285.html)



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

[CODE](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM),
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








