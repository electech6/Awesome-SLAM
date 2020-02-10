# Awesome-SLAM
SLAM code, paper, project collections




## ORB-SLAM2 related codes

Raúl Mur-Artal and Juan D. Tardós. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras. IEEE Transactions on Robotics,2017

[PAPER](https://arxiv.org/abs/1610.06475),
[CODE](https://github.com/raulmur/ORB_SLAM2),
[中文注释版](https://github.com/Vincentqyw/ORB-SLAM2-CHINESE)

### 改进方法

[ORBSLAM2_with_pointcloud_map](https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map)

[ORB-SLAM2_RGBD_DENSE_MAP,modified from Xiang Gao's "ORB_SLAM2_modified".It is added a dense loopclosing map model](https://github.com/tiantiandabaojian/ORB-SLAM2_RGBD_DENSE_MAP)

[ORB-YGZ-SLAM, average 3x speed up and keep almost same accuracy v.s. ORB-SLAM2, use direct tracking in SVO to accelerate the feature matching](https://github.com/gaoxiang12/ORB-YGZ-SLAM)

[YGZ-stereo-inertial SLAM, LK optical flow + sliding window bundle adjustment](https://github.com/gaoxiang12/ygz-stereo-inertial)

[VIORB, An implementation of Visual Inertial ORBSLAM based on ORB-SLAM2](https://github.com/jingpang/LearnVIORB)

[Fisheye-ORB-SLAM, A real-time robust monocular visual SLAM system based on ORB-SLAM for fisheye cameras, without rectifying or cropping the input images](https://github.com/lsyads/fisheye-ORB-SLAM)

[Save and load orb-slam2 maps](https://github.com/AlejandroSilvestri/osmap)

[ORB_SLAM2 with map load/save function](https://github.com/Jiankai-Sun/ORB_SLAM2_Enhanced)

[Viewer for maps from ORB-SLAM2 Osmap](https://github.com/AlejandroSilvestri/Osmap-viewer)

[ORB_SLAM2_SSD_Semantic, 动态语义SLAM 目标检测+VSLAM+光流/多视角几何动态物体检测+octomap地图+目标数据库](https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic)

[Add line feature based ORB-SLAM2](https://github.com/atlas-jj/ORB_Line_SLAM)

[RGBD-SLAM with Point and Line Features, developed based on ORB_SLAM2](https://github.com/maxee1900/RGBD-PL-SLAM)

### Different platforms
[Windows version ORBSLAM2,Easy built by visual studio](https://github.com/phdsky/ORBSLAM24Windows)

[ORB-SLAM-Android, test on Sony Xperia Z](https://github.com/castoryan/ORB-SLAM-Android)

[ORBSLAM2 on Mac OSX](https://github.com/meiroo/ORBSLAM2-OSX)

[ROS interface for ORBSLAM2](https://github.com/ethz-asl/orb_slam_2_ros)







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

### PL-VIO
Tightly-Coupled Monocular Visual–Inertial Odometry Using Point and Line Features

[CODE](https://github.com/HeYijia/PL-VIO)

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


## LIDAR-CAMERA SLAM

### LIMO

2018 IROS, LIMO: Lidar-Monocular Visual Odometry

[PAPER](https://arxiv.org/pdf/1807.07524.pdf),
[CODE](https://github.com/johannes-graeter/limo)

## VIO
### OpenVINS, Monolar + IMU
An open source platform for visual-inertial navigation research.

2019 IROS, OpenVINS: A Research Platform for Visual-Inertial Estimation

[PAPER](http://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf),
[CODE](https://github.com/rpng/open_vins),
[PROJECT](https://docs.openvins.com/)




## OpenVSLAM: A Versatile Visual SLAM Framework

1. A monocular, stereo, and RGBD visual SLAM system
2. Created maps can be stored and loaded

[PAPER](https://arxiv.org/abs/1910.01122),
[CODE](https://github.com/xdspacelab/openvslam),
[VIDEO DEMO](https://www.youtube.com/watch?v=Ro_s3Lbx5ms)

## 3D reconstruction

### Fastfusion
2014 ICRA, Volumetric 3D Mapping in Real-Time on a CPU

[PAPER](https://github.com/tum-vision/fastfusion),
[CODE](http://vision.in.tum.de/_media/spezial/bib/steinbruecker_etal_icra2014.pdf),
[Python wrapper](https://github.com/y-j-n/pyFastfusion)

### Intrinsic3D
Obtain high-quality 3D reconstructions from low-cost RGB-D sensors (with poses). The algorithm recovers fine-scale geometric details and sharp surface textures by simultaneously optimizing for reconstructed geometry, surface albedos, camera poses and scene lighting.

2017 ICCV, High-Quality 3D Reconstruction by Joint Appearance and Geometry Optimization with Spatially-Varying Lighting

[PAPER](https://arxiv.org/pdf/1708.01670.pdf),
[CODE](https://github.com/NVlabs/intrinsic3d)

### Open3D
Open3D: A Modern Library for 3D Data Processing, Support C++, Python

[Tutorial:A complete pipeline to reconstruct a 3D scene from an RGBD sequence](http://www.open3d.org/docs/release/tutorial/ReconstructionSystem/index.html)

[CODE](https://github.com/intel-isl/Open3D)



### DenseSurfelMapping


Given a sequence of depth images, intensity images, and camera poses, the proposed methods can fuse them into a globally consistent model using surfel representation. The fusion method supports both ORB-SLAM2 and VINS-Mono.

2019 ICRA, Real-time Scalable Dense Surfel Mapping

[PAPER](https://www.dropbox.com/s/h9bais2wnw1g9f0/root.pdf?dl=0),
[CODE](https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping)


## RGB-D SLAM

### Bundle Adjusted Direct RGB-D SLAM
CVPR 2019, "BAD SLAM: Bundle Adjusted Direct RGB-D SLAM".

[PAPER](http://openaccess.thecvf.com/content_CVPR_2019/html/Schops_BAD_SLAM_Bundle_Adjusted_Direct_RGB-D_SLAM_CVPR_2019_paper.html),
[CODE](https://github.com/ETH3D/badslam)

### StaticFusion
Basis for ElasticFusion.

2018 ICRA, StaticFusion: Background Reconstruction for Dense RGB-D SLAM in Dynamic Environments

[CODE](https://github.com/raluca-scona/staticfusion)

## CAMERA-IMU-LIDAR CALIBRATION

### The Kalibr visual-inertial calibration toolbox

[CODE](https://github.com/ethz-asl/kalibr)





