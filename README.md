# 3D Lidar Curb Dectection in ROS

> SOURCE https://github.com/wangguojun2018/LidarRoadBoundaryDetection

This repository changed the code of the above article to ROS implementation, and fixed some bugs.

## Example
![Example](./readme_data/ex.png)

## Dependencies

- glog
- pcl 1.8
- Opencv 3.4
- boost
- [limbo](http://www.resibots.eu/limbo/tutorials/compilation.html)

## Run
```bash
roslaunch lidar_curb_detection lidar_curb_detection.launch
```