/*
 * @Description: 
 * @Author: speedzjy
 * @Date: 2022-04-27 11:49:31
 */

#pragma once

#ifndef LIDAR_CURB_DEC_H
#define LIDAR_CURB_DEC_H

#include "lidar_curb_detection/curb_utils.hpp"
#include "lidar_curb_detection/cloud_mapper.hpp"
#include "lidar_curb_detection/ground_segment.hpp"
#include "lidar_curb_detection/feature_points.hpp"
#include "lidar_curb_detection/boundary_points.hpp"

namespace CurbDectection {

class LidarCurbDectection {

public:
  LidarCurbDectection();
  ~LidarCurbDectection();

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr);

  bool update;
  // 输入的每帧点云
  PointCloudType::Ptr complete_points;
  CloudQueue queue_complete_points;
  // 边界点云
  CloudPtrList boundary_points;

private:
  ros::NodeHandle nh_;
  ros::Subscriber subPointCloud_;

  ros::Publisher pubCompleteCloud_;
  ros::Publisher pubGroundCloud_;
  ros::Publisher pubCurbCloudLeft_;
  ros::Publisher pubCurbCloudRight_;
};

}

#endif