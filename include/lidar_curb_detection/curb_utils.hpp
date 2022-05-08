/*
 * @Description: None
 * @Author: speedzjy
 * @Date: 2022-04-26 17:00:20
 */

#pragma once

#ifndef CURB_UTILS_H
#define CURB_UTILS_H

#include <utility>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/don.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/serialize/text_archive.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>

#include "log.h"
#include "math_utils.h"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef std::vector<PointCloudType::Ptr> CloudPtrList;
typedef std::vector<PointCloudType> CloudList;
typedef std::deque<PointCloudType> CloudQueue;

typedef std::pair<size_t, size_t> IndexRange;
typedef std::vector<IndexRange> scanIndices;

typedef pcl::PointCloud<pcl::PointXY> PointCloudXY;

typedef struct cloudmappermsg {
  float lowerBound;
  float upperBound;
  int nScanRings;
} cloudMapperMsg;

typedef struct groundsegmentationmsg {
  float segThres;
} groundSegmentationMsg;

typedef struct featurepointsmsg {
  float heightMaxThres;
  float heightMinThres;
  int heightRegion;
  float heightSigmaThre;

  int curvatureRegion;
  float curvatureThres;

  float distanceHorizonThres;
  float distanceVerticalThres;

  float angularRes;

  bool useVerticle;
  bool useHorizon;
} featurePointsMsg;

typedef struct boundarypointsmsg {
  float varThres;
  float meanThres;
  int gridNum;
  float gridRes;
  float curveFitThres;
  bool useCurveRansac;
} boundaryPointsMsg;

namespace CurbDectection {

class ParamServer {
public:
  ros::NodeHandle nh_;

  std::string subpointCloudTopic;
  cloudMapperMsg cmMsg;
  groundSegmentationMsg gsMsg;
  featurePointsMsg fpMsg;
  boundaryPointsMsg bpMsg;

  ParamServer() {

    nh_.param<std::string>("lidar_curb_detection/pointCloudTopic",
                           subpointCloudTopic, "velodyne_points");

    // cloudMapperMsg
    nh_.param<float>("lidar_curb_detection/cloudMapperMsg/lowerBound",
                     cmMsg.lowerBound, -15);
    nh_.param<float>("lidar_curb_detection/cloudMapperMsg/upperBound",
                     cmMsg.upperBound, 15);
    nh_.param<int>("lidar_curb_detection/cloudMapperMsg/nScanRings",
                   cmMsg.nScanRings, 32);

    // groundSegmentationMsg
    nh_.param<float>("lidar_curb_detection/groundSegmentationMsg/segThres",
                     gsMsg.segThres, 0.5);

    // featurePointsMsg
    nh_.param<float>("lidar_curb_detection/featurePointsMsg/heightMaxThres",
                     fpMsg.heightMaxThres, 0.5);
    nh_.param<float>("lidar_curb_detection/featurePointsMsg/heightMinThres",
                     fpMsg.heightMinThres, 0.02);
    nh_.param<int>("lidar_curb_detection/featurePointsMsg/heightRegion",
                   fpMsg.heightRegion, 5);
    nh_.param<float>("lidar_curb_detection/featurePointsMsg/heightSigmaThre",
                     fpMsg.heightSigmaThre, 0.01);
    nh_.param<int>("lidar_curb_detection/featurePointsMsg/curvatureRegion",
                   fpMsg.curvatureRegion, 5);
    nh_.param<float>("lidar_curb_detection/featurePointsMsg/curvatureThres",
                     fpMsg.curvatureThres, 0.001);
    nh_.param<float>(
        "lidar_curb_detection/featurePointsMsg/distanceHorizonThres",
        fpMsg.distanceHorizonThres, 1);
    nh_.param<float>(
        "lidar_curb_detection/featurePointsMsg/distanceVerticalThres",
        fpMsg.distanceVerticalThres, 1);
    nh_.param<float>("lidar_curb_detection/featurePointsMsg/angularRes",
                     fpMsg.angularRes, 0.16);
    nh_.param<bool>("lidar_curb_detection/featurePointsMsg/useVerticle",
                    fpMsg.useVerticle, false);
    nh_.param<bool>("lidar_curb_detection/featurePointsMsg/useHorizon",
                    fpMsg.useHorizon, true);

    // boundaryPointsMsg
    nh_.param<float>("lidar_curb_detection/boundaryPointsMsg/varThres",
                     bpMsg.varThres, 2.5);
    nh_.param<float>("lidar_curb_detection/boundaryPointsMsg/meanThres",
                     bpMsg.meanThres, 1.5);
    nh_.param<int>("lidar_curb_detection/boundaryPointsMsg/gridNum",
                   bpMsg.gridNum, 200);
    nh_.param<float>("lidar_curb_detection/boundaryPointsMsg/gridRes",
                     bpMsg.gridRes, 0.5);
    nh_.param<float>("lidar_curb_detection/boundaryPointsMsg/curveFitThres",
                     bpMsg.curveFitThres, 0.15);
    nh_.param<bool>("lidar_curb_detection/boundaryPointsMsg/useCurveRansac",
                    bpMsg.useCurveRansac, true);

    usleep(100);
  }

  ~ParamServer() {}
};

} // namespace CurbDectection

#endif