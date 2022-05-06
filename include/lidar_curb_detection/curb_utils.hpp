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
#include <cmath>
#include <deque>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

 #include <opencv2/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

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
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/serialize/text_archive.hpp>

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


static float cloudMapperMsg_lowerBound = -15;
static float cloudMapperMsg_upperBound = 15;
static int cloudMapperMsg_nScanRings = 32;

static float groundSegmentationMsg_segThres = 0.4;

static float featurePointsMsg_heightMaxThres = 0.4;
static float featurePointsMsg_heightMinThres = 0.02;
static int featurePointsMsg_heightRegion = 5;
static float featurePointsMsg_heightSigmaThre = 0.01;
static int featurePointsMsg_curvatureRegion = 5;
static float featurePointsMsg_curvatureThres = 0.001;
static float featurePointsMsg_distanceHorizonThres = 1;
static float featurePointsMsg_distanceVerticalThres = 1;
static float featurePointsMsg_angularRes = 0.2;  // 激光雷达角度分辨率
static bool featurePointsMsg_useVerticle = false;
static bool featurePointsMsg_useHorizon = true;

static float boundaryPointsMsg_varThres = 2.5;
static float boundaryPointsMsg_meanThres = 1.5;
static int boundaryPointsMsg_gridNum = 200;
static float boundaryPointsMsg_gridRes = 0.5;
static float boundaryPointsMsg_curveFitThres = 0.15;
static bool boundaryPointsMsg_useCurveRansac= true;

#endif