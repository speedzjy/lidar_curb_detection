/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-03 15:16:28
 */

#pragma once

#ifndef BOUNDARY_POINTS_H
#define BOUNDARY_POINTS_H

#include "lidar_curb_detection/curb_utils.hpp"

#include "lidar_curb_detection/cloud_mapper.hpp"
#include "lidar_curb_detection/ground_segment.hpp"

#include <lib/grid_map.h>
#include "gaussian_process.h"
#include <lib/road_segmentation.h>


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace RoadBoundary;


class BoundaryPoints
{
public:
    BoundaryPoints(PointCloudType &incloud,BoundaryPointsMsg msg);
    void extractPointCloud(PointCloudType& incloud, pcl::PointIndicesPtr indices, PointCloudType& outcloud);
    void lineFitRansac(PointCloudType& incloud, pcl::PointIndices& indices);
    void statisticalFilter_indces(PointCloudType incloud, int meanK, pcl::PointIndicesPtr pointindices, double stdThreshold);
    void pointcloud_projection(PointCloudType::Ptr incloud,PointCloudType& outcloud);
    void ransac_curve(PointCloudType::Ptr incloud,float residualThreshold,
                                          PointCloudType::Ptr outcloud);
    void generateGrid(PointCloudType::Ptr incloud,vector<PointCloudType>& outcloud);
   void distanceFilterByGrid(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud, bool left);
   void distanceFilterByLaserLeft(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud);
   void distanceFilterByLaserRight(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud);
    void process(PointCloudType::Ptr obstacleCloud, CloudPtrList clusterCloud);
private:
    PointCloudType::Ptr _cloud;
    pcl::PointIndices _indicesLeft;
    pcl::PointIndices _indicesRight;

    // double _dbR;//密度直达点的搜索半径
    // double _neighborRate;
    // int _min_pets;//邻域内最少点个数
    float _varThres;
    float _meanThres;
    float _gridRes;
    int _gridNum;
    float _curveFitThres;
    bool _use_curve_fit;

};

#endif // BOUNDARY_POINTS_H
