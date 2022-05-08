/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-08 09:18:13
 */

#pragma once

#ifndef GROUND_SEGMENT_H
#define GROUND_SEGMENT_H

#include "lidar_curb_detection/curb_utils.hpp"

namespace CurbDectection {
  
class GroundSegmentation {
public:
  GroundSegmentation(PointCloudType::Ptr incloud, const groundSegmentationMsg &gsMsg);
  void extractGround(PointCloudType::Ptr outCloud,
                     PointCloudType::Ptr inputCloud,
                     pcl::PointIndices::Ptr Indices, bool setNeg = false);
  void planeSeg(PointCloudType::Ptr cloud,
                pcl::ModelCoefficients::Ptr coefficients,
                pcl::PointIndices::Ptr planeIndices);
  void groundfilter(PointCloudType::Ptr groundpoints,
                    PointCloudType::Ptr non_groundpoints);
  void process(PointCloudType::Ptr outcloud);

private:
  float _threshold;
  CloudPtrList _cloudptrlist;
};
} // namespace CurbDectection

#endif // GROUND_SEGMENT_H
