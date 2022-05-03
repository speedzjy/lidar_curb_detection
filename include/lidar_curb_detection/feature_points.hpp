/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-03 11:29:59
 */

#ifndef CURB_POINT_H
#define CURB_POINT_H

#include "lidar_curb_detection/curb_utils.hpp"

namespace CurbDectection {

class FeaturePoints {
public:
  // -sr 2 -hr 5 -st 1 -nrmin 0.25 -nt 0.25
  FeaturePoints(PointCloudType::Ptr incloud, std::vector<IndexRange> scanIndices);
  float RegionMaxZ(int j);
  float RegionMinZ(int j);
  float slope_angle(int i);
  void compute_normal_omp(double searchRadius,
                          pcl::PointCloud<pcl::Normal> &normal);
  std::vector<int> vectors_intersection(std::vector<int> v1, std::vector<int> v2);
  void extractPoints(PointCloudType::Ptr incloud, PointCloudType::Ptr outCloud,
                     boost::shared_ptr<std::vector<int>> indices,
                     bool setNeg = false);
  void extractFeatures(PointCloudType::Ptr feature_points);
  float calcPointDistance(const PointType &p);
  float computeHorizonDiff(int index, int region);
  float computeHeightSigma(int index, int region);
  void normal_diff_filter(PointCloudType::Ptr incloud,
                          pcl::IndicesConstPtr &outIndex);

private:
  PointCloudType::Ptr _cloud;
  scanIndices _scanindices;

  int _HeightRegion;
  float _HeightSigmaThres;
  float _HeightMaxThres;
  float _HeightMinThres;

  float _CurvatureThres;
  int _CurvatureRegion;

  float _DistanceHorizonThres;
  float _DistanceVerticalThres;

  float _AngularRes;

  std::vector<int> _HeightPointsIndex;

  std::vector<int> _CurvaturePointsIndex;

  std::vector<int> _DistanceVerticlePointsIndex;

  std::vector<int> _DistanceHorizonPointsIndex;

  std::vector<int> _Index;

  bool _use_verticle;
  bool _use_horizon;
};

} // namespace CurbDectection

#endif // CURB_POINT_H
