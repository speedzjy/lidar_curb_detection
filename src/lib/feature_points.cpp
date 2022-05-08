/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-08 09:27:27
 */

#include "lidar_curb_detection/feature_points.hpp"

namespace CurbDectection {

FeaturePoints::FeaturePoints(PointCloudType::Ptr incloud,
                             std::vector<IndexRange> scanIndices,
                             const featurePointsMsg &fpMsg)

{
  _cloud.reset(new PointCloudType);
  _cloud = incloud;
  _scanindices = scanIndices;

  _HeightRegion = fpMsg.heightRegion;
  _HeightSigmaThres = fpMsg.heightSigmaThre;
  _HeightMaxThres = fpMsg.heightMaxThres;
  _HeightMinThres = fpMsg.heightMinThres;

  _CurvatureThres = fpMsg.curvatureThres;
  _CurvatureRegion = fpMsg.curvatureRegion;

  _DistanceHorizonThres = fpMsg.distanceHorizonThres;
  _DistanceVerticalThres = fpMsg.distanceVerticalThres;

  _AngularRes = fpMsg.angularRes;

  _use_verticle = fpMsg.useVerticle;
  _use_horizon = fpMsg.useHorizon;
}

float FeaturePoints::computeHorizonDiff(int index, int region) {
  float ori_1 = std::atan2(_cloud->points[index - region].y,
                           _cloud->points[index - region].x) *
                180 / M_PI;
  float ori_2 = std::atan2(_cloud->points[index + region].y,
                           _cloud->points[index + region].x) *
                180 / M_PI;

  ori_1 = ori_1 < 0 ? ori_1 + 360 : ori_1;
  ori_2 = ori_2 < 0 ? ori_2 + 360 : ori_2;
  return abs(ori_1 - ori_2);
}

float FeaturePoints::computeHeightSigma(int index, int region) {
  //高度方差点
  float meanZ = _cloud->points[index].z;

  for (int j = 1; j <= region; j++) {
    meanZ += _cloud->points[index + j].z + _cloud->points[index - j].z;
  }

  meanZ = meanZ / (1 + 2 * region);

  float sigma_height = pow(_cloud->points[index].z - meanZ, 2);

  for (int j = 1; j <= region; j++) {
    sigma_height += pow(_cloud->points[index + j].z - meanZ, 2);
    sigma_height += pow(_cloud->points[index - j].z - meanZ, 2);
  }

  sigma_height = sqrt(sigma_height / (1 + 2 * region));

  return sigma_height;
}

float FeaturePoints::RegionMaxZ(int j) {
  float max_z = _cloud->points[j].z;

  for (int k = j - _HeightRegion; k <= j + _HeightRegion; ++k) {
    if (max_z < _cloud->points[k].z) {
      max_z = _cloud->points[k].z;
    }
  }

  return max_z;
}

float FeaturePoints::RegionMinZ(int j) {
  float min_z = _cloud->points[j].z;

  for (int k = j - _HeightRegion; k <= j + _HeightRegion; ++k) {
    if (min_z > _cloud->points[k].z) {
      min_z = _cloud->points[k].z;
    }
  }
  return min_z;
}

float FeaturePoints::calcPointDistance(const PointType &p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

std::vector<int> FeaturePoints::vectors_intersection(std::vector<int> v1,
                                                     std::vector<int> v2) {
  std::vector<int> v;
  sort(v1.begin(), v1.end());
  sort(v2.begin(), v2.end());
  set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                   back_inserter(v));

  return v;
}

void FeaturePoints::extractPoints(PointCloudType::Ptr incloud,
                                  PointCloudType::Ptr outCloud,
                                  boost::shared_ptr<std::vector<int>> indices,
                                  bool setNeg) {
  pcl::ExtractIndices<PointType> extract;
  extract.setNegative(setNeg);
  extract.setInputCloud(incloud);
  extract.setIndices(indices);
  extract.filter(*outCloud);
}

void FeaturePoints::extractFeatures(PointCloudType::Ptr feature_points) {
  // 之前分片的每段scan
  size_t nScans = _scanindices.size();

  float angleRegionThres = _AngularRes * 4;

  for (size_t i = 0; i < nScans; i++) {
    size_t scanStartIdx = _scanindices[i].first;
    size_t scanEndIdx = _scanindices[i].second;

    // 跳过点数小于100的scan
    if (scanEndIdx <= scanStartIdx + 100) {
      continue;
    }

    //提取高度特征点
    for (int k = scanStartIdx + _HeightRegion; k <= scanEndIdx - _HeightRegion;
         ++k) {
      if (computeHorizonDiff(k, _HeightRegion) >
          (_HeightRegion * 2) * angleRegionThres) {
        continue;
      }
      //绝对高度点

      // float angleVertical = std::atan(_cloud->points[k].z /
      // sqrt(pow(_cloud->points[k].x,2) +
      //                                                       pow(_cloud->points[k].y,2)));

      // float
      // Threshold=fabs(sin(angleVertical))*sqrt(pow(_cloud->points[k].x,2) +
      //                                                             pow(_cloud->points[k].y,2))*M_PI*0.18/180;

      float heightDiff = RegionMaxZ(k) - RegionMinZ(k);

      //高度方差点

      float sigma_height = computeHeightSigma(k, _HeightRegion);

      //方差和绝对高度都满足条件视为候选curb点
      if (heightDiff >= _HeightMinThres && heightDiff <= _HeightMaxThres &&
          sigma_height > _HeightSigmaThres) {
        _HeightPointsIndex.push_back(k);
      }
    }

    // 提取平滑特征点
    float pointWeight = -2 * _CurvatureRegion;

    for (size_t k = scanStartIdx + _CurvatureRegion;
         k <= scanEndIdx - _CurvatureRegion; k++) {
      if (computeHorizonDiff(k, _CurvatureRegion) >
          (_CurvatureRegion * 2) * angleRegionThres) {
        continue;
      }
      float diffX = pointWeight * _cloud->points[k].x;
      float diffY = pointWeight * _cloud->points[k].y;
      float diffZ = pointWeight * _cloud->points[k].z;

      for (int j = 1; j <= _CurvatureRegion; j++) {
        diffX += _cloud->points[k + j].x + _cloud->points[k - j].x;
        diffY += _cloud->points[k + j].y + _cloud->points[k - j].y;
        diffZ += _cloud->points[k + j].z + _cloud->points[k - j].z;
      }

      float curvatureValue =
          sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) /
          (_CurvatureRegion * calcPointDistance(_cloud->points[k]));
      if (curvatureValue > _CurvatureThres) {
        _CurvaturePointsIndex.push_back(k);
      }
    }

    //平面距离特征点
    for (int k = scanStartIdx + 1; k <= scanEndIdx; ++k) {
      if (computeHorizonDiff(k, 1) > (2 * angleRegionThres)) {
        continue;
      }

      float distanceHorizonThreshold =
          sqrt(pow(_cloud->points[k].x, 2) + pow(_cloud->points[k].y, 2)) *
          M_PI * _AngularRes / 180;

      float distancePre =
          sqrt(pow(_cloud->points[k].x - _cloud->points[k - 1].x, 2) +
               pow(_cloud->points[k].y - _cloud->points[k - 1].y, 2));
      if (distancePre < distanceHorizonThreshold * _DistanceHorizonThres) {
        _DistanceHorizonPointsIndex.push_back(k);
      }
    }

    //垂直距离特征
    for (int k = scanStartIdx + 1; k <= scanEndIdx; ++k) {
      if (computeHorizonDiff(k, 1) > (1 * 2) * angleRegionThres) {
        continue;
      }

      float angleVertical =
          std::atan(_cloud->points[k].z / sqrt(pow(_cloud->points[k].x, 2) +
                                               pow(_cloud->points[k].y, 2)));

      float distanceHorizonThreshold =
          fabs(sin(angleVertical)) *
          sqrt(pow(_cloud->points[k].x, 2) + pow(_cloud->points[k].y, 2)) *
          M_PI * _AngularRes / 180;

      float distancePre = abs(_cloud->points[k].z - _cloud->points[k - 1].z);
      if (distancePre > distanceHorizonThreshold * _DistanceVerticalThres) {
        _DistanceVerticlePointsIndex.push_back(k);
      }
    }
  }

  AINFO << " height index number is " << _HeightPointsIndex.size() << endl;
  AINFO << " curvature index number is " << _CurvaturePointsIndex.size()
        << endl;
  AINFO << " distance horizontal index number is "
        << _DistanceHorizonPointsIndex.size() << endl;
  AINFO << " distance vertical index number is "
        << _DistanceVerticlePointsIndex.size() << endl;

  std::vector<int> Index_temp;

  _Index = vectors_intersection(_HeightPointsIndex, _CurvaturePointsIndex);

  if (_use_horizon) {
    _Index = vectors_intersection(_Index, _DistanceHorizonPointsIndex);
  }

  if (_use_verticle) {
    _Index = vectors_intersection(_Index, _DistanceVerticlePointsIndex);
  }

  extractPoints(_cloud, feature_points,
                boost::make_shared<std::vector<int>>(_Index));
}

} // namespace CurbDectection
