/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: speedzjy
 * @LastEditTime: 2022-05-05 15:43:40
 */

#pragma once

#ifndef CLOUD_MAPPER_H
#define CLOUD_MAPPER_H

#include "lidar_curb_detection/curb_utils.hpp"

namespace CurbDectection {
class CloudMapper {
public:
  CloudMapper();
  ~CloudMapper();

  const float &getLowerBound() { return _lowerBound; }
  const float &getUpperBound() { return _upperBound; }
  const int &getNumberOfScanRings() { return _nScanRings; }
  int getRingForAngle(const float &angle);
  void processByIntensity(PointCloudType::Ptr incloud,
                          PointCloudType::Ptr outcloud,
                          scanIndices &scanindices);
  void processByOri(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud);
  void processByVer(PointCloudType::Ptr incloud, PointCloudType::Ptr outcloud,
                    scanIndices &scanindices);
  float _lowerBound; ///< the vertical angle of the first scan ring
  float _upperBound; ///< the vertical angle of the last scan ring
  int _nScanRings;   ///< number of scan rings
  float _factor;     ///< linear interpolation factor
};

} // namespace CurbDectection

#endif // CLOUD_MAPPER_H
