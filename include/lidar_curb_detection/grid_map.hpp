/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-03 15:20:50
 */

#pragma once

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "lidar_curb_detection/curb_utils.hpp"

namespace CurbDectection {

class point2D {
public:
  float r;
  float z;

  point2D() {}

  point2D(PointType point) {
    r = sqrt(pow(point.x, 2) + pow(point.y, 2));
    z = point.z;
  }
};

class GridMap {
public:
  GridMap(PointCloudType::Ptr incloud, float gridRes, int gridNum);
  void generateCartesianGrid(std::vector<std::vector<PointType>> &grid_map_vec_carte);

  void distanceFilterByCartesianGrid(PointCloudType::Ptr outcloud, bool left);

private:
  PointCloudType::Ptr _origin_cloud_ptr;

  int _grid_num;
  float _grid_res;
};
} // namespace CurbDectection

#endif // GRID_MAP_H
