/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-10 17:03:38
 */
#ifndef ROAD_SEGMENTATION_H
#define ROAD_SEGMENTATION_H

#include "lidar_curb_detection/curb_utils.hpp"

namespace CurbDectection {

class RoadSegmentation {
public:
  RoadSegmentation(PointCloudType::Ptr incloud);
  void generatePolarGrid();
  void computeDistanceVec();
  void computeSegmentAngle();
  void process(PointCloudType::Ptr incloud, CloudPtrList outcloud,
               MarkerList &road_central_line);

  std::vector<PointType> _nearest_points;
  std::vector<float> _distance_vec_filtered;
  std::vector<float> _distance_vec_;

private:
  PointCloudType::Ptr _completeCloud;

  std::vector<std::vector<PointType>> _grid_map_vec;

  std::vector<std::pair<float, int>> _distance_vec;
  std::vector<std::pair<float, int>> _distance_vec_front;
  std::vector<std::pair<float, int>> _distance_vec_rear;

  std::vector<int> _segmentAngle;

  MarkerList road_central_line_;
};

} // namespace CurbDectection

#endif // ROAD_SEGMENTATION_H
