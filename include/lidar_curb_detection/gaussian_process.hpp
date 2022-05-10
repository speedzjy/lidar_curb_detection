/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-05-10 15:31:15
 */
#ifndef GAUSSIAN_PROCESS_H
#define GAUSSIAN_PROCESS_H

#include "lidar_curb_detection/curb_utils.hpp"

using namespace limbo;

namespace CurbDectection {

class GaussianProcess {
public:
  GaussianProcess(PointCloudType::Ptr candidatePoints,
                  PointCloudType::Ptr leftInitialPoints, float varThres,
                  float meanThres);
  std::vector<int> vectors_difference(std::vector<int> v1, std::vector<int> v2);
  void process(PointCloudType::Ptr clusterCloud);
  void initialTrainData();

private:
  PointCloudType _candidatePoints;
  PointCloudType _leftInitialPoints;
  //    pcl::PointCloud<pcl::PointXYZI> _rightInitialPoints;
  std::vector<Eigen::VectorXd> _xLeft; //存储内点数据
  std::vector<Eigen::VectorXd> _yLeft;

  std::vector<Eigen::VectorXd> _remainX; //存储未处理点数据
  std::vector<Eigen::VectorXd> _remainY;
  std::vector<int> _leftIndex;
  //    vector<int> _rightIndex;
  std::vector<int> _remainIndex;
  std::vector<int> _candidateIndex;

  float _meanThres;
  float _varThres;

  struct Params {
    struct kernel_exp {
      BO_PARAM(float, sigma_sq, 38.1416);
      BO_PARAM(float, l, 16.1003);
      //            BO_PARAM(double, noise,0.0039);
    };
    struct kernel : public defaults::kernel {};
    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {};
    struct opt_rprop : public defaults::opt_rprop {};
  };
};
} // namespace CurbDectection

#endif // GAUSSIAN_PROCESS_H
