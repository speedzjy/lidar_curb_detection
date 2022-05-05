/*
 * @Description: 
 * @Author: speedzjy
 * @Date: 2022-04-26 16:00:30
 */

#include "lidar_curb_detection/curb_utils.hpp"
#include "lidar_curb_detection/lidar_curb_dec.hpp"
#include "lidar_curb_detection/cloud_mapper.hpp"
#include "lidar_curb_detection/ground_segment.hpp"

using namespace std;
using namespace Eigen;
using namespace CurbDectection;

int main(int argc,char** argv) {

  ros::init(argc, argv, "curb_detection");

  LidarCurbDectection LCD;

  ros::spin();

  return 0;
}