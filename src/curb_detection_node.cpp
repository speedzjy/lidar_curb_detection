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

// bool update=false;
boost::mutex mtex;

// PointCloudType::Ptr complete_points(new PointCloudType);
// CloudPtrList boundary_points(2);

// void curb_extract(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr) {
//   pcl::fromROSMsg(*in_cloud_ptr, *complete_points);
//   update = true;
// }


int main(int argc,char** argv) {

  ros::init(argc, argv, "curb_detection");

  LidarCurbDectection LCD;

  // ros::Subscriber subPointCloud = nh.subscribe("/velodyne_points",10, curb_extract);

  // for (size_t i = 0; i < boundary_points.size(); ++i)
  // {
  //     boundary_points[i] = boost::make_shared<PointCloudType>();
  // }

  // 可视化

  AINFO << "pcl view" << endl;

  pcl::visualization::PCLVisualizer viewer("curb_viewer");
  int v1(0);
  int v2(0);
  viewer.setBackgroundColor(0,0,0);
  viewer.addCoordinateSystem(3);

  pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud_left(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(255, 0, 0));
  pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud_right(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(0, 0, 255));
  pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(255, 255, 255));

  int i=0;

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
    ros::spinOnce();
    boost::mutex::scoped_lock lock(mtex);

    if (LCD.update)

    {
        viewer.removeAllShapes(); //删掉形状缓存
        viewer.removeAllPointClouds();//删除点云缓存

        handler_cloud->setInputCloud(LCD.complete_points);
        viewer.addPointCloud(LCD.complete_points, *handler_cloud, "cloud", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud",v1);

        handler_cloud_left->setInputCloud(LCD.boundary_points[0]);
        viewer.addPointCloud(LCD.boundary_points[0], *handler_cloud_left, "left",v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "left", v1);


        handler_cloud_right->setInputCloud(LCD.boundary_points[1]);
        viewer.addPointCloud(LCD.boundary_points[1], *handler_cloud_right, "right",v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "right", v1);

        LCD.update=false;
    }
    ++i;
  }

  return 0;
}