/*
 * @Description:
 * @Author: speedzjy
 * @Date: 2022-04-27 11:55:01
 */

#include "lidar_curb_detection/lidar_curb_dec.hpp"

namespace CurbDectection {

LidarCurbDectection::LidarCurbDectection()
    : complete_points(new PointCloudType), boundary_points(CloudPtrList(2)) {

  subPointCloud_ = nh_.subscribe(
      subpointCloudTopic, 10, &LidarCurbDectection::pointCloudCallback, this);

  pubCompleteCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/complete_cloud", 10);
  pubGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 10);
  pubNoGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/no_ground_cloud", 10);
  pubFeatureCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/feature_cloud", 10);
  pubCurbCloudLeft_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/curb_cloud_left", 10);
  pubCurbCloudRight_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/curb_cloud_right", 10);

  for (size_t i = 0; i < boundary_points.size(); ++i) {
    boundary_points[i] = boost::make_shared<PointCloudType>();
  }
}

LidarCurbDectection::~LidarCurbDectection() {}

void LidarCurbDectection::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr) {

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  PointCloudType::Ptr tmp_points(new PointCloudType);
  pcl::fromROSMsg(*in_cloud_ptr, *tmp_points);
  queue_complete_points.push_back(*tmp_points);

  complete_points->clear();
  for (size_t i = 0; i < boundary_points.size(); ++i) {
    boundary_points[i]->clear();
  }

  // 读取点云
  PointCloudType::Ptr completeCloud(new PointCloudType);
  if (!queue_complete_points.empty()) {
    *completeCloud = queue_complete_points.front();
    queue_complete_points.pop_front();
  }

  //计算点云laserID, 并将ID存为intensity
  CloudMapper mapper_laserID(cmMsg);
  PointCloudType::Ptr completeCloudMapper(new PointCloudType);
  mapper_laserID.processByOri(completeCloud, completeCloudMapper);
  AINFO << "raw points number: " << completeCloudMapper->points.size() << endl;

  // 地面提取
  // 论文中选取范围: Z x Y x X , [−3, 1] x [−40, 40] x [−70, 70]
  // 代码实际选取: Y x X , [−30, 30] x [−40, 40]
  // 使用 pcl 库进行平面特征点提取
  PointCloudType::Ptr ground_points(new PointCloudType);
  PointCloudType::Ptr ground_points_no(new PointCloudType); //非地面点

  GroundSegmentation ground(completeCloudMapper, gsMsg);
  ground.groundfilter(ground_points, ground_points_no);
  AINFO << "ground_points number: " << ground_points->points.size() << endl;
  AINFO << "no_ground_points number: " << ground_points_no->points.size()
        << endl;

  //根据之前计算的Intensity对地面点云进行mapper
  CloudMapper mapper2(cmMsg);
  scanIndices scanIDindices;
  PointCloudType::Ptr ground_points_mapper(new PointCloudType);
  mapper2.processByIntensity(ground_points, ground_points_mapper,
                             scanIDindices);

  //特征点提取
  pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  FeaturePoints curbExtract(ground_points_mapper, scanIDindices, fpMsg);
  curbExtract.extractFeatures(featurePoints);
  AINFO << "feature points number: " << featurePoints->points.size() << endl;

  //高斯过程提取
  BoundaryPoints refinePoints(*featurePoints, cmMsg, bpMsg);
  refinePoints.process(ground_points_no, boundary_points);

  *complete_points = *completeCloud;

  // 计时
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  AINFO << "compute this frame time cost = " << time_used.count()
        << " seconds. " << endl;

  sensor_msgs::PointCloud2 tmp_rosCloud;
  std::string in_cloud_frame_id = in_cloud_ptr->header.frame_id;

  //  完整点云
  pcl::toROSMsg(*complete_points, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubCompleteCloud_.publish(tmp_rosCloud);

  // 地面点
  pcl::toROSMsg(*ground_points, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubGroundCloud_.publish(tmp_rosCloud);

  // 非地面点
  pcl::toROSMsg(*ground_points_no, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubNoGroundCloud_.publish(tmp_rosCloud);

  // 特征点
  pcl::toROSMsg(*featurePoints, tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubFeatureCloud_.publish(tmp_rosCloud);

  // 左边缘点
  pcl::toROSMsg(*(boundary_points[0]), tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubCurbCloudLeft_.publish(tmp_rosCloud);

  // 右边缘点
  pcl::toROSMsg(*(boundary_points[1]), tmp_rosCloud);
  tmp_rosCloud.header.frame_id = in_cloud_frame_id;
  pubCurbCloudRight_.publish(tmp_rosCloud);
}

} // namespace CurbDectection