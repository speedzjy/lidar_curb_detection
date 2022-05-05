/*
 * @Description:
 * @Author: speedzjy
 * @Date: 2022-04-27 11:55:01
 */

#include "lidar_curb_detection/lidar_curb_dec.hpp"

namespace CurbDectection {

LidarCurbDectection::LidarCurbDectection(std::string cloud_topic_name)
    : nh_("~"), complete_points(new PointCloudType),
      boundary_points(CloudPtrList(2)) {

  update = false;
  subPointCloud_ = nh_.subscribe(
      cloud_topic_name, 10, &LidarCurbDectection::pointCloudCallback, this);

  pubCompleteCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/complete_cloud", 10);
  pubGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 10);
  pubNoGroundCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/no_ground_cloud", 10);
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

  PointCloudType::Ptr tmp_points(new PointCloudType);
  pcl::fromROSMsg(*in_cloud_ptr, *tmp_points);
  queue_complete_points.push_back(*tmp_points);

  // update = true;
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

  //计算点云laserID,并将ID存为intensity
  CloudMapper mapper_laserID;
  PointCloudType::Ptr completeCloudMapper(new PointCloudType);
  mapper_laserID.processByOri(completeCloud, completeCloudMapper);
  AINFO << "raw points number is " << completeCloudMapper->points.size()
        << endl;

  //地面提取
  PointCloudType::Ptr ground_points(new PointCloudType);
  PointCloudType::Ptr ground_points_no(new PointCloudType); //非地面点

  GroundSegmentation ground(completeCloudMapper);

  ground.groundfilter(ground_points, ground_points_no);
  AINFO << "after ground filter" << endl;

  //根据之前计算的Intensity对地面点云进行mapper
  CloudMapper mapper2;
  scanIndices scanIDindices;
  PointCloudType::Ptr ground_points_mapper(new PointCloudType);
  mapper2.processByIntensity(ground_points, ground_points_mapper,
                             scanIDindices);
  AINFO << "ground points mapper is " << ground_points_mapper->points.size()
        << endl;

  //特征点提取
  pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  FeaturePoints curbExtract(ground_points_mapper, scanIDindices);
  curbExtract.extractFeatures(featurePoints);
  AINFO << "feature points is " << featurePoints->points.size() << endl;

  //高斯过程提取
  BoundaryPoints refinePoints(*featurePoints);
  refinePoints.process(ground_points_no, boundary_points);

  *complete_points = *completeCloud;

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