#pragma once

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

class RosBagToPcdNode {
 public:
  RosBagToPcdNode();

  // void InCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& inMsg);
  void InCloudCallback(const sensor_msgs::PointCloud2 & inMsg);

 private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_in_cloud_;

  // topic name
  std::string in_cloud_topic_;

  // output file dir
  std::string out_pcd_dir_;

  int receive_file_count_;
  int file_count_;
};