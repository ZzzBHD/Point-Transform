#pragma once

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include <string.h>
#include <boost/program_options.hpp>
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv_head.h"
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件

class TransPointCloud
{
public:
  TransPointCloud();
  // TransPointCloud(ros::NodeHandle n);
  ~TransPointCloud(){}
  void pubCallback(const sensor_msgs::PointCloud2 & inMsg);
  void tofCallback(const sensor_msgs::ImageConstPtr& msg);
  void Get_Input_Cloud();
  void Write_Ouput_Cloud();
  void Trans_Init();
  void Update_path();
  sensor_msgs::PointCloud2 Point2Sensor();
  std::string input_type_option,pub_topic,tof_topic;
  int start_num,trans_count,current_num;
  std::string cloud_type;

private:
  ros::Subscriber pub_subscriber;
  ros::Subscriber tof_subscriber;
  std::string input_dir, out_dir, output_type_option;
  std::string input_filename,output_filename;
  bool kitti_type;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::PointCloud<pcl::PointXYZI> cloud_xyzi;
  std::stringstream temp;
  void Get_Input_Path();
  void Get_Output_Path();
  void Write_bin();
  void Write_pcd();
  double camera_cx,camera_cy,camera_fx,camera_fy;

};