#include "bag2pcd_node.h"

RosBagToPcdNode::RosBagToPcdNode() : nh_("~") {
  nh_.param<std::string>("in_cloud_topic", in_cloud_topic_, "in_cloud");
  nh_.param<std::string>("out_pcd_dir", out_pcd_dir_, "~/pcd/");

  sub_in_cloud_ = nh_.subscribe(in_cloud_topic_, 2,
                                &RosBagToPcdNode::InCloudCallback, this);

  file_count_ = 0;
  receive_file_count_ = 0;
}

// void RosBagToPcdNode::InCloudCallback(
//     const sensor_msgs::PointCloud2::ConstPtr& inMsg) 
  void RosBagToPcdNode::InCloudCallback(
    const sensor_msgs::PointCloud2 & inMsg){
  ROS_INFO("[RosBagToPcdNode::InCloudCallback]: have received: %d",
           receive_file_count_ + 1);


  if (receive_file_count_ % 1 == 0) {
    double time_start = ros::Time::now().toSec();

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
    //     new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI> cloud;

    int pointBytes = inMsg.point_step;
    int offset_x;
    int offset_y;
    int offset_z;
    int offset_int;
    for (int f=0; f<inMsg.fields.size(); ++f)
    {
      if (inMsg.fields[f].name == "x"){
        // std::cout<<"x"<<std::endl;
        offset_x = inMsg.fields[f].offset;}
      if (inMsg.fields[f].name == "y")
        offset_y = inMsg.fields[f].offset;
      if (inMsg.fields[f].name == "z")
        offset_z = inMsg.fields[f].offset;
      if (inMsg.fields[f].name == "intensity")
        {offset_int = inMsg.fields[f].offset;
        // std::cout<<"i:"<<offset_int<<std::endl;
        }
      // kitti: name = i
      // ours: name = intensity
      // std::cout<<inMsg.fields[f].name<<std::endl;
    }

    // populate point cloud object
    for (int p=0; p<inMsg.width; ++p)
    {
        pcl::PointXYZI newPoint;

        newPoint.x = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_x);
        newPoint.y = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_y);
        newPoint.z = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_z);
        newPoint.intensity = *(uint16_t*)(&inMsg.data[0] + (pointBytes*p) + offset_int);

        cloud.points.push_back(newPoint);
    }


    // pcl::fromROSMsg(*inMsg, *cloud_ptr);

    char ss[10];
    sprintf(ss, "/%u", file_count_+1);
    std::string file_path =
        out_pcd_dir_ + std::string(ss) + std::string(".pcd");

    cloud.width = 1;
    cloud.height = cloud.points.size();
    pcl::io::savePCDFile(file_path, cloud);

    // pcl::io::savePCDFileASCII(file_path, *cloud_ptr);
    ROS_INFO("[RosBagToPcdNode::InCloudCallback]: file_path: %s",
             file_path.c_str());

    file_count_++;
    cloud.points.clear();
    ROS_INFO("[RosBagToPcdNode::InCloudCallback]: save_file last: %f",
             ros::Time::now().toSec() - time_start);
  }
  receive_file_count_++;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pub_pointcloud_node");
  RosBagToPcdNode rosbag_to_pcd_node;
  ros::spin();

  return 0;
}
