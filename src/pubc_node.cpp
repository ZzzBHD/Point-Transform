#include "trans_pointcloud.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "pub_pointcloud_node");
    ros::NodeHandle node_handle("~");
    TransPointCloud Trans_pub;
    std::string cloud_type;
    int loop;
    node_handle.param<std::string>("cloud_type_", cloud_type, "XYZ");
    node_handle.param<int>("loop_", loop, 0);

    ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>(Trans_pub.pub_topic, 1000);
    
    for(int i=0;i<Trans_pub.trans_count;i++){
      Trans_pub.Trans_Init();
      sensor_msgs::PointCloud2 pub_cloud;
      if(!ros::ok()) break;
      if(Trans_pub.input_type_option=="pcd"||Trans_pub.input_type_option=="bin")
      {
        Trans_pub.Get_Input_Cloud();
        ROS_INFO("No.%u.pcd",Trans_pub.current_num);
        pub_cloud=Trans_pub.Point2Sensor();
        std::cout<<"Cloud points number: "<<pub_cloud.width<<std::endl;
        pub_cloud.header.frame_id="pub_cloud";
        cloud_pub.publish(pub_cloud);
        Trans_pub.Update_path();
      }
      if((i==Trans_pub.trans_count) && loop) 
        Trans_pub.current_num=Trans_pub.start_num;
    }
    
    
  return 0;
}