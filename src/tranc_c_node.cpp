#include "trans_pointcloud.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "tranc_pointcloud_node");
    ros::NodeHandle node_handle;
    TransPointCloud Trans_C;
    // std::cout<<Trans_C.current_num<<" "<<Trans_C.trans_count<<std::endl;
    if(Trans_C.input_type_option=="pcd"||Trans_C.input_type_option=="bin")
    {
      for(int i=0;i<Trans_C.trans_count;i++){
        if(!ros::ok()) break;
        Trans_C.Trans_Init();
        Trans_C.Get_Input_Cloud();
        ROS_INFO("No.%u.pcd",Trans_C.current_num);
        Trans_C.Write_Ouput_Cloud();
      }
    }
    else if(Trans_C.input_type_option=="tof"||Trans_C.input_type_option=="pub")
    {
      ros::spin();
    }
  return 0;
}