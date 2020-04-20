#include "trans_pointcloud.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "tranc_pointcloud_node");
    ros::NodeHandle node_handle;
    TransPointCloud Trans_P2C;
    // ros::Subscriber sub = node_handle.subscribe(Trans_P2C.pub_topic, 1000, pub_p_callback);
    std::cout<<"Start hearing....."<<std::endl;
    Trans_P2C.Trans_Init();
    ros::spin();
    return 0;
}