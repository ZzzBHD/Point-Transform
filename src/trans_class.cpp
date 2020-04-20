#include "trans_pointcloud.h"


TransPointCloud::TransPointCloud()
{
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    pn.param<std::string>("cloud_type_", cloud_type, "XYZ");
    pn.param<std::string>("input_dir_", input_dir, "input_dir");
    pn.param<std::string>("out_dir_", out_dir, "out_dir");
    pn.param<int>("trans_count_", trans_count, 10);
    pn.param<int>("start_num_", start_num, 11);
    pn.param<std::string>("input_type_option_", input_type_option, "pcd");
    pn.param<std::string>("output_type_option_", output_type_option, "bin");
    pn.param<std::string>("tof_topic_", tof_topic, "tof");
    pn.param<std::string>("pub_topic_", pub_topic, "pub");
    pn.param<bool>("kitti_type_", kitti_type, false);
    pn.param<double>("camera_cx", camera_cx, 487.6446798734261);
    pn.param<double>("camera_cy", camera_cy, 279.5953266225303);
    pn.param<double>("camera_cx", camera_fx, 535.9643029929152);
    pn.param<double>("camera_fy", camera_fy, 537.1964560834192);

    pub_subscriber = pn.subscribe(pub_topic, 5000, &TransPointCloud::pubCallback, this);
    tof_subscriber = pn.subscribe(tof_topic, 5000, &TransPointCloud::tofCallback, this);

    current_num=start_num;
    cloud_xyz.points.clear();
    cloud_xyzi.points.clear();
}

void TransPointCloud::pubCallback(const sensor_msgs::PointCloud2 & inMsg)
{
    cloud_xyz.points.clear();
    cloud_xyzi.points.clear();
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
        pcl::PointXYZ Pointxyz;
        pcl::PointXYZI Pointxyzi;
        if(cloud_type=="XYZ"){
        Pointxyz.x = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_x);
        Pointxyz.y = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_y);
        Pointxyz.z = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_z);
        cloud_xyz.points.push_back(Pointxyz);}
        else if(cloud_type=="XYZI"){
        Pointxyzi.x = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_x);
        Pointxyzi.y = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_y);
        Pointxyzi.z = *(float*)(&inMsg.data[0] + (pointBytes*p) + offset_z);
        Pointxyzi.intensity = *(uint16_t*)(&inMsg.data[0] + (pointBytes*p) + offset_int);
        cloud_xyzi.points.push_back(Pointxyzi);}
    }

    Get_Output_Path();

    if(cloud_type=="XYZ" && output_type_option=="pcd"){
        cloud_xyz.width = 1;
        cloud_xyz.height = cloud_xyz.points.size();
        pcl::io::savePCDFile(output_filename, cloud_xyz);
        ROS_INFO("No.%u.pcd",current_num);}
    else if(cloud_type=="XYZI"&& output_type_option=="pcd"){
        cloud_xyzi.width = 1;
        cloud_xyzi.height = cloud_xyzi.points.size();
        pcl::io::savePCDFile(output_filename, cloud_xyzi);
        ROS_INFO("No.%u.pcd",current_num);}
    else if(output_type_option=="bin"){
        Write_bin();
        ROS_INFO("No.%u.bin",current_num);}

    Update_path();
}

void TransPointCloud::tofCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image;
    pcl::PointXYZ P;
    float d;
    cloud_xyz.points.clear();
    try
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_depth (new pcl::PointCloud<pcl::PointXYZ>);
        cv_bridge::CvImagePtr cv_ptr;
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        Get_Output_Path();
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr->image.copyTo(image);
        for(int r = 0; r < image.rows; ++r)
        {
            const uint16_t *itI = image.ptr<uint16_t>(r);
            for(int c = 0; c < image.cols; ++c, ++itI)
            {
                d = *itI/1000.0f;
                if(d!=0)
                {
                    P.z=d;
                    P.x=(c-camera_cx)*P.z/camera_fx;
                    P.y=(r-camera_cy)*P.z/camera_fy;
                    cloud_depth->points.push_back(P);
                }
            }
        }
        cloud_depth->height=1;
        cloud_depth->width=cloud_depth->points.size();
        cloud_depth->is_dense=false;
        if(cloud_depth->points.size()!=0)
        {
            voxelgrid.setInputCloud(cloud_depth);//输入点云数据
            voxelgrid.setLeafSize(0.03f, 0.03f, 0.03f);//AABB长宽高,设置体素栅格在XYZ3个方向上的尺寸
            voxelgrid.filter(cloud_xyz);
            // pcl::io::savePCDFile(output_filename,cloud_xyz);
        }
        if(output_type_option=="pcd") {Write_pcd();ROS_INFO("No.%u.pcd",current_num);}
        else if(output_type_option=="bin") {Write_bin();ROS_INFO("No.%u.bin",current_num);}
    }
    catch (cv_bridge::Exception e)
    {
        ROS_ERROR("Could noe convert from '%s' to '16UC1'.",msg->encoding.c_str());
    }

    Update_path();
}

void TransPointCloud::Get_Input_Cloud()
{
    Get_Input_Path();
    if(input_type_option=="pcd"){
    if(cloud_type=="XYZ"){
        if (pcl::io::loadPCDFile(input_filename, cloud_xyz)==-1){
            std::cerr << "ERROR: Cannot open file "<<std::endl;exit(1);}}
    else if(cloud_type=="XYZI"){
        if (pcl::io::loadPCDFile(input_filename, cloud_xyzi)==-1){
            std::cerr << "ERROR: Cannot open file "<<std::endl;exit(1);}}
    }

    else if(input_type_option=="bin"){
    std::fstream input(input_filename.c_str(), std::ios::in | std::ios::binary);
	if(!input.good())
		std::cerr << "Could not read file: " << input_filename << std::endl;
	input.seekg(0, std::ios::beg);
    pcl::PointXYZ point_xyz;pcl::PointXYZI point_xyzi;
    if(cloud_type=="XYZ"){
      for (size_t i = 0; input.good() && !input.eof(); i++){
        input.read((char *) &point_xyz.x, 3*sizeof(float));
        cloud_xyz.push_back(point_xyz);}}

    if(cloud_type=="XYZI"){
      for (size_t i = 0; input.good() && !input.eof(); i++){
        input.read((char *) &point_xyzi.x, 3*sizeof(float));
		input.read((char *) &point_xyzi.intensity, sizeof(float));
        cloud_xyzi.push_back(point_xyzi);}}

    input.close();}

}

void TransPointCloud::Write_Ouput_Cloud()
{
    Get_Output_Path();
    if(output_type_option=="pcd"){ //bin2pcd  pcd2pcd 
      // std::cout<<output_filename<<std::endl;
      Write_pcd();
    }

    else if(output_type_option=="bin"){ //pcd2bin  bin2bin 
      // std::cout<<output_filename<<std::endl;
      Write_bin();
    }
    Update_path();
}

void TransPointCloud::Get_Input_Path(){
    temp<<current_num;
    input_filename="";
    if(input_type_option=="pcd")
        input_filename=input_dir + "/" + temp.str() + ".pcd";
    else if(input_type_option=="bin")
        input_filename=input_dir + "/" + temp.str() + ".bin";
    temp.str("");
}

void TransPointCloud::Get_Output_Path(){

    temp<<current_num;
    output_filename="";
    if(output_type_option=="pcd")
        output_filename=out_dir + "/" + temp.str() + ".pcd";
    else if(output_type_option=="bin")
        output_filename=out_dir + "/" + temp.str() + ".bin";
    temp.str("");
}

void TransPointCloud::Trans_Init()
{
    cloud_xyz.points.clear();
    cloud_xyzi.points.clear();
}

void TransPointCloud::Update_path()
{
    temp.str("");
    current_num++;
}

sensor_msgs::PointCloud2 TransPointCloud::Point2Sensor()
{
    sensor_msgs::PointCloud2 sensor_cloud;
    if(cloud_type=="XYZ"){
        pcl::toROSMsg(cloud_xyz,sensor_cloud);
        // std::cout<<"XYZ"<<std::endl;
    }
    else if(cloud_type=="XYZI"){
        pcl::toROSMsg(cloud_xyzi,sensor_cloud);
        // std::cout<<"XYZI"<<std::endl;
    }
    return sensor_cloud;
}

void TransPointCloud::Write_bin()
{
    std::ofstream bin_file(output_filename.c_str(),std::ios::out|std::ios::binary|std::ios::app);
    if(!bin_file.good()) std::cout<<"Couldn't open "<<output_filename<<std::endl;  

      //PCD 2 BIN 
    if(cloud_type=="XYZ"){
    for (size_t i = 0; i < cloud_xyz.points.size (); ++i)
        bin_file.write((char*)&cloud_xyz.points[i].x,3*sizeof(float));}
    // bin_file.write((char*)&cloud.points[i].intensity,sizeof(float));
    if(cloud_type=="XYZI"){
    for (size_t i = 0; i < cloud_xyzi.points.size (); ++i){
        bin_file.write((char*)&cloud_xyzi.points[i].x,3*sizeof(float));
        bin_file.write((char*)&cloud_xyzi.points[i].intensity,sizeof(float));}}

    bin_file.close();
}

void TransPointCloud::Write_pcd()
{
    if(cloud_type=="XYZ") pcl::io::savePCDFile(output_filename, cloud_xyz);
    else if(cloud_type=="XYZI") pcl::io::savePCDFile(output_filename, cloud_xyzi);
}