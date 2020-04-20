#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
import rospy
import time
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Float32MultiArray
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append("..")
import numpy as np
from roslib import message


def cloud_publish(filename):

    pcl_msg = []
    cloud_p = []
    cloud_msg = PointCloud2()
    pcl_msg = np.load(filename) #(pointnumber,3/4)
    
    
    print "Get: ",filename,"------ data number:",np.array(pcl_msg).shape
    global pub_cloud
    head = std_msgs.msg.Header()
    head.stamp = rospy.Time.now()
    head.frame_id = "py_pub_cloud"
    point_field = PointField
    if cloud_type == "XYZ":
        point_fields = [PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1)]
        if crop_cloud == "True": #原始数据为(64, 512, 6)
            cloud_p = pcl_msg[:, :, :3]
        else:
            cloud_p = pcl_msg
        cloud_p=np.reshape(cloud_p,(-1,3))
    elif cloud_type == "XYZI":
        point_fields = [PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('intensity', 16, PointField.FLOAT32, 1),]
        if crop_cloud == "True":
            cloud_p = pcl_msg[:, :, :4]
        else:
            cloud_p = pcl_msg
        cloud_p=np.reshape(cloud_p,(-1,4))
    print "cloud publish shape:",np.array(cloud_p).shape
    pcloud = pc2.create_cloud(head,point_fields,cloud_p)
    pub_cloud.publish(pcloud)


if __name__ == '__main__':
    rospy.init_node('tranp_pointcloud_node', anonymous=True)
    
    input_dir = rospy.get_param('~input_dir_')
    start_num = rospy.get_param('~start_num_')
    current_num = start_num
    trans_count = rospy.get_param('~trans_count_')
    cloud_type = rospy.get_param('~cloud_type_')
    pub_topic = rospy.get_param('~pub_topic_')
    loop = rospy.get_param('~loop_')
    crop_cloud = rospy.get_param('~crop_cloud_')

    pub_cloud = rospy.Publisher(pub_topic,PointCloud2,queue_size=1)
    count = trans_count
    print "Publish start......"
    while count>0 and not rospy.is_shutdown():
        count = count - 1
        filename = input_dir + "/" + np.str(current_num) +".npy"
        time.sleep(0.5)
        cloud_publish(filename)
        current_num = current_num + 1
        if loop == 1 and count == 0:
            current_num = start_num
            count = trans_count
    print "Publish end......"