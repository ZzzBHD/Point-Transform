#!/usr/bin/env python

import os
import sys
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Float32MultiArray
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append("..")
import numpy as np
import cv2


def cloud_callback(msg):

    pcl_msg = []
    if cloud_type == "XYZ":
        pcl_msg = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z"))
    elif cloud_type == "XYZI":
        pcl_msg = pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z","intensity"))
    
    np_p = np.array(list(pcl_msg), dtype=np.float32)
    global current_num
    filename = out_dir + "/" + np.str(current_num) +".npy"
    np.save(filename,np_p)
    print "Get: ",filename,"------ data number:",np_p.shape
    current_num=current_num+1

if __name__ == '__main__':
    rospy.init_node('tranp_pointcloud_node', anonymous=True)

    out_dir = rospy.get_param('~out_dir_')
    start_num = rospy.get_param('~start_num_')
    current_num = start_num
    cloud_topic = rospy.get_param('~pub_topic_p')
    cloud_type = rospy.get_param('~cloud_type_')

    sub_ = rospy.Subscriber(cloud_topic, PointCloud2,
                            cloud_callback, queue_size=1)

    print("Listening......")
    rospy.spin()