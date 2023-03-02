#! /usr/bin/env python
import csv
import numpy as np
import transforms3d as tf3d
from math import *
from myTransformations import *

import rospy as ros
import tf2_ros as tf2
import message_filters as mf
from geometry_msgs.msg import PoseStamped, TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray

# Tx is the alias of T_B_C
theta = radians(59)
dx, dy, dz = 27, 0, 16
t = np.array([[dx, dy, dz]], dtype=np.float64).T / 100
R = np.array([
        [ 0, -sin(theta),  cos(theta)],
        [-1,           0,           0],
        [ 0, -cos(theta), -sin(theta)]
    ])

Tx = np.hstack((R,t))
Tx = np.vstack((Tx,[0,0,0,1]))


class FusionLocation:
    def __init__(self):
        self.rate = ros.Rate(10) # 10
        self.flag = False # flag that indicates whether the initial transform is done
        
        self.buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.buffer)
        self.broadcaster = tf2.TransformBroadcaster()

        self.pub = ros.Publisher("/pose/fusion", PoseStamped, queue_size=10)

        self.sub_odom = mf.Subscriber("/pose/robot_pose_ekf", PoseStamped)
        self.sub_tags = mf.Subscriber("/tag_detections", AprilTagDetectionArray)
        
        self.ts = mf.ApproximateTimeSynchronizer([self.sub_odom, self.sub_tags], 10)
        self.ts.registerCallback(self.callback)


        self.B = "base_footprint"
        self.C = "usb_cam"
        self.O = "odom_combined"
        self.T = "tag"
        self.BT = "tag_combined"

        self.T_B_C = Tx
        self.T_O_T = np.eye(4)
        self.T_BO_BT = np.eye(4)

        ros.spin()

    def callback(self, msg_odom, msg_tags : AprilTagDetectionArray):
        cnt = len(msg_tags.detections)
        if not self.flag:
            if 1 == cnt:
                self.flag = True
                
                tag = msg_tags.detections[0]
                
        if not cnt:
            print("No tag detected!")
            tf_
            tf.header.stamp = msg_tags.header.stamp
            tf.child_frame_id = msg_tags.header.frame_id
            tf.header.frame_id = msg_tags.child_frame_id
        elif 1 == cnt:
            if not self.flag:
                self.flag = True

            tag = msg_tags.detections[0]
            #header = tag.pose.header

            pose_tag_in_cam = AprilTagDetection2PoseStamped(tag)
            pose_cam_in_tag = Transform2PoseStamped(
                                inverseTransform(
                                  PoseStamped2Transform(pose_tag_in_cam, "tag")
                                )
                              )
            try:
                print("pose_tag_in_cam:\n", pose_tag_in_cam)
                print("pose_cam_in_tag:\n", pose_cam_in_tag)
                self.pub0.publish(Bool(True))
                self.pub1.publish(pose_tag_in_cam)
                self.pub2.publish(pose_cam_in_tag)
            except ros.ROSInterruptException:
                pass
        else:
            print(f"{cnt} tags are detected!")

if "__main__" == __name__:
    ros.init_node("fusion_location", anonymous=False)
