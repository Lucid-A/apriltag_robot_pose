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
theta = radians(49)
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
        self.static_broadcaster = tf2.StaticTransformBroadcaster()

        self.pub = ros.Publisher("/pose/fusion", PoseStamped, queue_size=10)

        self.sub_odom = mf.Subscriber("/pose/robot_pose_ekf", PoseStamped)
        self.sub_tags = mf.Subscriber("/tag_detections", AprilTagDetectionArray)
        
        self.ts = mf.ApproximateTimeSynchronizer([self.sub_odom, self.sub_tags], 10, 1)
        self.ts.registerCallback(self.callback)


        self.B = "base_footprint"
        self.C = "usb_cam"
        self.M = "map"
        self.O = "odom_combined"
        self.T = "tag"

        self.T_B_C = Tx
        self.T_O_B = np.eye(4)
        self.T_F_O = np.eye(4)

        tf = TransformMat2Msg(self.T_B_C, self.B, self.C)
        self.static_broadcaster.sendTransform(tf)

        self.T_M_O = None
        while self.T_M_O is None:
            try:
                self.T_M_O = TransformMsg2Mat(self.buffer.lookup_transform(self.M, self.O, ros.Time()))
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
                self.T_M_O = None
                ros.loginfo(f"Didn't get Transfrom Matrix between /{self.M} and /{self.O}")
            
            for i in range(10):
                self.rate.sleep()
        
        ros.spin()

    def callback(self, msg_odom, msg_tags : AprilTagDetectionArray):
        cnt = len(msg_tags.detections)
        self.T_O_B = TransformMsg2Mat(PoseStamped2Transform(msg_odom, self.B))
        try:
            self.T_M_O = TransformMsg2Mat(self.buffer.lookup_transform(self.M, self.O, ros.Time()))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            ros.loginfo(f"Didn't get Transfrom Matrix from /{self.M} and /{self.O}")
            return

        if not self.flag:
            if 0 == cnt:
                ros.loginfo("No tag is detected yet, Fixed Frame init FAILED!")
            else:
                self.flag = True
                ros.loginfo("Successfully get Robot Pose in Fixed Frame!")
                
                tag = msg_tags.detections[0]
                self.T_C_T = TransformMsg2Mat(AprilTagDetection2Transform(tag, self.T))
                self.T_T_O = np.linalg.inv(np.dot(self.T_O_B, np.dot(self.T_B_C, self.T_C_T)))
                self.T_T_M = np.dot(self.T_T_O, np.linalg.inv(self.T_M_O))

                #tf = TransformMat2Msg(self.T_T_O, self.T, self.O)
                tf = TransformMat2Msg(self.T_T_M, self.T, self.M)
                self.broadcaster.sendTransform(tf)

                self.T_T_B = np.dot(self.T_T_O, self.T_O_B)
                pose = Transform2PoseStamped(TransformMat2Msg(self.T_T_B, self.T, self.B))
                self.pub.publish(pose)
                ros.loginfo(f"init_pose:\n{pose.pose}\n")                
        else:
            if cnt:
                tag = msg_tags.detections[0]
                self.T_C_T = TransformMsg2Mat(AprilTagDetection2Transform(tag, self.T))
                self.T_T_O = np.linalg.inv(np.dot(self.T_O_B, np.dot(self.T_B_C, self.T_C_T)))
                self.T_T_M = np.dot(self.T_T_O, np.linalg.inv(self.T_M_O))

            #tf = TransformMat2Msg(self.T_T_O, self.T, self.O)
            tf = TransformMat2Msg(self.T_T_M, self.T, self.M)
            self.broadcaster.sendTransform(tf)
            
            self.T_T_B = np.dot(self.T_T_O, self.T_O_B)
            pose = Transform2PoseStamped(TransformMat2Msg(self.T_T_B, self.T, self.B))
            self.pub.publish(pose)
                
if "__main__" == __name__:
   ros.init_node("fusion_location", anonymous=False)
   node = FusionLocation()
