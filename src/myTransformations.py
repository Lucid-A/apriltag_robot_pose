#! /usr/bin/env python

import numpy as np
from transforms3d import quaternions as quat

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

def inverseTransform(tf):
    q = tf.transform.rotation
    t = tf.transform.translation
    R = quat.quat2mat([q.w, q.x,q.y, q.z])
    M = np.hstack((R,[[t.x],[t.y],[t.z]]))
    M = np.vstack((M,[0,0,0,1]))
    inv_M = np.linalg.inv(M)

    inv_R = inv_M[0:3,0:3]
    inv_t = inv_M[0:3,3]
    inv_q = quat.mat2quat(inv_R)
    
    inv = TransformStamped()
    inv.header.seq = tf.header.seq
    inv.header.stamp = tf.header.stamp
    inv.child_frame_id = tf.header.frame_id
    inv.header.frame_id = tf.child_frame_id
    
    inv.transform.rotation.w = inv_q[0]
    inv.transform.rotation.x = inv_q[1]
    inv.transform.rotation.y = inv_q[2]
    inv.transform.rotation.z = inv_q[3]
    
    inv.transform.translation.x = inv_t[0]
    inv.transform.translation.y = inv_t[1]
    inv.transform.translation.z = inv_t[2]
    
    return inv

def Transform2PoseStamped(tf):
    pose = PoseStamped()
    pose.header.seq = tf.header.seq
    pose.header.stamp = tf.header.stamp
    pose.header.frame_id = tf.header.frame_id

    pose.pose.orientation = tf.transform.rotation
    pose.pose.position.x = tf.transform.translation.x
    pose.pose.position.y = tf.transform.translation.y
    pose.pose.position.z = tf.transform.translation.z

    return pose

def PoseStamped2Transform(pose, child_frame_id):
    trans = TransformStamped()
    trans.header.seq = pose.header.seq
    trans.header.stamp = pose.header.stamp
    trans.header.frame_id = pose.header.frame_id
    trans.child_frame_id = child_frame_id

    trans.transform.rotation = pose.pose.orientation
    trans.transform.translation.x = pose.pose.position.x
    trans.transform.translation.y = pose.pose.position.y
    trans.transform.translation.z = pose.pose.position.z
    
    return trans
   
def AprilTagDetection2PoseStamped(tag):
    pose = PoseStamped()
    pose.header = tag.pose.header
    pose.pose = tag.pose.pose.pose
    
    return pose
    
