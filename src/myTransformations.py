#! /usr/bin/env python

import numpy as np
from transforms3d import quaternions as quat

import rospy as ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
#from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

def TransformMsg2Mat(tf):
    q = tf.transform.rotation
    t = tf.transform.translation
    R = quat.quat2mat([q.w, q.x,q.y, q.z])
    M = np.hstack((R,[[t.x],[t.y],[t.z]]))
    M = np.vstack((M,[0,0,0,1]))
    return M

def TransformMat2Msg(mat, frame_id=None, child_frame_id=None, now=True):
    tf = TransformStamped()
    if now:
        tf.header.stamp = ros.Time.now()
    if frame_id is not None:
        tf.header.frame_id = frame_id
    if child_frame_id is not None:
        tf.child_frame_id = child_frame_id

    R = mat[0:3,0:3]
    t = mat[0:3,3]
    q = quat.mat2quat(R)

    tf.transform.rotation.w = q[0]
    tf.transform.rotation.x = q[1]
    tf.transform.rotation.y = q[2]
    tf.transform.rotation.z = q[3]
    
    tf.transform.translation.x = t[0]
    tf.transform.translation.y = t[1]
    tf.transform.translation.z = t[2]
    
    return tf

def inverseTransform(tf):
    M = TransformMsg2Mat(tf)
    inv_M = np.linalg.inv(M)
    inv = TransformMat2Msg(inv_M)

    inv.header.seq = tf.header.seq
    inv.header.stamp = tf.header.stamp
    inv.child_frame_id = tf.header.frame_id
    inv.header.frame_id = tf.child_frame_id
    
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
    
def AprilTagDetection2Transform(tag, child_frame_id):
    pose = AprilTagDetection2PoseStamped(tag)
    tf = PoseStamped2Transform(pose, child_frame_id)
    return tf