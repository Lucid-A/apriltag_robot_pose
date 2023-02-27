#! /usr/bin/env python

import rospy as ros
from myTransformations import *
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

class LocateCameraNode:
    def __init__(self):
        self.rate = ros.Rate(10) # Hz
        self.sub = ros.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.pub1 = ros.Publisher("/pose/map_in_cam", PoseStamped, queue_size=10)
        self.pub2 = ros.Publisher("/pose/cam_in_map", PoseStamped, queue_size=10)

        ros.spin()

    def callback(self, msg):
        cnt = len(msg.detections)
        if not cnt:
            print("No tag detected!")
        elif 1 == cnt:
            #h = msg.header
            tag = msg.detections[0]
            #header = tag.pose.header
            
            pose_map_in_cam = AprilTagDetection2PoseStamped(tag)
            pose_cam_in_map = Transform2PoseStamped(
                                inverseTransform(
                                  PoseStamped2Transform(pose_map_in_cam, "map")
                                )
                              )
            try:
                print("pose_map_in_cam:\n", pose_map_in_cam)
                print("pose_cam_in_map:\n", pose_cam_in_map)
                self.pub1.publish(pose_map_in_cam)
                self.pub2.publish(pose_cam_in_map)
            except ros.ROSInterruptException:
                pass
        else:
            print(f"{len(msg.detections)} tags are detected!")
                
if __name__ == "__main__":
    ros.init_node("locate_camera", anonymous=False)
    LocateCameraNode()
