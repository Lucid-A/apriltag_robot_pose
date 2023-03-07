#! /usr/bin/env python

import rospy as ros
from myTransformations import *
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

class LocateCameraNode:
    def __init__(self):
        self.rate = ros.Rate(10) # Hz
        self.sub = ros.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.pub0 = ros.Publisher("/tag_detections/is_available", Bool, queue_size=10)
        self.pub1 = ros.Publisher("/pose/tag_in_cam", PoseStamped, queue_size=10)
        self.pub2 = ros.Publisher("/pose/cam_in_tag", PoseStamped, queue_size=10)

        ros.spin()

    def callback(self, msg):
        cnt = len(msg.detections)
        if not cnt:
            print("No tag detected!")
            self.pub0.publish(Bool(False))
        elif 1 == cnt:
            #h = msg.header
            tag = msg.detections[0]
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
                
if __name__ == "__main__":
    ros.init_node("locate_camera", anonymous=False)
    LocateCameraNode()
