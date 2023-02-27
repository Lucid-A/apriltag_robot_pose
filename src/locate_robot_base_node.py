#! /usr/bin/env python

import rospy as ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class LocateBaseNode:
    def __init__(self):
        ros.init_node("locate_base")
        
        self.pub = ros.Publisher("/pose/robot_pose_ekf", PoseStamped, queue_size=10)
        self.sub = ros.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.callback)

        ros.spin()

    def callback(self, msg):
        try:
            self.pub.publish(PoseStamped(header=msg.header, pose=msg.pose.pose))
            print(msg.pose.pose)
        except ros.ROSInterruptException:
            pass
    
if __name__ == "__main__":
    node = LocateBaseNode()
