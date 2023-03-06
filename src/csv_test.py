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

if "__main__" == __name__:
    ros.init_node("fusion_location", anonymous=False)


    # Open a CSV file in write mode
    with open("dashgo_hand_on_eye.csv", "w") as of:
        # Instantiate a csv.writer object by passing in the file as its argument
        writer = csv.writer(of)
        # Write the list variable as a row to the CSV file
        writer.writerow(["zlz", 23, "Student"])
