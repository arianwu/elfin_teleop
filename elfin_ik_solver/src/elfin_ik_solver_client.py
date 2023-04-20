#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from elfin_ik_solver_msgs.srv import *
from geometry_msgs.msg import Pose
import numpy as np

def add_two_ints_client(pose):
    rospy.wait_for_service('calculate_ik')
    try:
        add_two_ints = rospy.ServiceProxy('calculate_ik', CalculateIK)
        resp1 = add_two_ints(pose)
        return resp1.solution
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 1.2

    pose.orientation.w = 1
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    xd = add_two_ints_client(pose)
    print(np.round(xd.position, 2))

