#!/usr/bin/env python
import time
import roslib#; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import numpy as np
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

