#!/usr/bin/env python
import sys, time
import rospy
import numpy as np, math
import actionlib
import threading
import PyKDL

from pykdl_utils.kdl_kinematics import create_kdl_kin
from hrl_geom.pose_converter import PoseConv

from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose

from complex_action_client import arm_client 

import rospy



if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    # create action client
    arm = arm_client.ArmClient()

    joints = arm.getJointAngles()

    x = arm.arm_kdl.forward_recursive(joints)
    print np.shape(x)


    
