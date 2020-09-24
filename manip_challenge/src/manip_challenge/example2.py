#!/usr/bin/env python
import sys, math, numpy as np
from copy import copy, deepcopy
import rospy
## import PyKDL
import misc

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse


if __name__ == '__main__':

    rospy.init_node('example1')
    rospy.sleep(1)

    rospy.wait_for_service('/get_object_pose')
    pose_srv_req = rospy.ServiceProxy('/get_object_pose', String_Pose)
    
    # You can send a query to obtain the pose of a selected object
    obj_pose = pose_srv_req("book").pose
    print obj_pose
