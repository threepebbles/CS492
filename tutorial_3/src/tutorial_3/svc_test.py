#!/usr/bin/env python
import rospy
from control_msgs.msg import *
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse


if __name__ == '__main__':
    rospy.init_node("svc_test")
    rospy.sleep(1)

    target_obj = 'glue'
    rospy.wait_for_service("get_object_range")
    pose_srv_req = rospy.ServiceProxy("get_object_range", String_Pose)
    
    try:
        obj_pose = pose_srv_req(target_obj).pose
        print obj_pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e
    

    rospy.spin()
