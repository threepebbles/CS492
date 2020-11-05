#!/usr/bin/env python
import rospy
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse


if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)

    # Initialize a service client
    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    # Query the pose of an object
    try:
        obj_pose = pose_srv_req("eraser").pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    print obj_pose
