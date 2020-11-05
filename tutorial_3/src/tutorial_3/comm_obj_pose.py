#!/usr/bin/env python
import rospy
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse


if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)


    # Confirm the list of topics: rostopic list

    # How can we get the values for the current joint angles?
    # 1. Check the information of a topic: rostopic info /joint_state
    # 2. Subscribe the topic: rostopic echo /joint_states
    # 3. You can subscribe the topic via a python script


    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    try:
        obj_pose = pose_srv_req("eraser").pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    print obj_pose
