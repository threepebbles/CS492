#!/usr/bin/env python
import rospy
import numpy as np

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose
from complex_action_client import misc

QUEUE_SIZE = 10

if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)

    target_object = 'eraser'

    # initialize publishers and Services
    obj_pose_viz_pub  = rospy.Publisher("pose_viz",
                                            PoseStamped,
                                            queue_size=QUEUE_SIZE,
                                            latch=True)

    grasp_pose_viz_pub  = rospy.Publisher("grasp_viz",
                                              PoseStamped,
                                              queue_size=QUEUE_SIZE,
                                              latch=True)
    
    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    try:
        obj_pose = pose_srv_req(target_object).pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    rospy.wait_for_service("get_object_height")
    pose_srv_req = rospy.ServiceProxy("get_object_height", String_Pose)
    
    try:
        height_pose = pose_srv_req(target_object).pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e
        
    print obj_pose, height_pose

    # compute grasping pose ----------------------------
    obj_frame = misc.pose2KDLframe(obj_pose)
    obj_frame.M.DoRotX(np.pi)
    obj_frame.p += misc.pose2KDLframe(height_pose).p
    obj_frame.p[2] -= 0.03
    grasp_pose = misc.KDLframe2Pose(obj_frame)
    # --------------------------------------------------

    # visualize the target object pose in RViZ
    ps = PoseStamped()
    ps.header.frame_id  = 'world'
    ps.pose = obj_pose
    obj_pose_viz_pub.publish(ps)

    ps = PoseStamped()
    ps.header.frame_id  = 'world'
    ps.pose = grasp_pose
    grasp_pose_viz_pub.publish(ps)
    
    rospy.spin()
