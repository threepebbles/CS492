#!/usr/bin/env python
import rospy
import numpy as np
import copy
import tf

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose
from complex_action_client import misc
from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient

QUEUE_SIZE = 10

def get_object_frame(target_object):
    """ Return the object top surface frame wrt the world frame.
    """
    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    try:
        obj_pose = pose_srv_req(target_object).pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    world2obj  = misc.pose2KDLframe(obj_pose)
    return world2obj 


def get_base_frame():
    # T from world to base_link
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
        break
    return misc.list2KDLframe(trans+rot)
    

if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)
    
    # initialize publishers and Services
    obj_pose_viz_pub  = rospy.Publisher("pose_viz",
                                            PoseStamped,
                                            queue_size=QUEUE_SIZE,
                                            latch=True)
    target_object = 'soap'    
    world2base = get_base_frame()

    # compute grasping pose ----------------------------
    world2obj = get_object_frame(target_object)
    print world2obj
    
    base2obj  = world2base.Inverse() * world2obj
    #base2obj.M.DoRotX(np.pi)
    base2obj.p[2] -= 0.03
    grasp_ps = misc.KDLframe2Pose(base2obj)
    # --------------------------------------------------

    # obtain a pre-grasping pose -----------------------
    pre_grasp_ps = copy.deepcopy(grasp_ps)
    pre_grasp_ps.position.z += 0.1
    # --------------------------------------------------
    
    # Initialize the UR5 control object
    arm = UR5ArmClient(timeout_scale=1., sim=True)

    arm.moveJoint([-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0])
    arm.gripperOpen()
    arm.movePose(pre_grasp_ps, 4.)
    arm.movePose(grasp_ps, 4.)
    arm.gripperClose()
    arm.movePose(pre_grasp_ps, 4.)
    arm.moveJoint([-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0])

    # visualize the target object pose in RViZ
    ## ps = PoseStamped()
    ## ps.header.frame_id  = 'world'
    ## ps.pose = misc.KDLframe2Pose(world2obj)
    ## obj_pose_viz_pub.publish(ps)
    rospy.spin()
