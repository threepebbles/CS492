#!/usr/bin/env python
import rospy
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose

QUEUE_SIZE = 10

if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)

    # initialize publishers and Services
    obj_pose_viz_pub  = rospy.Publisher("pose_viz",
                                            PoseStamped,
                                            queue_size=QUEUE_SIZE,
                                            latch=True)

    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    try:
        obj_pose = pose_srv_req("eraser").pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e
    print obj_pose

    # visualize the target object pose in RViZ
    ps = PoseStamped()
    ps.header.frame_id  = 'world'
    ps.pose = obj_pose
    obj_pose_viz_pub.publish(ps)
    
    rospy.spin()
