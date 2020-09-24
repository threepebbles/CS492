#!/usr/bin/env python
import sys
import rospy
import numpy as np
from complex_action_client import arm_client as ac
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose #PointStamped,

import unittest

class TestMoveStraight(unittest.TestCase):

    def test_get_pose(self):
        rospy.init_node('arm_client')
        rospy.sleep(1)
        controller_ns = "arm_controller"
        arm = ac.ArmClient(timeout_scale=0.6, controller_ns=controller_ns, use_gripper=False)
        rospy.sleep(1)

        p = arm.getEndeffectorPose()
        self.assertTrue(p is not None)


    def test_move_straight_pose(self):
        rospy.init_node('arm_client')
        rospy.sleep(1)
        controller_ns = "arm_controller"
        arm = ac.ArmClient(timeout_scale=0.6, controller_ns=controller_ns, use_gripper=False)
        rospy.sleep(1)

        arm.moveJoint([0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., np.pi/4.], timeout=4.0)
        self.assertTrue( arm.get_result()==0 )

        # Execute p2p pose
        pose1 = Pose(position=Point(x=0.6, y=-0.2, z=0.4),
                     orientation=Quaternion(x=0., y=0., z=-0.707, w=0.707))
        arm.movePose(pose1, timeout=2.)
        self.assertTrue( arm.get_result()==0 )


    def test_move_straight_poses(self):
        rospy.init_node('arm_client')
        rospy.sleep(1)
        controller_ns = "arm_controller"
        arm = ac.ArmClient(timeout_scale=0.6, controller_ns=controller_ns, use_gripper=False)
        rospy.sleep(1)

        ## arm.moveJoint([0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2.0, 0.05], timeout=3.0)
        arm.moveJoint([0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., np.pi/4.], timeout=4.0)
        self.assertTrue( arm.get_result()==0 )

        # Execute p2p pose
        pose1 = Pose(position=Point(x=0.8, y=-0.2, z=0.4),
                     orientation=Quaternion(x=0., y=0., z=-0.707, w=0.707))

        pose2 = Pose(position=Point(x=0.85, y=-10.2, z=0.4),
                     orientation=Quaternion(x=0., y=0., z=-0.707, w=0.707))

        pose3 = Pose(position=Point(x=0.85, y=-0.2-0.2, z=0.4),
                     orientation=Quaternion(x=0., y=0., z=-0.707, w=0.707))

        pose4 = Pose(position=Point(x=0.85, y=-0.2-0.2, z=0.2),
                     orientation=Quaternion(x=0., y=0., z=-0.707, w=0.707))

        pose5 = Pose(position=Point(x=0.85, y=-0.2, z=0.2),
                     orientation=Quaternion(x=0., y=0., z=-0.707, w=0.707))


        arm.movePose(pose1, timeout=2.)
        self.assertEqual( arm.get_result(), 0 )

        ## for i in xrange(2):
        arm.movePose(pose2, timeout=2.)
        self.assertEqual( arm.get_result(), 0 )

        arm.movePose(pose3, timeout=2.)
        self.assertEqual( arm.get_result(), 0 )

        arm.movePose(pose4, timeout=2.)
        self.assertEqual( arm.get_result(), 0 )

        arm.movePose(pose5, timeout=2.)
        self.assertEqual( arm.get_result(), 0 )

if __name__ == '__main__':
    unittest.main()
