#!/usr/bin/env python
import rospy

from complex_action_client import arm_client_ur5 as ac
import numpy as np

import unittest


class TestMoveJoints(unittest.TestCase):

    def test_get_joints(self):
        rospy.init_node('arm_control')
        rospy.sleep(1)
        controller_ns = "arm_controller"
        arm = ac.UR5ArmClient(timeout_scale=1., controller_ns=controller_ns, use_gripper=False)
        #arm = ac.ArmClient(timeout_scale=0.6)
        rospy.sleep(1)
        joints =  arm.getJointAngles()
        self.assertTrue(joints is not None and len(joints)==6)

    def test_move_joint(self):
        rospy.init_node('arm_control')
        rospy.sleep(1)
        controller_ns = "arm_controller"
        arm = ac.UR5ArmClient(timeout_scale=1., controller_ns=controller_ns, use_gripper=False)
        #arm = ac.ArmClient(timeout_scale=0.6)
        rospy.sleep(1)

        # Execute p2p joint
        Q1 = [0,-1.57,0,-1.57,0,0]
        arm.moveJoint(Q1, timeout=7.0)
        
        self.assertTrue( arm.get_result()==0 )
        print arm.get_state()
        print arm.get_result()
        
    def test_move_joints(self):
        rospy.init_node('arm_control')
        rospy.sleep(1)
        controller_ns = "arm_controller"
        arm = ac.UR5ArmClient(timeout_scale=1., controller_ns=controller_ns, use_gripper=False)
        #arm = ac.ArmClient(timeout_scale=0.6)
        rospy.sleep(1)

        # Execute p2p joint
        Q1 = [0,-1.57,0,-1.57,0,0]
        Q2 = [0,-1.57,0,-1.3,-3.,0]
        Q3 = [0,-3.,2.7,-1.3,-3.,0]
        arm.moveJoint(Q1, timeout=7.0)
        self.assertTrue(arm.get_result()==0) 
        arm.moveJoint(Q2, timeout=4.0)
        self.assertTrue(arm.get_result()==0) 
        arm.moveJoint(Q3, timeout=8.0)
        self.assertTrue(arm.get_result()==0) 



if __name__ == '__main__':
    unittest.main()
