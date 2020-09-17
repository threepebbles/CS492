#!/usr/bin/env python
import sys, math, numpy as np
from copy import copy, deepcopy
import rospy
## import PyKDL
import misc
from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient




if __name__ == '__main__':

    rospy.init_node('arm_client')
    rospy.sleep(1)
    arm = UR5ArmClient(timeout_scale=1., sim=True)


    # ---------------------------------------------------------------------
    # Check the robot status
    # --------------------------------------------------------------------
    # Get the current joint angle
    print arm.getJointAngles()
    # Get the current endeffector pose
    print arm.getEndeffectorPose()

    # ---------------------------------------------------------------------
    # Check gripper functions
    # ---------------------------------------------------------------------
    arm.gripperOpen()
    arm.gripperClose()
    arm.getGripperState()


    # ---------------------------------------------------------------------
    # Check joint movements
    # ---------------------------------------------------------------------    
    # Move Joints
    arm.moveJoint([0, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], timeout=10.0)
    arm.moveJoint([np.pi/3, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], timeout=10.0)


    # Move following a joint trajectory
    #arm.moveJointTraj([[please, add joints],[add joints], ..., [add goal joints]], timeout=10.0)
    
    
    
    
