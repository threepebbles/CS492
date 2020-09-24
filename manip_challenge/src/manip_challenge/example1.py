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
    #arm.gripperOpen()
    #arm.gripperClose()
    #arm.getGripperState()


    # ---------------------------------------------------------------------
    # Check joint movements
    # ---------------------------------------------------------------------    
    # Move Joints
    #arm.moveJoint([0, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], timeout=10.0)
    #arm.moveJoint([np.pi/3, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], timeout=10.0)


    # ---------------------------------------------------------------------
    # Check pick-and-place
    # ---------------------------------------------------------------------       
    arm.gripperOpen()
    arm.moveJoint([0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.], timeout=5.0)
    arm.moveJoint([0.16170570835536457, -1.4298043774726588, 1.32243941822082, -1.4706636556410826, -1.5801858129904114, 0.16369705271839724], timeout=5.0)
    arm.moveJoint([0.16456877764159317, -1.4026709127086092, 1.6609417499881791, -1.829975190229056, -1.570936444921175, 0.16300417865254457], timeout=5.0)
    
    arm.gripperClose()
    arm.moveJoint([0.1551163086351373, -1.43718690172762, 1.3254160855275712, -1.4612148276759138, -1.5647456289308708, 0.15966451933142772], timeout=5.0)

    arm.moveJoint([1.3679801926428028, -1.4297246128145917, 1.2446811592243003, -1.369128425872749, -1.5598725764592507, 2.1546155544206376], timeout=5.0)
    arm.moveJoint([1.3677681606035597, -1.4306672689958717, 1.6188301424835283, -1.7400717740118714, -1.5650770640998972, 2.156513636080088], timeout=5.0)
    arm.gripperOpen()
    arm.moveJoint([1.3770551501789219, -1.434575401913625, 1.2522653950772369, -1.3755392458833133, -1.5621581114491467, 2.1658595873828146], timeout=5.0)


    # Move following a joint trajectory
    #arm.moveJointTraj([[please, add joints],[add joints], ..., [add goal joints]], timeout=10.0)
    
    
    
    
