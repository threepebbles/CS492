#!/usr/bin/env python
import rospy

from complex_action_client import arm_control as ac


controller_ns = "arm_controller"
 
rospy.init_node('arm_control')
rospy.sleep(1)
    
arm = ac.armControl(timeout_scale=0.6, controller_ns=controller_ns)
rospy.sleep(1)

arm.gripperOpen()
rospy.sleep(5)
arm.gripperClose()
