#!/usr/bin/env python
import sys, signal
import rospy
import numpy as np
from complex_action_client import arm_client as ac
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose #PointStamped,

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

controller_ns = "arm_controller"
 
rospy.init_node('arm_client')
rospy.sleep(2)
    
arm = ac.ArmClient(timeout_scale=1., controller_ns=controller_ns, use_gripper=False)
rospy.sleep(2)


arm.moveJoint([-0.01, -0.97, np.pi/2.+0.2, -1.07, np.pi/2.0, 0.0], timeout=10.0)
#arm.moveJoint([0.0, -np.pi/2., 0, 0, 0, 0], timeout=5.0)
#print arm.getEndeffectorPose()
## print arm.getEndeffectorQuat()
rospy.sleep(2)
print arm.getEndeffectorPose()
#sys.exit()

## pose  = arm.getEndeffectorPose()
pose = Pose(position=Point(x=0.0, y=0.0, z=0.13),
             orientation=Quaternion(x=0., y=0., z=0., w=1.))
arm.movePoseRelative(pose, timeout=5.)

## pose = Pose(position=Point(x=0.0, y=0.0, z=0.1),
##              orientation=Quaternion(x=0., y=0., z=0., w=1.))
## arm.movePoseRelative(pose, timeout=5.)


## # Execute p2p pose
pose1 = Pose(position=Point(x=0.8, y=-0.2, z=0.4),
             orientation=Quaternion(x=1., y=0., z=0., w=0.))

pose2 = Pose(position=Point(x=0.85, y=-0.2, z=0.4),
             orientation=Quaternion(x=1., y=0., z=0., w=0.))

pose3 = Pose(position=Point(x=0.85, y=-0.2-0.2, z=0.4),
             orientation=Quaternion(x=1., y=0., z=0., w=0.))

pose4 = Pose(position=Point(x=0.85, y=-0.2-0.2, z=0.2),
             orientation=Quaternion(x=1., y=0., z=0., w=0.))

pose5 = Pose(position=Point(x=0.85, y=-0.2, z=0.2),
             orientation=Quaternion(x=1., y=0., z=0., w=0.))


arm.movePose(pose1, timeout=2.)
print arm.getEndeffectorPose()
rospy.sleep(1)

for i in xrange(2):
    print i, arm.getJointAngles()    
    arm.movePose(pose2, timeout=2.)
    print arm.getEndeffectorPose()

    print i, arm.getJointAngles()    
    arm.movePose(pose3, timeout=2.)
    print arm.getEndeffectorPose()

    print i, arm.getJointAngles()    
    arm.movePose(pose4, timeout=2.)
    print arm.getEndeffectorPose()

    print i, arm.getJointAngles()    
    arm.movePose(pose5, timeout=2.)
    print arm.getEndeffectorPose()
