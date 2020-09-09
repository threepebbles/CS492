#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *


client = None


def main():
            
    global client
    try:
        client = actionlib.SimpleActionClient('trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for ur5_arm server..."
        client.wait_for_server()
        print "Connected to ur5_arm server"
        ## move()
        ## print "moved"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
    
    rospy.loginfo("Initialized UR5")


def move(angles):
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        ## JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=angles, velocities=[0]*6, time_from_start=rospy.Duration(3.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    



if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    main()

    move([0,-1.8,1.0,0,0,0])
    rospy.sleep(1)

    
