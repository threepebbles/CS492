#!/usr/bin/env python
import numpy as np
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose

client = None
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def main():
            
    global client
    try:
        client = actionlib.SimpleActionClient('trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for ur5_arm server..."
        client.wait_for_server()
        print "Connected to ur5_arm server"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
    
    rospy.loginfo("Initialized UR5")


def move(angles):    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        ## JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=angles, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise



def move_joint(angles):
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        ## JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=angles, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
    client.send_goal(g)
    ## try:
    ##     client.wait_for_result()
    ## except KeyboardInterrupt:
    ##     client.cancel_goal()
    ##     raise


def move_position(goal_pose, init_joint):
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    #------------------------------------------------------------
    # ADD YOUR CODE
    #------------------------------------------------------------
    # construct forward kinematics (problem 2)

    # Get start position and joint angles
        
    # Get a position trajectory
        
    # Get a sequence of joint angles that track the position trajecotory by using Jacobian
    q = theta
    dt = 0.05
    time_from_start = 0
    for i in range(1, len(pose_traj)):
        J = 
        Jp = J[:3]
        J_inv = 

        dx =         
        dtheta = J_inv * dx

        q = np.array(q)+np.array(dtheta)[:,0]
        time_from_start = time_from_start + dt
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start))
            )        
    #------------------------------------------------------------
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

    return q


if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    # create action client
    main()

    # Problem 3
    theta = [0.516, -1.28, 1.27, -1.72, -1.62, 0]
    move_joint(theta)
    #print your_forward_kinematics(theta) #by importing your forward kinematics
    #rospy.spin()
    
    goal = Pose()
    goal.position.x = 0.487
    goal.position.y = -0.4
    goal.position.z = 0.274
    move_position(goal, theta)

