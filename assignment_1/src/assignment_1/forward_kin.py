#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose

client = None


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
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
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
    

def your_forward_kinematics(theta):
    #------------------------------------------------------------
    # Place your homogeneous transformation matrix here! 



    # you can print out a pose message by filling followings
    # please, do not import external library. 
    ps = Pose()
    ps.position.x = 
    ps.position.y = 
    ps.position.z = 
    ps.orientation.x = 
    ps.orientation.y = 
    ps.orientation.z = 
    ps.orientation.w =    
    #------------------------------------------------------------
    
    print ps
    return ps
    


if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    # create action client
    main()

    # Problem 2- C.i
    theta = [0,0,0,0,0,0]
    print your_forward_kinematics(theta)
    ## move(theta)


    # Problem 2- C.i    
    theta = [0,0,0,0,0,0]
    print your_forward_kinematics(theta)
    ## move(theta)


    
