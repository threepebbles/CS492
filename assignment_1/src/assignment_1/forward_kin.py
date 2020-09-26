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


import numpy as np 
def get_T(d, theta, a, alpha):
    aa = np.array([[1, 0, 0, a], \
           [0, np.cos(alpha), -np.sin(alpha), 0], \
           [0, np.sin(alpha), np.cos(alpha), 0], \
           [0, 0, 0, 1] \
           ])

    dt = np.array([[np.cos(theta), -np.sin(theta), 0, 0], \
            [np.sin(theta), np.cos(theta), 0, 0], \
            [0, 0, 1, d], \
            [0, 0, 0, 1] \
           ])

    return np.dot(dt, aa)


def your_forward_kinematics(theta, T):
    #------------------------------------------------------------
    # Place your homogeneous transformation matrix here! 

    tr = T[0][0]+T[1][1]+T[2][2]
    
    if (tr > 0):
        S = np.sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (T[2][1] - T[1][2]) / S
        qy = (T[0][2] - T[2][0]) / S 
        qz = (T[1][0] - T[0][1]) / S 
    elif ((T[0][0] > T[1][1])&(T[0][0] > T[2][2])):
        S = np.sqrt(1.0 + T[0][0] - T[1][1] - T[2][2]) * 2;
        qw = (T[2][1] - T[1][2]) / S
        qx = 0.25 * S
        qy = (T[0][1] + T[1][0]) / S 
        qz = (T[0][2] + T[2][0]) / S 
    elif (T[1][1] > T[2][2]):
        S = np.sqrt(1.0 + T[1][1] - T[0][0] - T[2][2]) * 2
        qw = (T[0][2] - T[2][0]) / S
        qx = (T[0][1] + T[1][0]) / S
        qy = 0.25 * S
        qz = (T[1][2] + T[2][1]) / S
    else:
        S = np.sqrt(1.0 + T[2][2] - T[0][0] - T[1][1]) * 2
        qw = (T[1][0] - T[0][1]) / S
        qx = (T[0][2] + T[2][0]) / S
        qy = (T[1][2] + T[2][1]) / S
        qz = 0.25 * S

    # you can print out a pose message by filling followings
    # please, do not import external library. 
    ps = Pose()
    ps.position.x = T[0][3]
    ps.position.y = T[1][3]
    ps.position.z = T[2][3]
    ps.orientation.x = qx
    ps.orientation.y = qy
    ps.orientation.z = qz
    ps.orientation.w = qw   
    #------------------------------------------------------------
    

    return ps
    


if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    # create action client
    main()

    # Problem 2- C.i

    theta = [0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0]

    T01 = get_T(0.089,   theta[0],   0,        -np.pi/2    )
    T12 = get_T(0.016,   theta[1],   0.425,    0           )
    T23 = get_T(0,       theta[2],   0.392,    0           )
    T34 = get_T(0.093,   theta[3],   0,        -np.pi/2    )
    T45 = get_T(0.095,   theta[4],   0,        np.pi/2     )
    T56 = get_T(0.24,    theta[5],   0,        0           )
    Ts = [T01, T12, T23, T34, T45, T56]

    T = np.identity(4)
    for i in range(0, len(Ts)):
        T = np.dot(T, Ts[i])
        
    print your_forward_kinematics(theta, T)
    move(theta)
    rospy.sleep(2)

    # Problem 2- C.i    
    
    # theta = [-np.pi/4, np.pi/4, np.pi/2, 0, 0, 0]
    # print your_forward_kinematics(theta)
    # move(theta)

