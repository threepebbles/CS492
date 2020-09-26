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
    """ 
    A function to setup a action clinet for sending commands
    """            
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
    """ 
    A function to send a joint configuration goal to the robot. 
    This function waits for completing the commanded motion.

    Parameters
    ----------
    angles : list
        a list of joint angles. The joint order must be same as JOINT_NAMES. 
    """
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=angles, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise



def move_joint(angles):
    """ 
    A function to send a joint configuration goal to the robot.
    This function does not wait for completing the commanded motion.

    Parameters
    ----------
    angles : list
        a list of joint angles. The joint order must be same as JOINT_NAMES. 
    """
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=angles, velocities=[0]*6, time_from_start=rospy.Duration(5))]
    client.send_goal(g)

    

import forward_kin
def get_xyz_with_theta_T(theta, T):
    ps = forward_kin.your_forward_kinematics(theta, T)
    return np.array([ps.position.x, ps.position.y, ps.position.z])


def get_cum_T_list(theta):
    T01 = forward_kin.get_T(0.089,   theta[0],   0,        -np.pi/2    )
    T12 = forward_kin.get_T(0.016,   theta[1],   0.425,    0           )
    T23 = forward_kin.get_T(0,       theta[2],   0.392,    0           )
    T34 = forward_kin.get_T(0.093,   theta[3],   0,        -np.pi/2    )
    T45 = forward_kin.get_T(0.095,   theta[4],   0,        np.pi/2     )
    T56 = forward_kin.get_T(0.24,    theta[5],   0,        0           )
    Ts = [np.identity(4), T01, T12, T23, T34, T45, T56]
    
    T = np.identity(4)
    T_cum = []
    for i in range(0, len(Ts)):
        T = np.dot(T, Ts[i])
        T_cum.append(T)

    return np.array(T_cum)


def get_analytic_jacobian_mat(theta):
    J = np.zeros((3, 6))
    J[0][0] = (0.24*((np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + 0.24*((np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]))*np.sin(theta[4]) + (-0.095*(np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 0.095*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) + (0.095*(np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + 0.095*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]) + (0.392*np.sin(theta[0])*np.sin(theta[1]) - 2.40030772632881e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-0.392*np.sin(theta[0])*np.cos(theta[1]) - 2.40030772632881e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]) + (-0.24*(-6.12323399573677e-17*(np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 6.12323399573677e-17*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*(np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + 6.12323399573677e-17*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]) - 0.24*np.cos(theta[0]))*np.cos(theta[4]) - 0.425*np.sin(theta[0])*np.cos(theta[1]) - 2.60237444818813e-17*np.sin(theta[1])*np.cos(theta[0]) - 0.109*np.cos(theta[0])
    J[0][1] = (-0.24*(-6.12323399573677e-17*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + 6.12323399573677e-17*(6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) + np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]))*np.cos(theta[4]) + (0.24*((6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + 0.24*((6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) + np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]))*np.sin(theta[4]) + (-0.095*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) + (0.095*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + 0.095*(6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) + np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]) + (2.40030772632881e-17*np.sin(theta[0])*np.sin(theta[1]) - 0.392*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-2.40030772632881e-17*np.sin(theta[0])*np.cos(theta[1]) - 0.392*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]) - 2.60237444818813e-17*np.sin(theta[0])*np.cos(theta[1]) - 0.425*np.sin(theta[1])*np.cos(theta[0])
    J[0][2] = (0.24*(-(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + 0.24*((6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]))*np.sin(theta[4]) + (-0.24*(6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]))*np.cos(theta[4]) + (0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) + (0.095*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]) - (-2.40030772632881e-17*np.sin(theta[0])*np.sin(theta[1]) + 0.392*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-2.40030772632881e-17*np.sin(theta[0])*np.cos(theta[1]) - 0.392*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2])
    J[0][3] = (0.24*(-(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) - 0.24*((-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]))*np.sin(theta[4]) + (0.24*(-6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) - 0.24*(-6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]))*np.cos(theta[4]) - (-0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) + (-0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3])
    J[0][4] = (0.24*(-(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) + 0.24*((-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]))*np.cos(theta[4]) - (-1.46957615897682e-17*(-(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + 1.46957615897682e-17*((-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]) - 0.24*np.sin(theta[0]))*np.sin(theta[4])
    J[0][5] = 0
    J[1][0] = (0.24*((-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]) + 0.24*((6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]))*np.sin(theta[4]) + (-0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]) + (0.095*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 0.095*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + (-2.40030772632881e-17*np.sin(theta[0])*np.sin(theta[1]) + 0.392*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (-2.40030772632881e-17*np.sin(theta[0])*np.cos(theta[1]) - 0.392*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]) + (-0.24*(-6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) + np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*(6.12323399573677e-17*np.sin(theta[0])*np.sin(theta[1]) - np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 6.12323399573677e-17*(-6.12323399573677e-17*np.sin(theta[0])*np.cos(theta[1]) - np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) - 0.24*np.sin(theta[0]))*np.cos(theta[4]) - 2.60237444818813e-17*np.sin(theta[0])*np.sin(theta[1]) - 0.109*np.sin(theta[0]) + 0.425*np.cos(theta[0])*np.cos(theta[1])
    J[1][1] = (-0.24*(-6.12323399573677e-17*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 6.12323399573677e-17*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*(np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 6.12323399573677e-17*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]))*np.cos(theta[4]) + (0.24*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]) + 0.24*((np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]))*np.sin(theta[4]) + (-0.095*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 0.095*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]) + (0.095*(np.sin(theta[0])*np.sin(theta[1]) - 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 0.095*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + (-0.392*np.sin(theta[0])*np.sin(theta[1]) + 2.40030772632881e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + (-0.392*np.sin(theta[0])*np.cos(theta[1]) - 2.40030772632881e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]) - 0.425*np.sin(theta[0])*np.sin(theta[1]) + 2.60237444818813e-17*np.cos(theta[0])*np.cos(theta[1])
    J[1][2] = (0.24*(-(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) + 0.24*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]))*np.sin(theta[4]) + (-0.24*(-6.12323399573677e-17*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 6.12323399573677e-17*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) - 0.24*(-6.12323399573677e-17*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + 6.12323399573677e-17*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]))*np.cos(theta[4]) + (-0.095*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + 0.095*(-np.sin(theta[0])*np.cos(theta[1]) - 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + (-0.095*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) + 0.095*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]) + (-0.392*np.sin(theta[0])*np.sin(theta[1]) + 2.40030772632881e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - (0.392*np.sin(theta[0])*np.cos(theta[1]) + 2.40030772632881e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2])
    J[1][3] = (-0.24*(-6.12323399573677e-17*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 6.12323399573677e-17*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + 0.24*(6.12323399573677e-17*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 6.12323399573677e-17*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]))*np.cos(theta[4]) + (-0.24*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) + 0.24*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]))*np.sin(theta[4]) + (-0.095*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) - 0.095*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) - (0.095*(-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - 0.095*(np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3])
    J[1][4] = (0.24*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.cos(theta[3]) + 0.24*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.sin(theta[3]))*np.cos(theta[4]) - (1.46957615897682e-17*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.sin(theta[2]) + (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.cos(theta[2]))*np.sin(theta[3]) - 1.46957615897682e-17*((-np.sin(theta[0])*np.sin(theta[1]) + 6.12323399573677e-17*np.cos(theta[0])*np.cos(theta[1]))*np.cos(theta[2]) - (np.sin(theta[0])*np.cos(theta[1]) + 6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[0]))*np.sin(theta[2]))*np.cos(theta[3]) + 0.24*np.cos(theta[0]))*np.sin(theta[4])
    J[1][5] = 0
    J[2][0] = 0
    J[2][1] = (-0.24*(-6.12323399573677e-17*np.sin(theta[1])*np.sin(theta[2]) + 6.12323399573677e-17*np.cos(theta[1])*np.cos(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[2]) + 6.12323399573677e-17*np.sin(theta[2])*np.cos(theta[1]))*np.cos(theta[3]))*np.cos(theta[4]) + (0.24*(1.0*np.sin(theta[1])*np.sin(theta[2]) - 1.0*np.cos(theta[1])*np.cos(theta[2]))*np.cos(theta[3]) + 0.24*(1.0*np.sin(theta[1])*np.cos(theta[2]) + 1.0*np.sin(theta[2])*np.cos(theta[1]))*np.sin(theta[3]))*np.sin(theta[4]) + (-0.095*np.sin(theta[1])*np.sin(theta[2]) + 0.095*np.cos(theta[1])*np.cos(theta[2]))*np.sin(theta[3]) + (0.095*np.sin(theta[1])*np.cos(theta[2]) + 0.095*np.sin(theta[2])*np.cos(theta[1]))*np.cos(theta[3]) + 0.392*np.sin(theta[1])*np.sin(theta[2]) - 0.392*np.cos(theta[1])*np.cos(theta[2]) - 0.425*np.cos(theta[1])
    J[2][2] = (-0.24*(-6.12323399573677e-17*np.sin(theta[1])*np.sin(theta[2]) + 6.12323399573677e-17*np.cos(theta[1])*np.cos(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[2]) + 6.12323399573677e-17*np.sin(theta[2])*np.cos(theta[1]))*np.cos(theta[3]))*np.cos(theta[4]) + (0.24*(1.0*np.sin(theta[1])*np.sin(theta[2]) - 1.0*np.cos(theta[1])*np.cos(theta[2]))*np.cos(theta[3]) + 0.24*(1.0*np.sin(theta[1])*np.cos(theta[2]) + 1.0*np.sin(theta[2])*np.cos(theta[1]))*np.sin(theta[3]))*np.sin(theta[4]) + (-0.095*np.sin(theta[1])*np.sin(theta[2]) + 0.095*np.cos(theta[1])*np.cos(theta[2]))*np.sin(theta[3]) + (0.095*np.sin(theta[1])*np.cos(theta[2]) + 0.095*np.sin(theta[2])*np.cos(theta[1]))*np.cos(theta[3]) + 0.392*np.sin(theta[1])*np.sin(theta[2]) - 0.392*np.cos(theta[1])*np.cos(theta[2])
    J[2][3] = (0.24*(6.12323399573677e-17*np.sin(theta[1])*np.sin(theta[2]) - 6.12323399573677e-17*np.cos(theta[1])*np.cos(theta[2]))*np.sin(theta[3]) - 0.24*(6.12323399573677e-17*np.sin(theta[1])*np.cos(theta[2]) + 6.12323399573677e-17*np.sin(theta[2])*np.cos(theta[1]))*np.cos(theta[3]))*np.cos(theta[4]) + (0.24*(1.0*np.sin(theta[1])*np.sin(theta[2]) - 1.0*np.cos(theta[1])*np.cos(theta[2]))*np.cos(theta[3]) - 0.24*(-1.0*np.sin(theta[1])*np.cos(theta[2]) - 1.0*np.sin(theta[2])*np.cos(theta[1]))*np.sin(theta[3]))*np.sin(theta[4]) - (0.095*np.sin(theta[1])*np.sin(theta[2]) - 0.095*np.cos(theta[1])*np.cos(theta[2]))*np.sin(theta[3]) + (0.095*np.sin(theta[1])*np.cos(theta[2]) + 0.095*np.sin(theta[2])*np.cos(theta[1]))*np.cos(theta[3])
    J[2][4] = (0.24*(1.0*np.sin(theta[1])*np.sin(theta[2]) - 1.0*np.cos(theta[1])*np.cos(theta[2]))*np.sin(theta[3]) + 0.24*(-1.0*np.sin(theta[1])*np.cos(theta[2]) - 1.0*np.sin(theta[2])*np.cos(theta[1]))*np.cos(theta[3]))*np.cos(theta[4]) - (-1.46957615897682e-17*(1.0*np.sin(theta[1])*np.sin(theta[2]) - 1.0*np.cos(theta[1])*np.cos(theta[2]))*np.cos(theta[3]) + 1.46957615897682e-17*(-1.0*np.sin(theta[1])*np.cos(theta[2]) - 1.0*np.sin(theta[2])*np.cos(theta[1]))*np.sin(theta[3]) + 1.46957615897682e-17)*np.sin(theta[4])
    J[2][5] = 0

    return J


def move_position(goal_pose, init_joint):
    """ 
    A function to send a pose goal for generating a straight motion.

    Parameters
    ----------
    goal_pose : Pose
        a Pose object that contain a desired end-effector position 
    init_joint : list
        a list of current joint angles. The joint order must be same as JOINT_NAMES

    Returns
    -------
    q : list
        a list of joint angles after moving the robot.
    """
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    #------------------------------------------------------------
    # ADD YOUR CODE
    #------------------------------------------------------------
    # construct forward kinematics (problem 2)

    # Get start position and joint angles
    theta = np.array(init_joint) # added
    T = forward_kin.get_cum_T(theta)
    x_c = get_xyz_with_theta_T(theta, T) # added, error can be caused

    # Get a position trajectory
    pose_traj = [x_c]
    for i in range(1, 101):
        pose_traj.append([ \
            x_c[0] + (goal_pose.position.x - x_c[0])/100.0*i, \
            x_c[1] + (goal_pose.position.y - x_c[1])/100.0*i, \
            x_c[2] + (goal_pose.position.z - x_c[2])/100.0*i  \
            ])

    # Get a sequence of joint angles that track the position trajecotory by using Jacobian
    q = theta
    dt = 0.05
    time_from_start = 0
    for i in range(1, len(pose_traj)):
        J = get_analytic_jacobian_mat(q)
        Jp = J[:3]
        J_inv = np.dot(Jp.T, np.linalg.inv(np.dot(Jp, Jp.T)))  # filled

        dx = (pose_traj[i] - x_c) # filled
        dtheta = np.dot(J_inv, dx)
        
        q = q + dtheta
        x_c = get_xyz_with_theta_T(q, forward_kin.get_cum_T(q)) # added

        time_from_start = time_from_start + dt
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start))
            )
        if(i%25==0):
            print("{}%: ".format(i), q)
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
    theta = [-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0]
    move_joint(theta)

    T = forward_kin.get_cum_T(theta)
    print forward_kin.your_forward_kinematics(theta, T)
    rospy.sleep(8)


    goal = Pose()
    goal.position.x = 0.487
    goal.position.y = 0.4
    goal.position.z = 0.274

    theta = move_position(goal, theta)
    T = forward_kin.get_cum_T(theta)
    print forward_kin.your_forward_kinematics(theta, T)

