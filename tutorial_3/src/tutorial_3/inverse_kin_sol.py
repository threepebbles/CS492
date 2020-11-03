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

    
def move_joint(angles, duration):
    """ 
    A function that moves joints to the desired angles.

    Parameters
    ----------
    angles : list
        a list of desired joint angles
    duration : float
        time duration
    """    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=angles, velocities=[0]*6, time_from_start=duration)]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise


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
    # construct forward kinematics 
    from pykdl_utils.kdl_kinematics import create_kdl_kin
    from assignment_1 import misc
    from hrl_geom.pose_converter import PoseConv
    
    arm_kdl = create_kdl_kin("base_link", "gripper_link")
    ## arm_kdl = create_kdl_kin("base_link", "robotiq_85_base_link")
    #    tool_offset_frame = misc.list2KDLframe([0.15, 0.003, 0, 1.5707, 0, 1.5707])
    
    # Get start position and joint angles
    if init_joint is None:
        return NotImplemented
    else:
        homo_mat = arm_kdl.forward(init_joint)
        pos, quat = PoseConv.to_pos_quat(homo_mat)
        pose = misc.list2Pose(list(pos)+list(quat))
        tool_frame = misc.pose2KDLframe(pose) #* tool_offset_frame
        start_pose = misc.KDLframe2Pose(tool_frame)
        print "start_pose: ", start_pose

    # Get a position trajectory
    diff_pos = np.array([goal_pose.position.x-start_pose.position.x,
                         goal_pose.position.y-start_pose.position.y,
                         goal_pose.position.z-start_pose.position.z])

    # Linear interpolation
    pose_traj = []
    for i, p in enumerate(np.linspace(0,1,100)):
        next_pose = Pose()
        next_pose.position.x = start_pose.position.x + diff_pos[0] * p
        next_pose.position.y = start_pose.position.y + diff_pos[1] * p
        next_pose.position.z = start_pose.position.z + diff_pos[2] * p
        pose_traj.append(next_pose)

        
    # Get a sequence of joint angles that track the position trajecotory
    q = theta
    dt = 0.05
    time_from_start = 0
    for i in range(1, len(pose_traj)):
        # Pseudo inverse
        J = arm_kdl.jacobian(q)
        Jp = J[:3]
        J_inv = Jp.T * np.linalg.inv(Jp*Jp.T)               

        # delta position
        dx = np.array([[pose_traj[i].position.x-pose_traj[i-1].position.x,
                        pose_traj[i].position.y-pose_traj[i-1].position.y,
                        pose_traj[i].position.z-pose_traj[i-1].position.z]]).T
        
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
    theta = [-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0]
    move_joint(theta, rospy.Duration(5.0))

    goal = Pose()
    goal.position.x = 0.457
    goal.position.y = 0.407
    goal.position.z = 0.421
    theta = move_position(goal, theta)


