#!/usr/bin/env python
import sys, time
import rospy
import numpy as np, math
import actionlib
import threading
import PyKDL

from pykdl_utils.kdl_kinematics import create_kdl_kin
from hrl_geom.pose_converter import PoseConv

from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose

from assignment_1 import misc
import quaternion

import min_jerk as mj


class ArmClient(object):

    def __init__(self, freq=100):
        """A function to initialize parameters and comm. functions."""        
        self.freq = freq #Hz
        
        self.js_lock = threading.Lock()
        self.js_joint_name     = None
        self.js_joint_position = None

        # Contruct forward kinematics
        # Since the robot URDF does not have the user-defined gripper_link,
        # we manually attach the transformation from the robotiq_85_base_link
        # to gripper_link using self.tool_offset_frame.
        self.arm_kdl = create_kdl_kin("base_link", "robotiq_85_base_link")
        self.tool_offset_frame = misc.list2KDLframe([0.15, 0.003, 0,
                                                         1.5707, 0, 1.5707])
        
        self.initComms()
        rospy.loginfo("Initialized UR5")
        
        
    def initComms(self):
        """A function to initialize communication functions."""        
        # Controller
        self.client =\
        actionlib.SimpleActionClient('trajectory_controller/follow_joint_trajectory',
                                         FollowJointTrajectoryAction)
        rospy.loginfo( "Waiting for ur5_arm server..." )
        server_up = self.client.wait_for_server()
        rospy.loginfo( "Connected to ur5_arm server" )

        if not server_up:
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit()

        # Publisher
        self.pose_pub = rospy.Publisher('pose_viz', PoseStamped, queue_size=10,
                                            latch=True)

        # Subscriber
        rospy.Subscriber('/trajectory_controller/state',
                             JointTrajectoryControllerState,
                             self.state_callback)
        while not rospy.is_shutdown():
            if self.js_joint_name is not None: break
            rospy.sleep(0.5)

        # Service

    def state_callback(self, msg):
        """ 
        A callback function that subscribes joint states.

        Parameters
        ----------
        msg : control_msgs/JointTrajectoryControllerState
            a message of desired, actual, and error state of joints
        """
        self.js_lock.acquire()
        self.js_joint_name     = msg.joint_names
        self.js_joint_position = list(msg.desired.positions)
        self.js_lock.release()


    def fk_request(self, joints, attach_tool=True):
        """ 
        A function to compute forward kinematics.

        Parameters
        ----------
        joints : list
            joint angles
        attach_tool : Boolean
            enable to attach offset to the end effector

        Returns
        -------
        pose : geometry_msgs/Pose
            a Pose object (after attaching the tool offset)
        """
        homo_mat = self.arm_kdl.forward(joints)
        pos, quat = PoseConv.to_pos_quat(homo_mat)
        pose = misc.list2Pose(list(pos)+list(quat))
        if attach_tool: return self.attachTool(pose)
        return pose
        

    def attachTool(self, pose):
        """ 
        A function to attach tool offset on the given pose.

        Parameters
        ----------
        pose : geometry_msgs/Pose
            a Pose object

        Returns
        -------
        pose : geometry_msgs/Pose
            a Pose object after attaching the tool offset
        """
        tool_frame = misc.pose2KDLframe(pose) * self.tool_offset_frame
        return misc.KDLframe2Pose(tool_frame)
        
    def detachTool(self, pose):
        """ 
        A function to detach tool offset from the given pose.

        Parameters
        ----------
        pose : geometry_msgs/Pose
            a Pose object

        Returns
        -------
        pose : geometry_msgs/Pose
            a Pose object after detaching the tool offset
        """
        ee_frame = misc.pose2KDLframe(pose) * self.tool_offset_frame.Inverse()
        return misc.KDLframe2Pose(ee_frame)
        
        
    def move_joint(self, angles, duration=3.):
        """ 
        A function that moves joints to the desired angles.

        Parameters
        ----------
        angles : list
            a list of desired joint angles
        duration : float
            time duration
        """
        time, pos, vel, _, _, = \
          mj.min_jerk(self.js_joint_position, angles, duration)

        g                        = FollowJointTrajectoryGoal()
        g.trajectory             = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name
        g.trajectory.points      = []
        # ------------------------------------------------------
        # Place your code here
        # ------------------------------------------------------
        #for ... 
        #    g.trajectory.points.append( 
        #        JointTrajectoryPoint(
        #                             positions= ...,
        #                             velocities= ...,
        #                             time_from_start=rospy.Duration(...))
        #                            )
        # ------------------------------------------------------
        self.client.send_goal(g)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise

        
    def move_position(self, pose, duration=5.):
        """ 
        A function that moves the end effector to a target position 
        without controlling orientation.

        Parameters
        ----------
        pose : Pose
            a target pose of the end effector
        duration : float
            time duration
        """        
        # Get a goal pose
        goal_pose  = pose
        
        # Get a start pose 
        start_pose = self.fk_request(self.js_joint_position, attach_tool=True)

        # Get a sequence of path variables from the min jerk trajectory planning
        time, progress, _, _, _, = mj.min_jerk([0], [1], duration)

        # get a trajectory by interpolating from the start and the goal
        poses = []
        for i, p in enumerate(progress):
            pose = Pose()
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # position
            #pose.position.x = ...
            #pose.position.y = ...
            #pose.position.z = ...
            # ------------------------------------------------------
            poses.append(pose)


        g                        = FollowJointTrajectoryGoal()
        g.trajectory             = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name            
        q                        = self.js_joint_position        
        time_from_start          = 0
        
        for i, t in enumerate(time):
            if i==0: continue
                            
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # get delta position
            # ...
            
            # get a jacobian
            J = self.arm_kdl.jacobian(q)
            J = J[:3]

            # get a pseudo inverse of jacobian mtx
            # ...
            
            
            #q = ... 
            # ------------------------------------------------------
            
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=q,
                                         velocities=[0]*6,
                                         time_from_start=rospy.Duration(t))
                )                    
            
        self.client.send_goal(g)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise


    def move_pose(self, pose, duration=5.):
        """ 
        A function that moves the end effector to a target pose (position+orientation).

        Parameters
        ----------
        pose : Pose
            a target pose of the end effector
        duration : float
            time duration
        """        
        # Get a goal pose
        goal_pose  = pose
        
        # Get a start pose 
        start_pose = self.fk_request(self.js_joint_position, attach_tool=True)

        # Get a sequence of path variables from the min jerk trajectory planning
        time, progress, _, _, _, = mj.min_jerk([0], [1], duration)

        # get a trajectory by interpolating from the start and the goal
        poses = []
        for i, p in enumerate(progress):
            pose = Pose()
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # position
            #pose.position.x = ...
            #pose.position.y = ...
            #pose.position.z = ...
            
            # orientation (You can use the SLERP function in quaternion.py)
            #pose.orientation = ...
            # ------------------------------------------------------
            
            # detach the tool before applying inverse kinematics (if needed)
            pose = self.detachTool(pose)
            poses.append(pose)


        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name            
        q = self.js_joint_position        
        time_from_start = 0
        
        for i, t in enumerate(time):
            if i==0: continue

            prev_frame = misc.pose2KDLframe(poses[i-1])
            frame      = misc.pose2KDLframe(poses[i])
                            
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # get delta position
            # ...

            # get delta orientation
            # ...
                            
            # get a jacobian
            J = self.arm_kdl.jacobian(q)

            # get a pseudo inverse of jacobian mtx
            # ...
            
            
            #q = ...
            # ------------------------------------------------------
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=q,
                                         velocities=[0]*6,
                                         time_from_start=rospy.Duration(t))
                )                    
            
        self.client.send_goal(g)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise

        
    def move_pose_trajectory(self, poses, duration=5.):
        """ 
        A function that moves the end effector following a pose traj.

        Parameters
        ----------
        pose : list
            a list of Pose objects that include position and orientation pairs
        duration : float
            time duration
        """        
        # Get a sequence of path variables from the min jerk trajectory planning
        time, progress, _, _, _, =\
          mj.min_jerk([0], [1], duration, freq=len(poses)/float(duration))

        # Get a trajectory by interpolating from the start and the goal
        poses_new = []
        for i, pose in enumerate(poses):
            p = Pose()            
            # detach the tool before applying inverse kinematics (if needed)
            p = self.detachTool(p)
            poses_new.append(p)

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name            
        q = self.js_joint_position        
        time_from_start = 0
        
        for i, t in enumerate(time):
            if i==0: continue

            prev_frame = misc.pose2KDLframe(poses[i-1])
            frame      = misc.pose2KDLframe(poses[i])
                            
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # get delta position
            # ...
            
            # get delta orientation
            # ...
                            
            # get a jacobian
            J = self.arm_kdl.jacobian(q)

            # get a pseudo inverse of jacobian mtx
            # ...
            
            #q = ...
            # ------------------------------------------------------
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=q,
                                         velocities=[0]*6,
                                         time_from_start=rospy.Duration(t))
                )                    
            
        self.client.send_goal(g)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise

        
    def display_pose(self, pose):
        """ 
        A function that publish a topic to visualize the input pose.

        Parameters
        ----------
        pose : Pose
            a Pose object including a pair of position and orientation
        """        
        ps                 = PoseStamped()
        ps.header.stamp    = rospy.Time.now()
        ps.header.frame_id = 'base_link'
        ps.pose            = pose
        self.pose_pub.publish(ps)
        
        


def problem_1b(arm):
    """Problem 1. B: joint trajectory """       
    arm.move_joint([0,0,0,-1.57,0,0])

    while not rospy.is_shutdown():  
        arm.move_joint([0.4,0,0,-1.57,0,0])
        rospy.sleep(4)
        arm.move_joint([-0.4,0,0,-1.57,0,0])
        rospy.sleep(4)

        
def problem_1c(arm):
    """Problem 1. C: position trajectory """

    arm.move_joint([-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0])
    ## print arm.fk_request(arm.js_joint_position)
        
    goal_pose = Pose()
    goal_pose.position.x = 0.4365
    goal_pose.position.y = 0.2
    goal_pose.position.z = 0.2778
    goal_pose.orientation.x = 0.9287
    goal_pose.orientation.y = 0.3342
    goal_pose.orientation.z = -0.1208
    goal_pose.orientation.w = 0.1054
    arm.move_position(goal_pose, duration=3.)

    
def problem_1d(arm):
    """Problem 1. D: orientation trajectory """

    arm.move_joint([0, -1.57, 1.57, -1.57, -1.57, 0])
    start_pose = arm.fk_request(arm.js_joint_position)

    goal_frame = misc.pose2KDLframe(start_pose)
    goal_frame.M = PyKDL.Rotation.RPY(np.pi/4.,0,0)*goal_frame.M
    goal_pose = misc.KDLframe2Pose(goal_frame)

    arm.move_pose(goal_pose, duration=3.)

    
def problem_1e(arm):
    """Problem 1. E: pose trajectory """

    arm.move_joint([-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0])
    ## print arm.fk_request(arm.js_joint_position)
        
    goal_pose = Pose()
    goal_pose.position.x = 0.4365
    goal_pose.position.y = 0.2
    goal_pose.position.z = 0.2778
    goal_pose.orientation.x = 0.707 
    goal_pose.orientation.y = 0.707 
    goal_pose.orientation.z = 0 
    goal_pose.orientation.w = 0 
    
    arm.move_pose(goal_pose, duration=3.)
    
        
if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    # create action client
    arm = ArmClient()

    # Comment in/out the function you want
    problem_1b(arm)
    problem_1c(arm)
    problem_1d(arm)
    problem_1e(arm)



    

    ## self.display_pose(goal_pose)
    ## rate = rospy.Rate(10) # 10hz
    ## while not rospy.is_shutdown():
    ##     rate.sleep()        
    ## sys.exit()        
    ## from IPython import embed; embed(); sys.exit()
