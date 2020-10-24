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
import quaternion as qt

import min_jerk as mj
# ------------------------------
import matplotlib.pyplot as plt
# ------------------------------

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
        time, pos, vel, _acc, _jerk, = \
          mj.min_jerk(self.js_joint_position, angles, duration)

        g                        = FollowJointTrajectoryGoal()
        g.trajectory             = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name
        g.trajectory.points      = [
            # JointTrajectoryPoint(positions=angles, velocities=[0]*6, time_from_start=rospy.Duration(5.0))
        ]
        # ------------------------------------------------------
        # Place your code here
        # ------------------------------------------------------
        
        for i in range(len(time)):
            g.trajectory.points.append( 
                JointTrajectoryPoint(
                                    positions= pos[i],
                                    velocities= vel[i],
                                    time_from_start=rospy.Duration(time[i]))
                                   )

        # ------------------------------------------------------

        self.client.send_goal(g)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise

        return time, pos, vel, _acc, _jerk
        

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
            pose.position.x = start_pose.position.x + (goal_pose.position.x - start_pose.position.x)*p
            pose.position.y = start_pose.position.y + (goal_pose.position.y - start_pose.position.y)*p
            pose.position.z = start_pose.position.z + (goal_pose.position.z - start_pose.position.z)*p

            # ------------------------------------------------------
            poses.append(pose)


        g                        = FollowJointTrajectoryGoal()
        g.trajectory             = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name            
        q                        = self.js_joint_position        
        time_from_start          = 0

        qs, dqs = [], []
        x_c = start_pose
        for i, t in enumerate(time):
            if i==0: continue
                            
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # get delta position
            # ...
            e_p = np.array([poses[i].position.x - x_c.position.x, poses[i].position.y - x_c.position.y, poses[i].position.z - x_c.position.z])
            
            # get a jacobian
            J = self.arm_kdl.jacobian(q)
            J = np.array(J[:3])
            
            # get a pseudo inverse of jacobian mtx
            J_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))  # filled
           

            dq = np.dot(J_inv, e_p).reshape(-1)
            q = q + dq
            qs.append(q)
            dqs.append(dq/(t-time[i-1]))
            x_c = self.fk_request(q, attach_tool=True)
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

        return time[1:], np.array(qs), np.array(dqs)


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
            pose.position.x = start_pose.position.x + (goal_pose.position.x - start_pose.position.x)*p
            pose.position.y = start_pose.position.y + (goal_pose.position.y - start_pose.position.y)*p
            pose.position.z = start_pose.position.z + (goal_pose.position.z - start_pose.position.z)*p
            
            # orientation (You can use the SLERP function in quaternion.py)
            pose.orientation = qt.slerp(start_pose.orientation, goal_pose.orientation, p)
            # ------------------------------------------------------
            
            # detach the tool before applying inverse kinematics (if needed)
            # pose = self.detachTool(pose)
            poses.append(pose)
        

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name
        q = self.js_joint_position # joint angles
        time_from_start = 0
        
        qs, dqs = [], [] # for plotting
        x_c = start_pose
        for i, t in enumerate(time):
            if i==0: continue
            
            # PyKDL.Frame: [[rotation matrix], [x,y,z]]
            prev_frame = misc.pose2KDLframe(poses[i-1]) # prev_frame.M: current rotation matrix, R
            frame      = misc.pose2KDLframe(poses[i]) # frame.M: desired rotation matrix. R_d
            
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # get delta position, error of position
            e_p = np.squeeze(np.array([poses[i].position.x - x_c.position.x, 
                poses[i].position.y - x_c.position.y, 
                poses[i].position.z - x_c.position.z]))
            
            # get delta orientation, error of orientation
            e_o = np.asarray((frame.M * prev_frame.M.Inverse()).GetEulerZYX())
            
            # get a jacobian
            J = self.arm_kdl.jacobian(q)
            J = np.squeeze(np.asarray(J)) # numpy.matrix to numpy.ndarray

            # get a pseudo inverse of jacobian mtx
            # epsilon = 1e-9
            # J_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + epsilon*np.identity(7)))  # filled
            J_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))  # filled

            dq = np.dot(J_inv, np.hstack([e_p, e_o])).reshape(-1)
            q = q + dq

            qs.append(q)
            dqs.append(dq/(t-time[i-1]))
            x_c = self.fk_request(q, attach_tool=True)
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

        return time[1:], np.array(qs), np.array(dqs)

        
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
            # p = self.detachTool(p)
            poses_new.append(p)

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.js_joint_name            
        q = self.js_joint_position        
        time_from_start = 0
        
        qs, dqs = [], [] # for plotting
        x_c = self.fk_request(self.js_joint_position)
        for i, t in enumerate(time):
            if i==0: continue
            
            # PyKDL.Frame: [[rotation matrix], [x,y,z]]
            prev_frame = misc.pose2KDLframe(poses[i-1]) # prev_frame.M: current rotation matrix, R
            frame      = misc.pose2KDLframe(poses[i]) # frame.M: desired rotation matrix. R_d
            
            # ------------------------------------------------------
            # Place your code here
            # ------------------------------------------------------
            # get delta position, error of position
            e_p = np.squeeze(np.array([poses[i].position.x - x_c.position.x, 
                poses[i].position.y - x_c.position.y, 
                poses[i].position.z - x_c.position.z]))
            
            # get delta orientation, error of orientation
            e_o = np.asarray((frame.M * prev_frame.M.Inverse()).GetEulerZYX())
            
            # get a jacobian
            J = self.arm_kdl.jacobian(q)
            J = np.squeeze(np.asarray(J)) # numpy.matrix to numpy.ndarray

            # get a pseudo inverse of jacobian mtx
            # epsilon = 1e-9
            # J_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + epsilon*np.identity(7)))  # filled
            J_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))  # filled

            dq = np.dot(J_inv, np.hstack([e_p, e_o])).reshape(-1)
            q = q + dq

            qs.append(q)
            dqs.append(dq/(t-time[i-1]))
            x_c = self.fk_request(q, attach_tool=True)
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

        return time[1:], np.array(qs), np.array(dqs)
        

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
    t, pos, vel, acc, jerk = arm.move_joint([0,0,0,-1.57,0,0])

    for idx in range(3):
    # while not rospy.is_shutdown():  
        _t, _pos, _vel, _acc, _jerk = arm.move_joint([0.4,0,0,-1.57,0,0])
        _t = _t + t[-1]
        
        rospy.sleep(3)
        t = np.hstack((t, _t))
        pos = np.vstack((pos, _pos))
        vel = np.vstack((vel, _vel))
        acc = np.vstack((acc, _acc))
        jerk = np.vstack((jerk, _jerk))

        _t, _pose, _vel, _acc, _jerk = arm.move_joint([-0.4,0,0,-1.57,0,0])
        _t = _t + t[-1]
        
        rospy.sleep(3)
        t = np.hstack((t, _t))
        pos = np.vstack((pos, _pos))
        vel = np.vstack((vel, _vel))
        acc = np.vstack((acc, _acc))
        jerk = np.vstack((jerk, _jerk))

    print_plt(t, pos, vel, acc, jerk, num_of_pos=6)
        

def problem_1c(arm):
    """Problem 1. C: position trajectory """
    arm.move_joint([-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0])
    # print arm.fk_request(arm.js_joint_position)
        
    goal_pose = Pose()
    goal_pose.position.x = 0.4365
    goal_pose.position.y = 0.2
    goal_pose.position.z = 0.2778
    goal_pose.orientation.x = 0.9287
    goal_pose.orientation.y = 0.3342
    goal_pose.orientation.z = -0.1208
    goal_pose.orientation.w = 0.1054

    t, qs, dqs = arm.move_position(goal_pose, duration=3.)
    print_plt(t, qs, dqs, num_of_pos=6)

    
def problem_1d(arm):
    """Problem 1. D: orientation trajectory """

    arm.move_joint([0, -1.57, 1.57, -1.57, -1.57, 0])
    start_pose = arm.fk_request(arm.js_joint_position)

    goal_frame = misc.pose2KDLframe(start_pose)
    goal_frame.M = PyKDL.Rotation.RPY(np.pi/4.,0,0)*goal_frame.M
    # goal_frame.M = PyKDL.Rotation.RPY(0,np.pi/4.,0)*goal_frame.M
    # goal_frame.M = PyKDL.Rotation.RPY(0,0,np.pi/4.)*goal_frame.M
    goal_pose = misc.KDLframe2Pose(goal_frame)
    
    t, qs, dqs = arm.move_pose(goal_pose, duration=3.)
    print_plt(t, qs, dqs, num_of_pos=6)
    

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
    
    t, qs, dqs = arm.move_pose(goal_pose, duration=3.)
    print_plt(t, qs, dqs, num_of_pos=6)


y_titles = ["position", "vel", "acc", "jerk"]
y_colors = ["r-", "g-", "b-", "c-"]
def print_plt(X, *Ys, **flag):
    row_num = flag["num_of_pos"] # the number of joints or positions
    
    if row_num == 3:
        x_name = "position"
    elif row_num == 6:
        x_name = "angle of joint"
    else:
        x_name = NotImplemented

    col_num = len(Ys) # the number of differentials
    fig = plt.figure(1)

    for j, Y in enumerate(Ys):
        for i in range(row_num):
            Y = np.array(Y)
            ax = fig.add_subplot(row_num, col_num, i*col_num + j + 1)
            plt.plot(X, Y[:, i], y_colors[j], markersize = 20)
            plt.title("{} of {}[{}]".format(y_titles[j], x_name, i))
        
    plt.subplots_adjust(left = 0.1, 
        bottom=0.1, 
        right=0.9, 
        top=0.9, 
        wspace=0.5, 
        hspace=2)
    plt.show()


# def problem_1e_test(arm):
#     # problem 1d debug
#     myp = Pose()
#     myp.position.x = 1
#     myp.position.y = 6
#     myp.position.z = 3
#     myp.orientation.x = 1 
#     myp.orientation.y = 0
#     myp.orientation.z = 0 
#     myp.orientation.w = 0 
#     myframe = misc.pose2KDLframe(myp)

#     myp2 = Pose()
#     myp2.position.x = 1
#     myp2.position.y = 5
#     myp2.position.z = 3
#     myp2.orientation.x = 0 
#     myp2.orientation.y = 1 
#     myp2.orientation.z = 0 
#     myp2.orientation.w = 0 
#     myframe2 = misc.pose2KDLframe(myp2)
#     print(myframe.M)
#     print(myframe2.M)
#     print(myframe.M * myframe2.M.Inverse())

#     a = np.array([[1,0,0],
#         [0,-1,0],
#         [0,0,-1]])
#     b = np.array([[-1,0,0],
#         [0,1,0],
#         [0,0,-1]])
#     print(np.dot(a,np.linalg.inv(b)))

#     # import operator
#     res = np.asarray(map(operator.sub, myframe.M.GetEulerZYX(), myframe2.M.GetEulerZYX()))
#     print("res:", res)
#     print(res)
#     a = np.array([1, 2, 3])
#     b = np.array([4,5,6,7])
#     print(np.hstack([a, b]))


if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    # create action client
    arm = ArmClient()

    # Comment in/out the function you want
    # problem_1b(arm)

    # problem_1c(arm)

    # problem_1d(arm) # SLERP

    problem_1e(arm) # minimum-jerk trajectory generation function, produce the joint trajectory via inverse kinematics

    ## self.display_pose(goal_pose)
    ## rate = rospy.Rate(10) # 10hz
    ## while not rospy.is_shutdown():
    ##     rate.sleep()        
    ## sys.exit()        
    ## from IPython import embed; embed(); sys.exit()
