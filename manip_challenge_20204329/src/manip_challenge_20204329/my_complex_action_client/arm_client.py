#!/usr/bin/env python
import sys
import math, numpy as np
from copy import copy, deepcopy
## import collections

import rospy
import actionlib
import threading
import PyKDL
import json

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose, WrenchStamped, Wrench #PointStamped,
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTrajectoryControllerState
# from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import *
from trajectory_msgs.msg import *

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

from complex_action_client import misc, quaternion as qt
from complex_action_client.msg import EndpointState
from complex_action_client.srv import String_None, None_StringResponse
from complex_action_client.srv import None_String, String_NoneResponse
from complex_action_client.srv import String_Int, String_IntResponse

from hrl_geom.pose_converter import PoseConv

QUEUE_SIZE = 10


class ArmClient(object):
 
    def __init__(self, kin_solver='trac_ik', timeout_scale=1.0,
                 sim=False, enable_endstate=False,
                 param_dict={}):
        """
        kin_solver: this decided to use which IK method.
                    'service' will use IK service on UR5.
                    'kdl' will use KDL kinematics.
                    'trac_ik' uses Trac IK.
        timeout_scale: determine the speed scale
                       1.0 uses the specified timeout,
                       0.5 double the specified timeout.
        
        (TODO) Joint name order checker, velocity profile, continuous movement
        Assumption: this arm client assumes spherical wrist joints.
        """

        self._timeout_scale   = timeout_scale
        self._enable_endstate = enable_endstate
        self._sim             = sim

        # Root-TCP Frames for the kinematic chain
        self._arm_base_frame_id = rospy.get_param("arm_base_frame")
        self._ee_frame_id       = rospy.get_param("ee_frame")
        
        # Tool frame offset from a plam frame to a grasping point
        param = rospy.get_param("tool_offset_frame", "[0,0,0,0,0,0]") # for UR5 default
        if type(param) is str: param = eval(param)
        self._tool_offset_frame = misc.list2KDLframe(param) 
        
        # Connect Joint Trajector Action Server
        self._controller_ns   = rospy.get_param("controller_ns", "")
        self._client = actionlib.SimpleActionClient(
            self._controller_ns+"/follow_joint_trajectory",
            FollowJointTrajectoryAction )

        rospy.sleep(1) # need for simulation
        server_up = self._client.wait_for_server(timeout=rospy.Duration.from_sec(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit()
        ## from IPython import embed; embed(); sys.exit()

        self._js_lock = threading.Lock()
        self._js_joint_ids = None
        self._js_joint_name     = None
        self._js_joint_position = None
        self._js_joint_des_position = None
        self._fs_lock = threading.Lock()
        self._fs_force = None
        self._joint_names = None
        self._joint_limit_upper = None
        self._joint_limit_lower = None

        # Goal variables
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Duration.from_sec(0.05)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
                    
        self._initParams()
        self._initComms()
        self._set_fk(kin_solver)
        self._set_ik(kin_solver)

        self._n_joints = len(self._joint_names)       

        if self._enable_endstate:
            self._endpoint_state_pub = rospy.Publisher("EndpointState", EndpointState,
                                                       queue_size=QUEUE_SIZE)        
        
        
        # TODO: Add gripper initialization routines
        self._gripper = None

        # check joint states
        for i in range(10):
            if self.getJointAngles() is not None:
                rospy.logout("ArmClient: joint state is ready")
                break

        # Assign the current joint angle as the current goal 
        cur_positions = self.getDesJointAngles()
        self._clear()
        self._add_point(cur_positions, timeout=1e-6) # for real robot
        self._add_point(cur_positions, timeout=1e-1) # for real robot
        self._client.send_goal(self._goal)
        # rospy.logout('ArmClient: Ready to run the robot')


    def _initParams(self):
        """ Load parameters from ROS server """        
        self._frequency      = 200 #Hz
        self._pos_max_vel    = 0.4 #0.5 #*self._timeout_scale # m/sec
        self._jnt_max_vel    = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi])#*0.6
        self._pos_max_step   = self._pos_max_vel/self._frequency 
        self._jnt_max_step   = self._jnt_max_vel/self._frequency #np.radians(1.0)

        self._pos_deadzon = 0.01
        self._ang_deadzon = 1.*np.pi/180.
        
        rospy.logout('ArmClient: Loaded parameters')


    def _initComms(self):
        """ Initialize ROS publishers/subscribers """
        self._goal_viz_pub  = rospy.Publisher("arm_client/goal_viz",
                                              PoseStamped,
                                              queue_size=QUEUE_SIZE,
                                              latch=True)
        self._traj_viz_pub  = rospy.Publisher("arm_client/traj_viz",
                                              PoseArray,
                                              queue_size=QUEUE_SIZE,
                                              latch=True)

        rospy.Subscriber('/joint_states', JointState, self._joint_states_callback)

        try:
            rospy.Service('arm_client/command', String_Int, self._command_callback)
            rospy.Service('arm_client/status', None_String, self._status_callback)
        except:
            rospy.logwarn("Already registered services")

        if self._enable_endstate:
            self._endpoint_state_pub = rospy.Publisher("EndpointState", EndpointState,
                                                       queue_size=QUEUE_SIZE)        
            
        rospy.logout('ArmClient: Initialized communications')


    #callback function: when a json command message arrives, execute the command
    def _command_callback(self, req):
        cmd_dict = json.loads(req.data)
        action_type = cmd_dict['action_type'].encode('utf-8')
        goal        = cmd_dict.get('goal', None)
        frame       = cmd_dict.get('frame', None)
        timeout     = cmd_dict.get('timeout', 3.)
        no_wait     = cmd_dict.get('no_wait', False)
        chk_contact = cmd_dict.get('check_contact', False)
        rospy.loginfo("Command dict: "+str(req.data))
        
        if action_type in ['cancel_goal']:
            rospy.loginfo("Stop")
            self._client.cancel_goal()
            ret = GoalStatus.SUCCEEDED
            
        elif action_type in ['setSpeed']:
            ret=eval('self.{}({})'.format(action_type, goal))
            
        else:
            if action_type.find('movePose')>=0:
                goal = json.loads(goal)
                goal = Pose(Point(goal['x'], goal['y'], goal['z']),
                            Quaternion(goal['qx'], goal['qy'], goal['qz'], goal['qw']))
            elif action_type.find('moveJoint')>=0:
                d = json.loads(goal)
                goal = []
                for i in range(len(d.keys())):
                    goal.append(d[str(i)])
                    
            ret=eval('self.{}(goal, frame=frame, timeout=timeout, no_wait=no_wait, check_contact=chk_contact)'.format(action_type))

        ## from IPython import embed; embed(); sys.exit()
        return String_IntResponse(ret)

    
    #callback function: when a message arrives, return status json string
    def _status_callback(self, req):
        state = self.get_state()  # return GoalStatus
        ret   = self.get_result() # return error code
        gripper_state = self.getGripperState() # return GoalStatus
        return None_StringResponse( json.dumps({'state': state, 'result': ret, 'gripper_state': gripper_state}) )
        

    #callback function: when a joint_states message arrives, save the values
    def _joint_states_callback(self, msg):
        self._js_lock.acquire()
        self._js_joint_name     = msg.name
        self._js_joint_position = self._js_joint_des_position = msg.position
        self.velocity = msg.velocity
        #self.effort = msg.effort
        self._js_lock.release()


    #callback function: when a joint_states message arrives, save the values
    def _joint_traj_control_state_callback(self, msg):
        self._js_lock.acquire()
        self._js_joint_name         = msg.joint_names
        self._js_joint_position     = msg.actual.positions
        self._js_joint_des_position = msg.desired.positions
        self.velocity = msg.actual.velocities
        #self.effort = msg.effort
        self._js_lock.release()


    #callback function: when a ft sensor message arrives, publish endpoint state message
    def _ft_sensor_callback(self, msg):
        self._fs_lock.acquire()
        #self._fs_force = [msg.Fx, msg.Fy, msg.Fz]
        self._fs_force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self._fs_lock.release()
        
        state = EndpointState()
        state.header = msg.header
        state.header.frame_id = self._ee_frame_id
        state.pose   = self.getEndeffectorPose()

        f = self._ft_2_ee_frame*PyKDL.Vector(self._fs_force[0],
                                             self._fs_force[1],
                                             self._fs_force[2])
        state.wrench.force.x = f[0]
        state.wrench.force.y = f[1]
        state.wrench.force.z = f[2]
        self._endpoint_state_pub.publish(state)
        
        

    def _set_fk(self, fk, ee_frame=None):
        """ Create a kinematics object """
        if fk=='trac_ik' or fk=='kdl':
            from pykdl_utils.kdl_kinematics import create_kdl_kin
            self.arm_kdl = create_kdl_kin(self._arm_base_frame_id, self._ee_frame_id)
        else:
            return NotImplemented

        
    def _set_ik(self, ik):
        """ Create an inverse kinematics object """
        if ik=='trac_ik':
            from trac_ik_python.trac_ik import IK            
            ## self.ik_solver = IK(self._arm_base_frame_id, self._ee_frame_id, solve_type="Speed")
            self.ik_solver = IK(self._arm_base_frame_id, self._ee_frame_id, timeout=0.05, solve_type="Distance")
            self._joint_names = self.ik_solver.joint_names

            if self._joint_limit_upper is None:
                self._joint_limit_upper = self.arm_kdl.joint_limits_upper
            if self._joint_limit_lower is None:
                self._joint_limit_lower = self.arm_kdl.joint_limits_lower
                
            self.ik_solver.set_joint_limits(self._joint_limit_lower, self._joint_limit_upper)
        elif ik=='kdl':
            ## from manipulator_control.kdl_kinematics import create_kdl_kin
            ## self.arm_kdl = create_kdl_kin(self.arm_base_frame_id, self._ee_frame_id)
            self.arm_kdl.joint_safety_lower = self.arm_kdl.joint_limits_lower
            self.arm_kdl.joint_safety_upper = self.arm_kdl.joint_limits_upper
        else:
            return NotImplemented

        
    def fk_request(self, joints, attach_tool=True):
        """ A function to compute forward kinematics. """
        homo_mat = self.arm_kdl.forward(joints)
        pos, quat = PoseConv.to_pos_quat(homo_mat)
        pose = misc.list2Pose(list(pos)+list(quat))
        if attach_tool: return self.attachTool(pose)
        return pose
               
    def ik_request(self, poses, seed_state=None, bx=5e-3, by=5e-3, bz=5e-3, brx=1e-2, bry=1e-2, brz=1e-2):

        if seed_state is None:
            seed_state = self.getJointAngles()
        joint_positions = []

        if type(poses) is not list:
            poses = [poses]

        for i, ps in enumerate(poses):
            # ret: list of the joint angles [theta0, theta1, theta2, ...]
            ret = self.ik_solver.get_ik(seed_state,
                                        ps.position.x,
                                        ps.position.y,
                                        ps.position.z,
                                        ps.orientation.x,
                                        ps.orientation.y,
                                        ps.orientation.z,
                                        ps.orientation.w,
                                        bx=bx, by=by, bz=bz,
                                        brx=brx, bry=bry, brz=brz )
                                        ## bx=5e-3, by=5e-3, bz=5e-3,
                                        ## brx=1e-2, bry=1e-2, brz=1e-2 )
            if ret is None: 
                rospy.logerr("INVALID POSE - No Valid Joint Solution Found in {}th pose.".format(i))
                return False

            # TODO: need to check left/right, elbow up/down config
            joint_positions.append(ret)

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        limb_joints = dict(zip(self._joint_names, np.array(joint_positions).T))
        return limb_joints

    
    # -----------------------------------------------------------------------------------------
    def _add_point(self, positions, velocities=None, timeout=1.0):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        if velocities is not None:
            point.velocities = copy(velocities)
        else:
            point.velocities = [0]*self._n_joints
        ## point.accelerations = [0]*self._n_joints

        point.time_from_start = rospy.Duration.from_sec(timeout)
        
        self._goal.trajectory.points.append(point)
        self._goal.trajectory.header.stamp = rospy.Time.now()

    def _clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names
        
                      
    # -----------------------------------------------------------------------------------------
    def getEndeffectorPose(self, attach_tool=True):
        # {'position': (x, y, z), 'orientation': (x, y, z, w)}}
        joint_state = self.getJointAngles()
        homo_mat = self.arm_kdl.forward(joint_state)
        pos, quat = PoseConv.to_pos_quat(homo_mat)
        pose = misc.list2Pose(list(pos)+list(quat))
        if attach_tool: return self.attachTool(pose)
        else: return pose

    def getEndeffectorDesPose(self, attach_tool=True):
        # {'position': (x, y, z), 'orientation': (x, y, z, w)}}
        joint_state = self.getDesJointAngles()
        homo_mat = self.arm_kdl.forward(joint_state)
        pos, quat = PoseConv.to_pos_quat(homo_mat)
        pose = misc.list2Pose(list(pos)+list(quat))
        if attach_tool: return self.attachTool(pose)
        else: return pose
            
    def getEndeffectorFrame(self):
        return misc.pose2KDLframe(self.getEndeffectorPose())
    
    def getEndeffectorRPY(self, attach_tool=True):
        pose  = self.getEndeffectorPose(attach_tool)
        frame = misc.pose2KDLframe(pose)
        return frame.M.GetRPY()

    def getEndeffectorQuat(self, attach_tool=True):
        pose  = self.getEndeffectorPose(attach_tool)
        frame = misc.pose2KDLframe(pose)
        return frame.M.GetQuaternion()

    def getJointAngles(self):
        while not rospy.is_shutdown():
            if self._joint_names is None: continue
            if self._js_joint_name is None: continue
            if self._js_joint_ids is None or True:
                assert len(self._js_joint_name)>0, "Failed to subscribe joint states"
                self._js_joint_ids = []
                for ns in self._joint_names:
                    for i, joint_name in enumerate(self._js_joint_name):
                        if joint_name.find(ns)>=0:
                            self._js_joint_ids.append(i)
            if len(self._js_joint_ids)!=6: continue
            
            self._js_lock.acquire()
            joint_position = copy(self._js_joint_position)
            self._js_lock.release()
            
            if joint_position is None or len(joint_position)==0 or \
              len(joint_position)<self._n_joints:
                rospy.sleep(0.0)
            else:
                angles = np.array(joint_position)[ self._js_joint_ids ]
                if len(angles)!=6: continue
                return angles

    def getDesJointAngles(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.0)
            if self._joint_names is None: continue
            if self._js_joint_name is None: continue
            if self._js_joint_ids is None or True:
                assert len(self._js_joint_name)>0, "Failed to subscribe joint states"
                self._js_joint_ids = []
                for ns in self._joint_names:
                    for i, joint_name in enumerate(self._js_joint_name):
                        if joint_name.find(ns)>=0:
                            self._js_joint_ids.append(i)
            if len(self._js_joint_ids)!=6: continue
            
            self._js_lock.acquire()
            joint_position = copy(self._js_joint_des_position)
            self._js_lock.release()
            
            if joint_position is None or len(joint_position)==0 or \
              len(joint_position)<self._n_joints:
                rospy.sleep(0.0)
            else:
                angles = np.array(joint_position)[ self._js_joint_ids ]
                if len(angles)!=6:
                    rospy.logerr("len(angles)!=6")
                    continue
                return angles


    def getSpeed(self):
        return self._timeout_scale

    def setSpeed(self, scale):
        """ scale is (0,1.0] """
        self._timeout_scale = scale

    def get_state(self):
        state = self._client.get_state()
        if state is GoalStatus.LOST:
            return GoalStatus.REJECTED
        ## if state is GoalStatus.ABORTED and self._client.get_result().error_code==0:
        ##     state = GoalStatus.SUCCEEDED
        return state
        
    def get_result(self):
        """ Return error code """
        error_code = {
        0: "SUCCESSFUL",
        -1: "INVALID_GOAL",
        -2: "INVALID_JOINTS",
        -3: "OLD_HEADER_TIMESTAMP",
        -4: "PATH_TOLERANCE_VIOLATED",
        -5: "GOAL_TOLERANCE_VIOLATED"}
            
        msg = self._client.get_result()
        if msg is None: return msg

        # NOTE: we ignore the path tolerance since the effort control in simulation
        #       does not produce precise behavior.
        if msg is not None and (msg.error_code==-4 or msg.error_code==-5):
            ## print "------------------"
            ## print msg.error_code
            ## print "we ignore the path tolerance since the effort control in simulation\
            ## does not produce precise behavior"
            ## print "------------------"
            msg.error_code=0
            
        if msg is not None and not (msg.error_code==0):
            #print error_code[msg.error_code]
            print "error_string: ", msg.error_string
            ## print "cacelled a goal"
            ## self._client.cancel_goal()
            ## self.wait_time(5)
        return msg.error_code



    # -----------------------------------------------------------------------------------------
    def gripperOpen(self, timeout=3., check_contact=False):
        return None

    def gripperClose(self):
        return None

    def gripperGotoPos(self, pos, speed=150, force=150, timeout=3., check_contact=False, **kwargs):
        return None

    def getGripperState(self):
        return None
            

        
    # -----------------------------------------------------------------------------------------
    def moveJoint(self, positions, velocities=None, check_contact=False, timeout=2.0,
                  no_wait=False, **kwargs):
        """ Run a point-to-point movement in joint space """
        rospy.logout('ArmClient: moveJoint')
        # To avoid start configuration errors on a real robot, need to repeat twice.
        cur_positions = self.getDesJointAngles()
        
        if velocities is None:
            # Max joint velocity
            diff_ang = abs(np.array(cur_positions)-np.array(positions))
            if max(diff_ang) > np.radians(10.0):
                new_timeout = max(np.divide(diff_ang, self._jnt_max_vel)) #+0.5
                if new_timeout > timeout: timeout = new_timeout

        timeout /= self._timeout_scale

        self._clear()
        self._add_point(cur_positions, timeout=1e-3) # for real robot
        self._add_point(positions, velocities, timeout)

        if check_contact is False:
            if no_wait:
                self._client.send_goal(self._goal)
            else:
                # in simulation, the wait may not work
                self._client.send_goal_and_wait(self._goal, rospy.Duration.from_sec(timeout))
        else:
            self._client.send_goal(self._goal)
            # TODO: need to handle some GoalStatus flags            
            self.wait_contact(timeout, direction=[0,1], ths=4.)

        # If successful, this returns GoalStatus.SUCCEEDED or GoalStatus.PREEMPTED
        return GoalStatus.SUCCEEDED

    
    def moveJointTraj(self, positions, timeout=10., **kwargs):
        """Joint trajectory movement"""
        rospy.logout('ArmClient: moveJointTraj')
        
        filter_size = 5
        jnt_ang     = self.getDesJointAngles()
        jnt_pos_l   = np.array(positions).T        

        # moving average filter and zero velocity padding
        new_jnt_pos_l = []
        for i in xrange(len(self._joint_names)):
            new_jnt_pos = [jnt_ang[i]]*filter_size + list(jnt_pos_l[i]) + [jnt_pos_l[i][-1]]*filter_size*2
            jnt_pos = np.convolve(new_jnt_pos,np.ones((filter_size,))/float(filter_size), mode='valid')
            new_jnt_pos_l.append(jnt_pos)
        new_jnt_pos_l = np.array(new_jnt_pos_l).T

        # TODO: reinterpolate progress...
        new_jnt_pos_len = len(new_jnt_pos_l)
        progress = []
        for i in range(new_jnt_pos_len):
            progress.append(float(i)/float(new_jnt_pos_len-1))
            # progress.append(np.log(i/((new_jnt_pos_len-1)/(np.exp(1)-1)) + 1))
            # progress.append(-float((i-new_jnt_pos_len+1)**2)/((new_jnt_pos_len-1)**2) + 1)
        

        self._clear()
        for i, jnt_pos in enumerate(new_jnt_pos_l):
            self._add_point(jnt_pos, timeout=progress[i]*float(timeout))

        self._client.send_goal_and_wait(self._goal, rospy.Duration.from_sec(timeout))
        return GoalStatus.SUCCEEDED


    def movePoint(self, point, **kwargs):        
        """
        Run a point-to-point movement in cartesian space w/o orientation control        
        """
        rospy.logout('ArmClient: movePoint')
        return NotImplemented

        
    def movePose(self, pose, timeout=1.0, no_wait=False, **kwargs):
        rospy.logout('ArmClient: movePose')
        """
        Run a point-to-point movement in cartesian space
        The reference frame of the pose input is arm_baselink
        """
        ee_ps    = self.detachTool(pose)
        timeout /= self._timeout_scale

        # IK
        bx=5e-3; by=5e-3; bz=5e-3; brx=1e-2; bry=1e-2; brz=1e-2 
        for i in range(10):
            ik_goal = self.ik_request(ee_ps,
                                      bx=bx, by=by, bz=bz,
                                      brx=brx, bry=bry, brz=brz )
            if ik_goal is not False: break
            bx *= 3.; by *= 3.; bz *= 3.
            brx *= 3.; bry *= 3.; brz *= 3.
            
        if ik_goal is False:
            rospy.logerr("Maybe unreachable goal pose... ")
            self._client.stop_tracking_goal() # to avoid returning the result of the previous'goal
            return GoalStatus.REJECTED
            ## return FollowJointTrajectoryResult.INVALID_GOAL
            
        position = [float(ik_goal[joint][0]) for joint in self._joint_names]

        self._clear()
        cur_positions = self.getDesJointAngles()
        self._add_point(cur_positions, timeout=1e-3)
        self._add_point(position, timeout=timeout)
        if no_wait:
            self._client.send_goal(self._goal)
        else:
            self._client.send_goal_and_wait(self._goal, rospy.Duration.from_sec(timeout))
        return GoalStatus.SUCCEEDED
    
                
    def movePoseStraight(self, pose, timeout=2.0, check_contact=False, no_wait=False, **kwargs):
        """ Move the end-effector on a straight-line trajectory """
        return NotImplemented
            

    # filled
    def movePoseTrajectory(self, poses, timeout=5., **kwargs):
        """ Move the end-effector following a pose trajectory """
        return NotImplemented
    

    def movePoseRelative(self, pose, timeout=15.0, frame='ur_arm_base_link', check_contact=False,
                         no_wait=False, **kwargs):
        """ """
        return NotImplemented


    def movePoseRoot(self, pose, timeout=4., no_wait=False, **kwargs):
        """
        Move the first joint toward the pose for pick and place tasks.
        """
        return NotImplemented

    
    # -----------------------------------------------------------------------------------------
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
        tool_frame = misc.pose2KDLframe(pose) * self._tool_offset_frame
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
        ee_frame = misc.pose2KDLframe(pose) * self._tool_offset_frame.Inverse()
        return misc.KDLframe2Pose(ee_frame)


    # -----------------------------------------------------------------------------------------
    def wait_contact(self, timeout, direction=[0,1,2,3,4,5], ths=15.):
        """Check contact. The direction index indicate -x,+x,-y,+y,-z,+z inorder."""
        rospy.loginfo("Start to measure contact force")


        # Get initial force
        rate = rospy.Rate(10)
        count = 0
        f_list = []
        while not rospy.is_shutdown():
            f_list.append( self._fs_force )
            if len(f_list)>5: break
            rate.sleep()
        try:
            f_init = np.mean(f_list, axis=0)
        except:
            print "arm_client:wait_contact"
            print f_list, self._fs_force
            rospy.logwarn("No Contact Force!")
            if self._sim:
                return GoalStatus.SUCCEEDED
            else:
                return GoalStatus.ABORTED
                
            
            
        start_time = rospy.Time.now().secs
        rate = rospy.Rate(30)
        count = 0
        while not rospy.is_shutdown():
            
            # convert end-effector force to base frame
            #ps = self.getEndeffectorPose()
            #frame = misc.pose2KDLframe(ps)
            #f = frame.M.Inverse()*PyKDL.Vector(self._endpoint_force[0],
            #                                   self._endpoint_force[1],
            #                                   self._endpoint_force[2])
            f = self._fs_force
            f = f - f_init
            mag = np.linalg.norm(f)
            rospy.loginfo("{} => {} ".format(f, mag))

            contact_flag = False
            if np.linalg.norm(f) > ths:
                count +=1
                if count > 0:
                    rospy.loginfo("Found contact!")
                    #self._client.cancel_goal()
                    self._client.stop_tracking_goal()
                    contact_flag = True
                    return GoalStatus.ABORTED
                    #break
            #
            # Need to check
            #contact_flag = False
            #for i in direction:
            #    if i%2 == 0: sign = -1.
            #    else: sign = 1.
            #    if sign*f[i/2] > ths:
            #        count +=1
            #        if count>1:
            #            rospy.loginfo("Found contact!")
            #            self._client.cancel_goal()
            #            contact_flag = True
            #            break
                    
            if contact_flag:
                return GoalStatus.SUCCEEDED

            if rospy.Time.now().secs - start_time > timeout*1.3:
                rospy.loginfo("Timeout")
                return GoalStatus.ABORTED

            rate.sleep()


    def wait_time(self, timeout=2.):

        rospy.loginfo("Wait time")
        start_time = rospy.Time.now().secs
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.Time.now().secs-start_time > timeout:
                break
            rate.sleep()

    # -----------------------------------------------------------------------------------------
    def goal_viz(self, pose):
        ps = PoseStamped()
        ps.header.frame_id  = self._arm_base_frame_id
        ps.pose = copy(pose)
        self._goal_viz_pub.publish(ps)

    def traj_viz(self, poses):
        obj = PoseArray()
        obj.header.frame_id = self._arm_base_frame_id
        for ps in poses:
            obj.poses.append(ps)

        self._traj_viz_pub.publish(obj)
            
    # -----------------------------------------------------------------------------------------
    def get_ik_estimate(self, pose):
        rospy.logout('ArmClient: get_ik_estimate')
        """
        Run a point-to-point movement in cartesian space
        The reference frame of the pose input is arm_baselink
        """
        ee_ps    = self.detachTool(pose)

        # IK
        bx=5e-3; by=5e-3; bz=5e-3; brx=1e-2; bry=1e-2; brz=1e-2 
        # ik_goal = self.ik_request(ee_ps,
        #                               bx=bx, by=by, bz=bz,
        #                               brx=brx, bry=bry, brz=brz )
        for i in range(10):
            ik_goal = self.ik_request(ee_ps,
                                      bx=bx, by=by, bz=bz,
                                      brx=brx, bry=bry, brz=brz )
            if ik_goal is False: return -1
            if ik_goal is not False: break
            bx *= 3.; by *= 3.; bz *= 3.
            brx *= 3.; bry *= 3.; brz *= 3.
            
        if ik_goal is False:
            rospy.logerr("Maybe unreachable goal pose... ")
            self._client.stop_tracking_goal() # to avoid returning the result of the previous'goal
            # return GoalStatus.REJECTED
            return -1
            
        position = [float(ik_goal[joint][0]) for joint in self._joint_names]
        return position