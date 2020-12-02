#!/usr/bin/env python
import sys
import math, numpy as np
from copy import copy, deepcopy

import rospy
import actionlib
import PyKDL

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose, WrenchStamped, Wrench #PointStamped,
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus
import control_msgs.msg

import misc
from arm_client import ArmClient


class UR5ArmClient(ArmClient):

    def __init__(self, kin_solver='trac_ik', timeout_scale=1.0,
                 sim=False, enable_endstate=True):
        """
        kin_solver: this decided to use which IK method.
                    'service' will use IK service on UR5.
                    'kdl' will use KDL kinematics.
                    'trac_ik' uses Trac IK.
        timeout_scale: determine the speed scale
                       1.0 uses the specified timeout,
                       0.5 double the specified timeout.
        """
        
        # if you don't know the limits, make it None
        ## self._joint_limit_upper = [np.pi, np.pi/2.-0.01, np.pi, np.pi, np.pi, np.pi]
        self._joint_limit_upper = None #[np.pi, np.pi/2.-0.01, np.pi, np.pi, np.pi/4., np.pi]
        ## self._joint_limit_lower = [-np.pi, -np.pi/2.+0.01, -np.pi, -np.pi, -np.pi, -np.pi]
        self._joint_limit_lower = None #[-np.pi, -np.pi/2.+0.01, -0.01, -np.pi, -np.pi+np.pi/4., -np.pi] #for safety
        self._gripper_open_pos = rospy.get_param("gripper_open_pos", 0)
        self._gripper_close_pos = rospy.get_param("gripper_close_pos", 0.8)

        # TODO: Need to remove
        ## self._ft_2_ee_frame  = PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.5, 0.5, -0.5, 0.5),
        ##                                    PyKDL.Vector(0.007, 0., 0.)).Inverse()
        
        super(UR5ArmClient, self).__init__(kin_solver=kin_solver, timeout_scale=timeout_scale,
                                           sim=sim, enable_endstate=enable_endstate )
        
        if self._enable_endstate:
            rospy.Subscriber('robotiq_force_torque_wrench', WrenchStamped,
                             self._ft_sensor_callback)


        rospy.loginfo('ArmClient: Gripper setup')
        rospy.sleep(1)
        # Position controller
        self._gripper = actionlib.SimpleActionClient(
            '/gripper_controller/gripper_cmd',  # namespace of the action topics
            control_msgs.msg.GripperCommandAction # action type
            )
        
        # Wait until the action server has been started and is listening for goals
        rospy.loginfo('ArmClient: Gripper server wait')
        rospy.sleep(1)
        self._gripper.wait_for_server()
        # self.gripperClose()
        
        rospy.loginfo('ArmClient: Ready to run the robot')


    # -----------------------------------------------------------------------------------------
    def gripperOpen(self, timeout=3., check_contact=False):
        # Do not limit the effor by setting force = -1 
        self.gripperGotoPos(self._gripper_open_pos, force=-1, timeout=timeout,
                            check_contact=check_contact)

    def gripperClose(self):
        # Do not limit the effor by setting force = -1 
        self.gripperGotoPos(self._gripper_close_pos, force=-1)
    

    def gripperGotoPos(self, pos, speed=150, force=1, timeout=3.,
                           check_contact=False, **kwargs):
        """
        pos=0: open
        pos=0.8: close
        """
        if check_contact:
            ret = self.wait_contact(timeout, direction=[0,1,2], ths=4.)

        # Position control
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position   = pos   # From 0.0 to 0.8
        goal.command.max_effort = force # 0.1 #-1: Do not limit the effort
        
        self._gripper.send_goal(goal)
        self._gripper.wait_for_result()
        return self.getGripperState()

    
    def getGripperState(self):
        ret = self._gripper.get_state()
        if abs(ret)==GoalStatus.ABORTED or abs(ret)==GoalStatus.REJECTED:
            ret = GoalStatus.SUCCEEDED # ignore the path tolerance error
        return ret

    
    def getGripperPos(self):
        self._js_lock.acquire()
        joint_position = copy(self._js_joint_des_position)
        self._js_lock.release()

        for i, joint_name in enumerate(self._js_joint_name):
            if joint_name.find("robotiq_85_left_knuckle_joint")>=0:
                joint_id = i
                break                            
        angle = np.array(joint_position)[ joint_id ]
        return angle


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--sim', action='store_true', dest='sim',
                 help='use this option if you use simulated ur5')
    p.add_option('--init', action='store_true',
                 help='use this option if you move ur5 to an init pose')
    p.add_option('--timeout_scale', action='store',
                 default=1., help='use this option to specify expected timeout scale')
    p.add_option('--viz', '-v', action='store_true', dest='viz',
                 help='use visualization code for rviz')
    opt, args = p.parse_args()

    rospy.init_node('arm_client')
    rospy.sleep(1)
    arm = UR5ArmClient(timeout_scale=float(opt.timeout_scale), sim=opt.sim)


    if opt.init:
        rospy.sleep(5.0)
        Q = [0,-1.570796,1.570796,-1.570796,-1.570796,0]
        arm.moveJoint(positions=Q, timeout=6.0)
        rospy.sleep(1)
    else:            
        rospy.sleep(1)
        rospy.spin()
    sys.exit()
    
    # ---------------------------------------------------------------------
    # Confirm IK
    # ---------------------------------------------------------------------
    # Get the current joint angle
    print arm.getJointAngles()
    # Get the current endeffector pose
    pose = arm.getEndeffectorPose()
    print pose
    # Get the current joint angle from IK
    pose    = arm.detachTool(pose)
    ik_goal = arm.ik_request(pose)
    print [ik_goal[ns][0] for ns in arm._joint_names]
    # ---------------------------------------------------------------------

    # ---------------------------------------------------------------------
    # Confirm Gripper
    # ---------------------------------------------------------------------
    arm.gripperOpen()
    arm.gripperClose()
    arm.getGripperState()

    # ---------------------------------------------------------------------
    # Confirm Movements
    # ---------------------------------------------------------------------    
    # Move Joints
    print arm.moveJoint([0, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], timeout=10.0, no_wait=False)
    print arm.moveJoint([np.pi/3, 0, 0, -np.pi/2.0, 0, -np.pi/2.0], timeout=10.0, no_wait=False)
    print arm._client.get_state()

    # Move Relative Pose
    pose = Pose(position=Point(x=-0.2, y=0.0, z=0.0))
    arm.movePoseRelative(pose, timeout=3.0, frame='robotiq_85_base_link')
    
    ## print arm._client.get_result()
    ## print arm.getJointAngles()    
    ## print arm.getEndeffectorRPY()

    ## q = PyKDL.Rotation.RPY(-np.pi, np.pi/2., -np.pi).GetQuaternion()
    ## q = PyKDL.Rotation.RPY(0, 0, 0).GetQuaternion()
    ## pose1 = Pose(position=Point(x=0.2, y=0.0, z=0.0),
    ##              orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
    ## arm.movePoseRelative(pose1, timeout=3.0, frame='l_palm')
    
    ## pose1 = Pose(position=Point(x=0.0, y=0.0, z=0.1),
    ##              orientation=Quaternion(x=0, y=0, z=0, w=1))
    ## arm.movePoseRelative(pose1, timeout=3.0, frame='ur_arm_base_link')



    ## print arm.getJointAngles()
    ## print arm.getEndeffectorPose()
    ## # Execute p2p pose
    ## pose1 = Pose(position=Point(x=0.43, y=0.1, z=0.5),
    ##              orientation=Quaternion(x=0., y=0.707, z=0., w=0.707))
    ## arm.movePoseStraight(pose1, timeout=2.0)
    ## ## arm.movePose(pose1, timeout=4.0)
    ## print arm.getEndeffectorPose()
    ## sys.exit()



    
    ## arm.movePose(pose1, timeout=2.0)
    ## print arm.getEndeffectorPose()

    ## while not rospy.is_shutdown():
    ##     arm.movePoseRelative(Pose(position=Point(x=0,y=0,z=-0.2)),1)
    ##     arm.movePoseRelative(Pose(position=Point(x=0,y=0,z=0.2)),1)
    ## sys.exit()
    

    
    ## arm.movePose(pose2, timeout=2.0)
    ## print arm.getEndeffectorPose()


    ## arm.movePoseStraight(pose1, timeout=2.0)
    ## print arm.getEndeffectorPose()


