#!/usr/bin/env python
import sys, math, numpy as np
import copy
import rospy
## import PyKDL
from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient
from complex_action_client import misc, min_jerk, quaternion as qt

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse

import tf

import rrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

from gazebo_msgs.msg import ModelStates, LinkStates
index_of_model_states = {'ground_plane':0, 'ur5_base':1, 'cafe_table':2, 'cafe_table_left':3, 'cafe_table_right':4, 'storage_left':5, 'storage_right':6, 'robot':7, 'book':8, 'eraser':9, 'snacks':10, 'soap2':11, 'biscuits':12, 'glue':13, 'soap':14}

QUEUE_SIZE = 10

# subscribe once
def get_model_states(timeout=3.):
    try:
        msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout)
        # msg = rospy.wait_for_message('/gazebo/link_states', LinkStates, timeout)
        # print(len(msg.name))
        # for idx, name in enumerate(msg.name):
        #     print('\'{}\': {},'.format(name, idx), end=' ')
        # print(msg.pose)
        # pprint(dir(msg), indent=2)
        # print(msg.pose[index_of_model_states[target_object]])
        
    except ROSException as e:
        rospy.logwarn('get_joint_state timed out after %1.1f s' % timeout)

    return msg


def get_object_frame(target_object):
    """ Return the object top surface frame wrt the world frame. """
    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    try:
        obj_pose = pose_srv_req(target_object).pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    world2obj  = misc.pose2KDLframe(obj_pose)

    return world2obj


def get_object_top_frame(target_object):
    """ Return the object top surface frame wrt the world frame. """
    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    try:
        obj_pose = pose_srv_req(target_object).pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    world2obj  = misc.pose2KDLframe(obj_pose)

    # top offset
    rospy.wait_for_service("get_object_height")
    pose_srv_req = rospy.ServiceProxy("get_object_height", String_Pose)
    
    try:
        obj_top = pose_srv_req(target_object).pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    obj2top  = misc.pose2KDLframe(obj_top)
    
    return world2obj*obj2top


def get_base_frame():
    """ Return the frame from world to base_link """
    # T from world to base_link
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
        break
    return misc.list2KDLframe(trans+rot)
    

def get_storage_frame(target_storage):
    """ Return the object top surface frame wrt the world frame. """
    rospy.wait_for_service("get_object_pose")
    pose_srv_req = rospy.ServiceProxy("get_object_pose", String_Pose)
    
    try:
        obj_pose = pose_srv_req(target_storage).pose
    except rospy.ServiceException, e:
        print "Pose Service is not available: %s"%e

    world2obj  = misc.pose2KDLframe(obj_pose)
    return world2obj


# def get_object_pose(target_object):
#     rospy.wait_for_service('/get_object_pose')
#     pose_srv_req = rospy.ServiceProxy('/get_object_pose', String_Pose)
    
#     # You can send a query to obtain the pose of a selected object
#     obj_pose = pose_srv_req(target_object).pose
#     return obj_pose


if __name__ == '__main__':

    rospy.init_node('arm_client')
    rospy.sleep(1)
    arm = UR5ArmClient(timeout_scale=1., sim=True)

    # ---------------------------------------------------------------------
    # Check the robot status
    # --------------------------------------------------------------------
    # Get the current joint angle
    # print arm.getJointAngles()
    # Get the current endeffector pose
    # print arm.getEndeffectorPose()

    # ---------------------------------------------------------------------
    # Check gripper functions
    # ---------------------------------------------------------------------
    #arm.gripperOpen()
    #arm.gripperClose()
    #arm.getGripperState()

    
    # Move straight
    arm.moveJoint([1.3770551501789219, -1.434575401913625, 1.2522653950772369, -1.3755392458833133, -1.5621581114491467, 2.1658595873828146], timeout=3.0)
    print arm.getEndeffectorPose()
    arm.gripperOpen()

    target_object = 'soap'    
    world2base = get_base_frame()

    # compute grasping pose ----------------------------
    world2obj = get_object_top_frame(target_object)
    base2obj  = world2base.Inverse() * world2obj
    # print(type(world2obj.M.UnitZ()))
    # print(world2obj.M.UnitX(), world2obj.M.UnitY(), world2obj.M.UnitZ())
    # print([world2obj.M.UnitX()[0], world2obj.M.UnitX()[1], world2obj.M.UnitX()[2]])
    # sys.exit()

    # base2obj.M.DoRotY(-np.pi/2.)
    base2obj.p[2] -= 0.04
    grasp_ps = misc.KDLframe2Pose(base2obj)
    # --------------------------------------------------

    # compute a pre-grasping pose -----------------------
    pre_grasp_ps = copy.deepcopy(grasp_ps)
    pre_grasp_ps.position.z += 0.1
    # --------------------------------------------------

    start_pose = arm.getEndeffectorPose()
    goal_pose = pre_grasp_ps
    # print start_pose
    # print goal_pose
    # arm.movePose(goal_pose, timeout=4.0)
    # sys.exit()
    

    ##############################################################################################
    dimension = 3
    start_position = [start_pose.position.x, start_pose.position.y, start_pose.position.z]
    goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
    obstacle_names = ['book', 'eraser', 'snacks', 'soap2', 'biscuits', 'glue', 'soap']
    obstacles_sizes = {'book':(0.13, 0.03, 0.206), 'eraser':(0.135, 0.06, 0.05), 'snacks': (0.165, 0.06, 0.235), 'soap2':(0.065, 0.04, 0.105), 'biscuits':(0.19, 0.06, 0.15), 'glue':(0.054, 0.032, 0.133), 'soap':(0.14, 0.065, 0.1)}
    # print(msg.pose[index_of_model_states[target_object]])
    
    obstacle_list = []
    for name, size in obstacles_sizes.items():
        world2obj = get_object_frame(name)
        base2obj  = world2base.Inverse() * world2obj

        vector_n = np.array([base2obj.M.UnitX()[0], base2obj.M.UnitX()[1], base2obj.M.UnitX()[2]])
        vector_s = np.array([base2obj.M.UnitY()[0], base2obj.M.UnitY()[1], base2obj.M.UnitY()[2]])
        vector_a = np.array([base2obj.M.UnitZ()[0], base2obj.M.UnitZ()[1], base2obj.M.UnitZ()[2]])
        obstacle_position = np.array([base2obj.p.x(), base2obj.p.y(), base2obj.p.z()])

        p1 = obstacle_position - size[0]/2.0*vector_n - size[1]/2.0*vector_s
        p2 = obstacle_position - size[0]/2.0*vector_n + size[1]/2.0*vector_s
        p3 = obstacle_position + size[0]/2.0*vector_n + size[1]/2.0*vector_s
        p4 = obstacle_position + size[0]/2.0*vector_n - size[1]/2.0*vector_s
        p5 = obstacle_position - size[0]/2.0*vector_n - size[1]/2.0*vector_s + size[2]*vector_a
        p6 = obstacle_position - size[0]/2.0*vector_n + size[1]/2.0*vector_s + size[2]*vector_a
        p7 = obstacle_position + size[0]/2.0*vector_n + size[1]/2.0*vector_s + size[2]*vector_a
        p8 = obstacle_position + size[0]/2.0*vector_n - size[1]/2.0*vector_s + size[2]*vector_a

        obstacle_list.append((p1, p2, p3, p4, p5, p6, p7, p8))

    extend_size = 100
    observation_space_low = [-0.1, -0.7, -0.1]
    observation_space_high = [0.8, 0.7, 1.0]

    grid_limits = [observation_space_low, observation_space_high]
    resolution = 0.01
    
    my_rrt = rrt.RRT(
        start_position=start_position,
        goal_position=goal_position,
        obstacle_list=obstacle_list,
        grid_limits=grid_limits,
        arm = arm,
        expand_dis=0.03, # step size
        path_resolution=resolution, # grid size
        goal_sample_rate=5,
        max_iter=500,
        dimension=dimension)

    show_animation = True
    
    path = my_rrt.planning(animation=show_animation)
    path.reverse()
    print("len(path):", len(path))

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            plt.plot([x for (x, y, z) in path], [y for (x, y, z) in path], [z for (x, y, z) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()

    traj_len = len(path)
    pose_traj = []
    for i, pos in enumerate(path):
        cur_p = Pose()
        cur_p.position.x = pos[0]
        cur_p.position.y = pos[1]
        cur_p.position.z = pos[2]

        # orientation (You can use the SLERP function in quaternion.py)
        cur_p.orientation = qt.slerp(start_pose.orientation, goal_pose.orientation, float((i+1.)/traj_len))
        # ------------------------------------------------------
        pose_traj.append(cur_p)

        # print (pos, [cur_p.orientation.x, cur_p.orientation.y, cur_p.orientation.z, cur_p.orientation.w])

    arm.movePoseTrajectory(pose_traj, timeout=2.)
    print arm.getEndeffectorPose()
    ##################################################################################3
    # pose = Pose(position=Point(x=0.01, y=0.4, z=0.33))
    # arm.movePoseStraight(pose, timeout=3.0, frame='robotiq_85_base_link')
    # print arm.getEndeffectorPose()

    
    # Move pose trajectory
    # alpha = np.random.randint(1., 4)/20.0
    # print(alpha)
    # goal_pose = Pose(position=Point(x=0.01, y=0.5, z=0.3))
    # current_pose = arm.getEndeffectorPose()
    # poses = []
    
    # for i in range(1, 3):
    #     pose = [(goal_pose.position.x-current_pose.position.x)/100.0*i,
    #     (goal_pose.position.y-current_pose.position.y)/100.0*i,
    #     (goal_pose.position.z-current_pose.position.z)/100.0*i]
    #     poses.append(Pose(position=Point(x=pose[0], y=pose[1], z=pose[2])))
    # arm.movePoseStraight(goal_pose, timeout=3.0, frame='robotiq_85_base_link')
    # arm.movePoseTrajectory(poses, timeout=3.0, frame='robotiq_85_base_link')


    # Move following a joint trajectory
    # current_angle = arm.getJointAngles()
    # arm.moveJointTraj([[please, add joints],[add joints], ..., [add goal joints]], timeout=10.0)
    
    
    
    
