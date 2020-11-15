#!/usr/bin/env python
import sys, math, numpy as np
import copy
import rospy
## import PyKDL
from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient
from complex_action_client import misc, min_jerk, quaternion as qt

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse
from std_msgs.msg import String
import tf

import rrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import json
from copy import deepcopy
import timeit

import pickle, os

QUEUE_SIZE = 10
obstacle_names = ['book', 'eraser', 'snacks', 'soap2', 'biscuits', 'glue', 'soap']
obstacles_sizes = {'book':(0.13, 0.03, 0.206), 'eraser':(0.135, 0.06, 0.05), 'snacks': (0.165, 0.06, 0.235), 'soap2':(0.065, 0.04, 0.105), 'biscuits':(0.19, 0.06, 0.15), 'glue':(0.054, 0.032, 0.133), 'soap':(0.14, 0.065, 0.1)}

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


def get_object_pose(target_object, world2base):
    world2obj = get_object_frame(target_object)
    base2obj  = world2base.Inverse() * world2obj

    return misc.KDLframe2Pose(base2obj)


def get_object_grasp_pose(target_object, world2base, direction=2):
    world2obj = get_object_frame(target_object)
    base2obj  = world2base.Inverse() * world2obj
    
    obj_pose_l = misc.pose2array(misc.KDLframe2Pose(base2obj))

    vector_n = np.array([base2obj.M.UnitX()[0], base2obj.M.UnitX()[1], base2obj.M.UnitX()[2]])
    vector_s = np.array([base2obj.M.UnitY()[0], base2obj.M.UnitY()[1], base2obj.M.UnitY()[2]])    
    vector_a = np.array([base2obj.M.UnitZ()[0], base2obj.M.UnitZ()[1], base2obj.M.UnitZ()[2]])
    unit_vectors = np.array([vector_n, vector_s, vector_a])
    obstacle_position = np.array([base2obj.p.x(), base2obj.p.y(), base2obj.p.z()])

    grasp_pose = misc.list2Pose( np.concatenate((obj_pose_l[:3] + 
        unit_vectors[direction]*(obstacles_sizes[target_object][direction] - 0.04), obj_pose_l[3:])) )
    pre_grasp_pose = misc.list2Pose( np.concatenate((obj_pose_l[:3] + 
        unit_vectors[direction]*(obstacles_sizes[target_object][direction] + 0.12), obj_pose_l[3:])) )

    return grasp_pose, pre_grasp_pose


def move_position2position(start_position, goal_position, world2base, show_animation=False, dimension=6, timeout=4.):
    if(goal_position==-1):
        print("invalid position")
        sys.exit()

    obstacle_list = []
    observation_space_low = [-0.8*np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]
    observation_space_high = [0.8*np.pi, 0., np.pi, np.pi, np.pi, np.pi]
    grid_limits = [observation_space_low, observation_space_high]
    resolution = 0.003
    show_animation = False
    
    my_rrt = rrt.RRT(
        start_position=start_position,
        goal_position=goal_position,
        obstacle_list=obstacle_list,
        grid_limits=grid_limits,
        arm = arm,
        expand_dis=0.005, # step size
        path_resolution=resolution, # grid size
        goal_sample_rate=50,
        max_iter=6000,
        dimension=dimension,
        animation=show_animation)

    start_time = timeit.default_timer()
    path = my_rrt.planning()
    end_time = timeit.default_timer()
    print("running time: {}...".format(end_time - start_time))

    path.reverse()
    print("len(path):", len(path))

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

    arm.moveJointTraj(path, timeout=timeout)


if __name__ == '__main__':

    rospy.init_node('arm_client')
    rospy.sleep(1)
    arm = UR5ArmClient(timeout_scale=1., sim=True)
    arm.gripperOpen()

    # reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    # reset_simulation()

    # get task commands
    try:
        msg = rospy.wait_for_message("/task_commands", String)
        tsk = json.loads(msg.data)
        print(tsk)
    except rospy.ServiceException, e:
        print "There are no task commands: %s"%e
        sys.exit()
    
    world2base = get_base_frame()
    world2sl = get_object_frame("storage_left")
    base2sl  = world2base.Inverse() * world2sl
    sl_ps = misc.KDLframe2Pose(base2sl)
    pre_sl_ps = deepcopy(sl_ps)
    pre_sl_ps.position.z+=0.2
    
    world2sr = get_object_frame("storage_right")
    base2sr  = world2base.Inverse() * world2sr
    sr_ps = misc.KDLframe2Pose(base2sr)
    pre_sr_ps = deepcopy(sr_ps)
    pre_sr_ps.position.z+=0.2

    # arm.moveJoint([1.3770551501789219, -1.434575401913625, 1.2522653950772369, -1.3755392458833133, -1.5621581114491467, 2.1658595873828146], timeout=3.0)
    center_state = [0.15996126, -1.15643552, 1.10131287, -1.55980649, -1.57114171, -0.61828164]
    storage_left_center = [0., 0.55, 0.6]
    storage_right_center = [0., -0.55, 0.6]
    step = [0.45*1/3, 0.35*1/3]

    ############################## task ########################################################
    xs = [ ( storage_left_center[0] + step[0]*(i//3) - step[0] ) for i in range(9) ]
    ys = [ ( storage_left_center[1] + step[1]*(i%3) - step[1] ) for i in range(9) ]
    start_time = timeit.default_timer()
    print(tsk['storage_left'])
    for i, target_object in enumerate(tsk['storage_left']):
        move_position2position(start_position=arm.getJointAngles(), goal_position=center_state, 
                        world2base=world2base, dimension=6, timeout=3.)
        rospy.sleep(3)

        print("Moving {}...".format(target_object))
        grasp_ps, pre_grasp_ps = get_object_grasp_pose(target_object=target_object, world2base=world2base, direction=2)
        move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(pre_grasp_ps), 
                        world2base=world2base, dimension=6, timeout=3.)
        rospy.sleep(3)

        move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(grasp_ps), 
                        world2base=world2base, dimension=6, timeout=2.)
        rospy.sleep(2)
        arm.gripperClose() # pick

        arm.moveJoint(arm.get_real_ik(pre_grasp_ps), timeout=2.)
        rospy.sleep(2)

        move_position2position(start_position=arm.getJointAngles(), goal_position=center_state, 
                        world2base=world2base, dimension=6, timeout=2.)
        rospy.sleep(2.)

        pre_sl_ps.position.x = xs[i]
        pre_sl_ps.position.y = ys[i]
        move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(pre_sl_ps), 
                        world2base=world2base, dimension=6, timeout=3.)
        rospy.sleep(3.)
        arm.gripperOpen() # place

    xs = [ ( storage_right_center[0] + step[0]*(i//3) - step[0] ) for i in range(9) ]
    ys = [ ( storage_right_center[1] + step[1]*(i%3) - step[1] ) for i in range(9) ]
    print(tsk['storage_right'])
    for i, target_object in enumerate(tsk['storage_right']):
        move_position2position(start_position=arm.getJointAngles(), goal_position=center_state, 
                        world2base=world2base, dimension=6, timeout=3.)
        rospy.sleep(3)

        print("Moving {}...".format(target_object))
        grasp_ps, pre_grasp_ps = get_object_grasp_pose(target_object=target_object, world2base=world2base, direction=2)
        move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(pre_grasp_ps), 
                        world2base=world2base, dimension=6, timeout=3.)
        rospy.sleep(3)

        move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(grasp_ps), 
                        world2base=world2base, dimension=6, timeout=2.)
        rospy.sleep(2)
        arm.gripperClose() # pick

        arm.moveJoint(arm.get_real_ik(pre_grasp_ps), timeout=2.)
        rospy.sleep(2)

        move_position2position(start_position=arm.getJointAngles(), goal_position=center_state, 
                        world2base=world2base, dimension=6, timeout=2.)
        rospy.sleep(2.)

        pre_sr_ps.position.x = xs[i]
        pre_sr_ps.position.y = ys[i]
        move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(pre_sr_ps), 
                        world2base=world2base, dimension=6, timeout=3.)
        rospy.sleep(3.)
        arm.gripperOpen() # place
    

    end_time = timeit.default_timer()
    print("running time: {}...".format(end_time - start_time))