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

import collision_check
import rospy, rospkg
import numpy as np
import copy
import os, sys
import threading

import trimesh
import json
from bs4 import BeautifulSoup
import tf

from complex_action_client import misc
from urdf_parser_py import urdf
import urdf_parser_py
import PyKDL
from pykdl_utils.kdl_kinematics import create_kdl_kin
from riro_rviz import draw_scene as ds

from std_msgs.msg import String
from visualization_msgs.msg import Marker

from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient


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
    # print(trans, rot) # trans: xyz position, rot: angle(euler or quaternion)
    return misc.list2KDLframe(trans+rot)


def get_object_pose(target_object, world2base):
    world2obj = get_object_frame(target_object)
    base2obj  = world2base.Inverse() * world2obj

    return misc.KDLframe2Pose(base2obj)


def get_object_grasp_pose(target_object, world2base, direction=2):
    world2obj = get_object_frame(target_object)
    base2obj  = world2base.Inverse() * world2obj

    # move the frame from bottom to center
    vector_a = np.array([base2obj.M.UnitZ()[0], base2obj.M.UnitZ()[1], base2obj.M.UnitZ()[2]])
    obj_pose_l = misc.pose2array(misc.KDLframe2Pose(base2obj))
    obj_pose_l[:3] = obj_pose_l[:3] + vector_a*(obstacles_sizes[target_object][2]/2.)
    base2obj_center = PyKDL.Frame(base2obj.M, PyKDL.Vector( obj_pose_l[0], obj_pose_l[1], obj_pose_l[2]))

    # rotate w.r.t. direction
    if(direction==0):   # positive x axis
        base2obj_center.M.DoRotY(np.pi/2.)
    elif(direction==1): # positive y axis, impossible cuz of the size
        base2obj_center.M.DoRotX(-np.pi/2.)
        base2obj_center.M.DoRotZ(-np.pi/2.)
    elif(direction==2): # positive z axis
        pass
    elif(direction==3): # negative x axis
        base2obj_center.M.DoRotY(-np.pi/2.)
    elif(direction==4): # negative y axis, impossible cuz of the size
        base2obj_center.M.DoRotX(np.pi/2.)
        base2obj_center.M.DoRotZ(np.pi/2.)
    elif(direction==5): # negative z axis
        base2obj_center.M.DoRotY(np.pi)
    vector_a = np.array([base2obj_center.M.UnitZ()[0], base2obj_center.M.UnitZ()[1], base2obj_center.M.UnitZ()[2]])
    obj_pose_l = misc.pose2array(misc.KDLframe2Pose(base2obj_center))
    
    grasp_pose = misc.list2Pose( np.concatenate((obj_pose_l[:3] + vector_a*(obstacles_sizes[target_object][direction%3]/2. - obstacles_sizes[target_object][direction%3]/3.), obj_pose_l[3:])) )
    pre_grasp_pose = misc.list2Pose( np.concatenate((obj_pose_l[:3] + vector_a*(obstacles_sizes[target_object][direction%3]/2. - obstacles_sizes[target_object][direction%3]/3.) + [0., 0., 0.2], obj_pose_l[3:])) )

    return grasp_pose, pre_grasp_pose


def move_position2position(start_position, goal_position, world2base, show_animation=False, dimension=6, timeout=4.):
    if(goal_position==-1):
        print("invalid position")
        return False

    obstacle_list = []
    observation_space_low = [-0.8*np.pi, -0.8*np.pi, 0.3, -0.8*np.pi, -np.pi, -np.pi]
    observation_space_high = [0.8*np.pi,  0.,        np.pi, 0.8*np.pi, np.pi,  np.pi]
    # observation_space_low = [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]
    # observation_space_high = [np.pi,  np.pi,  np.pi,  np.pi,  np.pi,  np.pi]
    grid_limits = [observation_space_low, observation_space_high]
    resolution = 0.01
    show_animation = False
    
    my_rrt = rrt.RRT(
        start_position=start_position,
        goal_position=goal_position,
        obstacle_list=obstacle_list,
        grid_limits=grid_limits,
        arm = arm,
        expand_dis=0.03, # step size
        path_resolution=resolution, # grid size
        goal_sample_rate=20,
        max_iter=1000,
        dimension=dimension,
        animation=show_animation)

    start_time = timeit.default_timer()
    print("finding path...")
    path = my_rrt.planning()
    end_time = timeit.default_timer()
    print("running time: {}...".format(end_time - start_time))
    
    if path is None:
        print("Cannot find path")
        return False

    print("found path!!")
    path.reverse()
    print("len(path): {}".format(len(path)))
    arm.moveJointTraj(path, timeout=timeout)
    return True


if __name__ == '__main__':

    rospy.init_node('arm_client')
    rospy.sleep(1)
    arm = UR5ArmClient(timeout_scale=1., sim=True)
    arm.gripperOpen()
    arm_kdl = create_kdl_kin('base_link', 'gripper_link')
    collision_check_manager = collision_check.CollisionChecker(arm_kdl, viz=True)
    # update a collision manager for objects
    collision_check_manager.update_manager()
    rospy.sleep(1)
    # state =    [-0.56377754507607612, 0.8014983653490225, -1.836142711096006, -1.9609894379612989, -1.7944873665859653, -1.5311642128981138]
    # print( collision_check_manager.in_collision(state) )
    # sys.exit()


    center_state = [0., -1.3543552, 1.10131287, -1.55980649, -1.57114171, -np.pi/2]
    arm.moveJoint(center_state)

    storage_left_center = [0., 0.55, 0.6]
    storage_right_center = [0., -0.55, 0.6]
    step = [0.45*1/3, 0.35*1/3]
    lxs = [ ( storage_left_center[0] + step[0]*(i//3) - step[0] ) for i in range(9) ]
    lys = [ ( storage_left_center[1] + step[1]*(2-i%3) - step[1] ) for i in range(9) ]
    rxs = [ ( storage_right_center[0] + step[0]*(i//3) - step[0] ) for i in range(9) ]
    rys = [ ( storage_right_center[1] + step[1]*(i%3) - step[1] ) for i in range(9) ]
    lidx, ridx = 0, 0

    world2base = get_base_frame()
    world2sl = get_object_frame("storage_left")
    base2sl  = world2base.Inverse() * world2sl
    sl_ps = misc.KDLframe2Pose(base2sl)
    pre_sl_ps = deepcopy(sl_ps)
    pre_sl_ps.position.z += 0.2
    
    world2sr = get_object_frame("storage_right")
    base2sr  = world2base.Inverse() * world2sr
    sr_ps = misc.KDLframe2Pose(base2sr)
    pre_sr_ps = deepcopy(sr_ps)
    pre_sr_ps.position.z += 0.2


    target_object = "snacks"
    storage = "storage_right"
    # grasp_ps, pre_grasp_ps = get_object_grasp_pose(target_object=target_object, world2base=world2base, direction=1)
    
    # state = arm.get_real_ik(grasp_ps)
    # print( collision_check_manager.in_collision(arm.get_real_ik(pre_grasp_ps)) )
    # rospy.sleep(1.)
    # print( collision_check_manager.in_collision(arm.get_real_ik(grasp_ps)) )
    for direction in range(2, 8):
        # if(direction%2==0): continue
        if direction>=6:
            di = direction-6
        else:
            di = direction
        grasp_ps, pre_grasp_ps = get_object_grasp_pose(target_object=target_object, world2base=world2base, direction=di)
        state = arm.get_real_ik(pre_grasp_ps)
        if(state==-1):
            print("invalid grasp pose")
            continue
        else:
            print( collision_check_manager.in_collision(state) )
            rospy.sleep(1.)
        state = arm.get_real_ik(grasp_ps)
        if(state==-1):
            print("invalid pre grasp pose")
            continue
        else:
            print( collision_check_manager.in_collision(state) )
            rospy.sleep(1.)
        

        continue

        flag = move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(pre_grasp_ps), 
                        world2base=world2base, dimension=6, timeout=3.)
        rospy.sleep(3.)

        flag = move_position2position(start_position=arm.getJointAngles(), goal_position=arm.get_real_ik(grasp_ps), 
                        world2base=world2base, dimension=6, timeout=3.)
        if not flag: continue
        rospy.sleep(3.)
        arm.gripperClose()

        flag = move_position2position(start_position=arm.getJointAngles(), goal_position=center_state, 
                        world2base=world2base, dimension=6, timeout=2.)
        break
    # rospy.spin()

   