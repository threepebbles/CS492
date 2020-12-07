#!/usr/bin/env python
import sys, math, numpy as np
import copy
import rospy
import PyKDL
from complex_action_client import misc

from geometry_msgs.msg import Pose
from riro_srvs.srv import String_Pose
from std_msgs.msg import String
import tf

import json
from copy import deepcopy
import time
from collections import deque
from threading import Thread

from my_complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient


object_size_dict = {'book':(0.13, 0.03, 0.206), 'eraser':(0.135, 0.06, 0.05), 'snacks': (0.165, 0.06, 0.235), 'soap2':(0.065, 0.04, 0.105), 'biscuits':(0.19, 0.06, 0.15), 'glue':(0.054, 0.032, 0.133), 'soap':(0.14, 0.065, 0.1)}
observation_space_low  = [-0.8*np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]
observation_space_high = [ 0.8*np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]
grid_limits = [observation_space_low, observation_space_high]

center_state = [0., -1.3543552, 1.10131287, -1.55980649, -1.57114171, -np.pi/2]
storage_left_center = [0., 0.55, 0.6]
storage_right_center = [0., -0.55, 0.6]
storage_size = [0.45, 0.35]
step = [storage_size[0]/3 - 0.03, storage_size[1]/2]
lxs = [ ( storage_left_center[0] - storage_size[0]/3 + step[0]*(i%3) + (0.08*(i//3)) ) for i in range(6) ]
rxs = [ ( storage_right_center[0] - storage_size[0]/3  + step[0]*(i%3) + (0.08*(i//3)) ) for i in range(6) ]
lys = [ ( storage_left_center[1] + storage_size[1]/2 - 0.15 - step[1]*(i//3) ) for i in range(6) ]
rys = [ ( storage_right_center[1] - storage_size[1]/2 + 0.15 + step[1]*(i//3) ) for i in range(6) ]
lidx, ridx = 1, 1

# storage_right_top_position = [-1.8, -1.35, 1.1, -1.56, -1.57, -1.57]
# storage_left_top_position =  [1.5, -1.35, 1.1, -1.56, -1.57, -1.57]
storage_right_top_position = [-1.8, -1.35, 1.1, -1.56, -1.57, +0.3]
storage_right_top_position2 = [-2.06, -1.90, 2.14, -2.01, -1.486, +1.256]
storage_left_top_position =  [1.5, -1.35, 1.1, -1.56, -1.57, -0.3]
storage_left_top_position2 =  [1.2, -1.96, 2.14, -2.01, -1.486, +1.256]

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


def get_z_align_moved_frame(alpha, base2obj):
    # move the frame from bottom to center
    vector_a = np.array([base2obj.M.UnitZ()[0], base2obj.M.UnitZ()[1], base2obj.M.UnitZ()[2]])
    obj_pose_l = misc.pose2array(misc.KDLframe2Pose(base2obj))
    obj_pose_l[:3] = obj_pose_l[:3] + vector_a*alpha
    return PyKDL.Frame(base2obj.M, PyKDL.Vector( obj_pose_l[0], obj_pose_l[1], obj_pose_l[2]))


def get_z_align_direction(target_object, world2base):
    world2obj = get_object_frame(target_object)
    
    worldz = np.array([world2base.M.UnitZ()[0], world2base.M.UnitZ()[1], world2base.M.UnitZ()[2]])

    vector_n = np.array([world2obj.M.UnitX()[0], world2obj.M.UnitX()[1], world2obj.M.UnitX()[2]])
    vector_s = np.array([world2obj.M.UnitY()[0], world2obj.M.UnitY()[1], world2obj.M.UnitY()[2]])
    vector_a = np.array([world2obj.M.UnitZ()[0], world2obj.M.UnitZ()[1], world2obj.M.UnitZ()[2]])
    vs = [vector_n, vector_s, vector_a]

    for i, v in enumerate(vs):
        if abs(np.dot(worldz, v)) <= 1e-2: continue
        
        if np.dot(worldz, v)<=0:
            return i+3
        else:
            return i
    return -1


def get_object_grasp_pose(target_object, world2base, direction=2):
    world2obj = get_object_frame(target_object)
    base2obj  = world2base.Inverse() * world2obj

    # move the frame from bottom to center
    base2obj_center = get_z_align_moved_frame(object_size_dict[target_object][2]/2., base2obj)

    # rotate w.r.t. direction
    if(direction==0):   # positive x axis
        base2obj_center.M.DoRotY(np.pi/2.)
        if(object_size_dict[target_object][(direction+1)%3] > object_size_dict[target_object][(direction+2)%3]):
            base2obj_center.M.DoRotZ(np.pi/2.)
    # elif(direction==1): # positive y axis, impossible cuz of the size
    #     base2obj_center.M.DoRotX(-np.pi/2.)
    #     if(object_size_dict[target_object][(direction+1)%3] > object_size_dict[target_object][(direction+2)%3]):
    #         base2obj_center.M.DoRotZ(np.pi/2.)
    elif(direction==2): # positive z axis
        pass
    elif(direction==3): # negative x axis
        base2obj_center.M.DoRotY(-np.pi/2.)
        if(object_size_dict[target_object][(direction+1)%3] > object_size_dict[target_object][(direction+2)%3]):
            base2obj_center.M.DoRotZ(np.pi/2.)
    # elif(direction==4): # negative y axis, impossible cuz of the size
    #     base2obj_center.M.DoRotX(np.pi/2.)
    #     if(object_size_dict[target_object][(direction+1)%3] > object_size_dict[target_object][(direction+2)%3]):
    #         base2obj_center.M.DoRotZ(np.pi/2.)
    elif(direction==5): # negative z axis
        base2obj_center.M.DoRotY(np.pi)

    grasp_pose = misc.KDLframe2Pose(get_z_align_moved_frame(object_size_dict[target_object][direction%3]/2. - 0.03, base2obj_center))
    
    pre_grasp_pose = misc.KDLframe2Pose(get_z_align_moved_frame(object_size_dict[target_object][direction%3]/2. - 0.03 + 0.15, base2obj_center))

    return grasp_pose, pre_grasp_pose


def move_to_storage(start_position, goal_xyz, arm, target_object, direction, ori):
    path_traj = []

    pose2 = Pose()
    pose2.position.x = goal_xyz[0]
    pose2.position.y = goal_xyz[1]
    pose2.position.z = goal_xyz[2]
    pose2.orientation = ori
    # place position
    place_position = arm.get_ik_estimate(pose2)

    # pre place position
    pre_place_position = deepcopy(place_position)
    pre_place_position[2] -= 0.3

    return pre_place_position, place_position


if __name__ == '__main__':
    rospy.init_node('arm_client')
    rospy.sleep(1)
    arm = UR5ArmClient(timeout_scale=1., sim=True)
    arm.gripperOpen()
    # print(arm.getEndeffectorPose())
    # print(arm.getJointAngles())
    # sys.exit()

    world2base = get_base_frame()
    world2sl = get_object_frame("storage_left")
    base2sl  = world2base.Inverse() * world2sl
    base2sl.M.DoRotZ(np.pi/2)
    sl_ps = misc.KDLframe2Pose(base2sl)
    sl_place_ori = (arm.fk_request(storage_left_top_position)).orientation
    
    world2sr = get_object_frame("storage_right")
    base2sr  = world2base.Inverse() * world2sr
    base2sr.M.DoRotZ(np.pi/2)
    sr_ps = misc.KDLframe2Pose(base2sr)
    sr_place_ori = (arm.fk_request(storage_right_top_position)).orientation

    # get task commands
    try:
        msg = rospy.wait_for_message("/task_commands", String)
        tsk = json.loads(msg.data)
    except rospy.ServiceException, e:
        print "There are no task commands: %s"%e
        sys.exit()

    # start_state = [1.5, -1.8, -1.6, -1.0, 1.5, 0.5]
    # arm.moveJoint(start_state)
    # rospy.sleep(2.)

    what_storage = {}
    d_base = {}
    for storage, target_objects in tsk.items():
        for target_object in target_objects:
            what_storage[target_object] = storage
            stdi = get_z_align_direction(target_object=target_object, world2base=world2base)
            h = np.zeros(3)
            h[2] = (-1)*(stdi//3)*object_size_dict[target_object][stdi%3]/2.
            d_base[target_object] = np.linalg.norm(np.array([0., 0., 1.]) - misc.pose2array(get_object_pose(target_object, world2base))[:3] + h)
    sorted_objects = sorted(what_storage.items(), key=lambda x: d_base[x[0]])

    path_traj = []
    start_position = arm.getJointAngles()
    for idx, (target_object, storage) in enumerate(sorted_objects):
        print("[CMD]: Moving {} to {}...".format(target_object, storage))
        # target_object = "biscuits"
        # storage = "storage_left"

        stdi = get_z_align_direction(target_object=target_object, world2base=world2base)

        if stdi==-1:
            # print("I cannot grasp T.T")
            continue

        rospy.sleep(0.1)
        for direction in range(stdi, stdi+6):
            if(direction%3==1): continue # impossible cuz of the size
            if direction>=6:
                di = direction-6
            else:
                di = direction
            
            grasp_ps, pre_grasp_ps = get_object_grasp_pose(target_object=target_object, world2base=world2base, direction=di)
            pre_grasp_position = arm.get_ik_estimate(pre_grasp_ps)
            grasp_position = arm.get_ik_estimate(grasp_ps)
            if pre_grasp_position==-1 or grasp_position==-1: continue

            arm.moveJoint(grasp_position, timeout=3.)
            arm.gripperClose()
            
            break
        
        if(grasp_position==-1 or pre_grasp_position==-1):
            # print('There is no way to plan it')
            arm.gripperOpen()
            continue

        arm.moveJoint(pre_grasp_position, timeout=1.5)

        if(storage=='storage_left'):
            place_xyz = [lxs[lidx], lys[lidx], sl_ps.position.z+object_size_dict[target_object][di%3]+0.03]

            pre_sl_position, pre_sl_position2 = move_to_storage(start_position=pre_grasp_position, goal_xyz=place_xyz, arm=arm, target_object=target_object, direction=di, ori=sl_place_ori)
            
            arm.moveJoint(pre_sl_position, timeout=3.)
            arm.moveJoint(pre_sl_position2, timeout=4.)
            arm.gripperOpen()
            
            if(idx==6): break
            
            arm.moveJoint(pre_sl_position, timeout=1.5)
            start_position = pre_sl_position
            lidx += 1

        elif(storage=='storage_right'):
            place_xyz = [rxs[ridx], rys[ridx], sr_ps.position.z+object_size_dict[target_object][di%3]+0.03]
            
            pre_sr_position, pre_sr_position2 = move_to_storage(start_position=pre_grasp_position, goal_xyz=place_xyz, arm=arm, target_object=target_object, direction=di, ori=sr_place_ori)
            
            arm.moveJoint(pre_sr_position, timeout=3.)
            arm.moveJoint(pre_sr_position2, timeout=4.)
            arm.gripperOpen()

            if(idx==6): break

            arm.moveJoint(pre_sr_position, timeout=1.5)
            start_position = pre_sr_position
            ridx += 1
