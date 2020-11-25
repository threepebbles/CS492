#!/usr/bin/env python
import sys, math, numpy as np
import copy
import rospy
import PyKDL
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

QUEUE_SIZE = 10
# objects_names = ['book', 'eraser', 'snacks', 'soap2', 'biscuits', 'glue', 'soap']
object_size_l = {'book':(0.13, 0.03, 0.206), 'eraser':(0.135, 0.06, 0.05), 'snacks': (0.165, 0.06, 0.235), 'soap2':(0.065, 0.04, 0.105), 'biscuits':(0.19, 0.06, 0.15), 'glue':(0.054, 0.032, 0.133), 'soap':(0.14, 0.065, 0.1)}

object_list = []
observation_space_low = [-0.8*np.pi, -np.pi/2., 0.3,      -0.75*np.pi, -np.pi, -np.pi]
observation_space_high = [0.8*np.pi, -0.1,      np.pi/2., -0.4*np.pi,        np.pi,  np.pi]
grid_limits = [observation_space_low, observation_space_high]
resolution = 0.01

center_state = [0., -1.3543552, 1.10131287, -1.55980649, -1.57114171, -np.pi/2]
storage_left_center = [0., 0.55, 0.6]
storage_right_center = [0., -0.55, 0.6]
storage_size = [0.45, 0.35]
step = [storage_size[0]/3, storage_size[1]/3]
lxs = [ ( storage_left_center[0] + step[0]*(i//3) - step[0] - 0.02) for i in range(9) ]
lys = [ ( storage_left_center[1] + step[1]*(2-i%3) - step[1] - 0.02) for i in range(9) ]
rxs = [ ( storage_right_center[0] + step[0]*(i//3) - step[0] - 0.02) for i in range(9) ]
rys = [ ( storage_right_center[1] + step[1]*(i%3) - step[1] + 0.02) for i in range(9) ]
lidx, ridx = 0, 0


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

    # move the frame from bottom to center
    vector_a = np.array([base2obj.M.UnitZ()[0], base2obj.M.UnitZ()[1], base2obj.M.UnitZ()[2]])
    obj_pose_l = misc.pose2array(misc.KDLframe2Pose(base2obj))
    obj_pose_l[:3] = obj_pose_l[:3] + vector_a*(object_size_l[target_object][2]/2.)
    base2obj_center = PyKDL.Frame(base2obj.M, PyKDL.Vector( obj_pose_l[0], obj_pose_l[1], obj_pose_l[2]))

    # rotate w.r.t. direction
    if(direction==0):   # positive x axis
        base2obj_center.M.DoRotY(np.pi/2.)
        if(object_size_l[target_object][(direction+1)%3] > object_size_l[target_object][(direction+2)%3]):
            base2obj_center.M.DoRotZ(np.pi/2.)

    elif(direction==1): # positive y axis, impossible cuz of the size
        base2obj_center.M.DoRotX(-np.pi/2.)
        if(object_size_l[target_object][(direction+1)%3] > object_size_l[target_object][(direction+2)%3]):
            base2obj_center.M.DoRotZ(np.pi/2.)
    elif(direction==2): # positive z axis
        pass
    elif(direction==3): # negative x axis
        base2obj_center.M.DoRotY(-np.pi/2.)
        if(object_size_l[target_object][(direction+1)%3] > object_size_l[target_object][(direction+2)%3]):
            base2obj_center.M.DoRotZ(np.pi/2.)

    elif(direction==4): # negative y axis, impossible cuz of the size
        base2obj_center.M.DoRotX(np.pi/2.)
        if(object_size_l[target_object][(direction+1)%3] > object_size_l[target_object][(direction+2)%3]):
            base2obj_center.M.DoRotZ(np.pi/2.)
    elif(direction==5): # negative z axis
        base2obj_center.M.DoRotY(np.pi)
    vector_a = np.array([base2obj_center.M.UnitZ()[0], base2obj_center.M.UnitZ()[1], base2obj_center.M.UnitZ()[2]])
    obj_pose_l = misc.pose2array(misc.KDLframe2Pose(base2obj_center))
    
    grasp_pose = misc.list2Pose( np.concatenate((obj_pose_l[:3] + vector_a*(object_size_l[target_object][direction%3]/2. - 0.03), obj_pose_l[3:])) )
    pre_grasp_pose = misc.list2Pose( np.concatenate((obj_pose_l[:3] + vector_a*(object_size_l[target_object][direction%3]/2. - 0.03) + [0., 0., 0.15], obj_pose_l[3:])) )

    return grasp_pose, pre_grasp_pose


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


def get_rrt_path_position2position(start_position, goal_position, goal_sample_rate, **kwargs):
    if(goal_position==-1):
        print("invalid position")
        return -1

    my_rrt = rrt.RRT(
        start_position=start_position,
        goal_position=goal_position,
        obstacle_list=object_list,
        grid_limits=grid_limits,
        expand_dis=0.02, # step size
        path_resolution=resolution, # grid size
        goal_sample_rate=goal_sample_rate,
        max_iter=3000,
        dimension=6,
        animation=False,

        contain_gripper=kwargs['contain_gripper'],
        grasping_object=kwargs['grasping_object'])

    path = my_rrt.planning()
    path.reverse()

    if path is None:
        print("Cannot find path")
        return -1

    return path


if __name__ == '__main__':

    rospy.init_node('arm_client')
    rospy.sleep(1)
    arm = UR5ArmClient(timeout_scale=1., sim=True)
    arm.gripperOpen()
    # print(arm.getJointAngles())
    # sys.exit()

    world2base = get_base_frame()
    world2sl = get_object_frame("storage_left")
    base2sl  = world2base.Inverse() * world2sl
    base2sl.M.DoRotZ(np.pi/2)
    sl_ps = misc.KDLframe2Pose(base2sl)
    
    pre_sl_ps = deepcopy(sl_ps)
    
    world2sr = get_object_frame("storage_right")
    base2sr  = world2base.Inverse() * world2sr
    base2sr.M.DoRotZ(np.pi/2)
    sr_ps = misc.KDLframe2Pose(base2sr)
    
    pre_sr_ps = deepcopy(sr_ps)

    start_time = timeit.default_timer()
    # get task commands
    try:
        msg = rospy.wait_for_message("/task_commands", String)
        tsk = json.loads(msg.data)
    except rospy.ServiceException, e:
        print "There are no task commands: %s"%e
        sys.exit()

    # tsk = {"storage_left": [],
    #     "storage_right": ['snacks', 'biscuits', 'glue', 'soap', 'book', 'eraser', 'soap2'] }

    what_storage = {}
    d_base = {}
    for storage, target_objects in tsk.items():
        for target_object in target_objects:
            what_storage[target_object] = storage
            stdi = get_z_align_direction(target_object=target_object, world2base=world2base)
            h = np.zeros(3)
            h[2] = (-1)*(stdi//3)*object_size_l[target_object][stdi%3]/2.
            d_base[target_object] = np.linalg.norm(np.array([0., 0., 0.3]) - misc.pose2array(get_object_pose(target_object, world2base))[:3] + h)        
    sorted_objects = sorted(what_storage.items(), key=lambda x: d_base[x[0]])


    print("order: ", sorted_objects)
    arm.moveJoint(center_state)

    path_traj = []
    start_position = arm.getJointAngles()
    for (target_object, storage) in sorted_objects:
        print("cmd: Moving {} to {}...".format(target_object, storage))

        stdi = get_z_align_direction(target_object=target_object, world2base=world2base)
        if stdi==-1:
            print("I cannot grasp T.T")
            sys.exit()

        for direction in range(stdi, stdi+6):
            if(direction%4==1): continue # impossible cuz of the size
            if direction>=6:
                di = direction-6
            else:
                di = direction
            
            grasp_ps, pre_grasp_ps = get_object_grasp_pose(target_object=target_object, world2base=world2base, direction=di)
            pre_grasp_position = arm.get_ik_estimate(pre_grasp_ps)
            grasp_position = arm.get_ik_estimate(grasp_ps)
            if pre_grasp_position==-1 or grasp_position==-1: continue

            path_traj += get_rrt_path_position2position(start_position, pre_grasp_position, 30, contain_gripper=True, grasping_object=None)

            path_traj += get_rrt_path_position2position(start_position=pre_grasp_position, goal_position=grasp_position, goal_sample_rate=100, contain_gripper=False, grasping_object=None)

            # print("path len: ", len(path_traj))
            arm.moveJointTraj(path_traj, timeout=len(path_traj)*0.021)
            # rospy.sleep(len(path_traj)*0.021)

            arm.gripperClose() # pick
            rospy.sleep(.5)
            break
        
        if(grasp_position==-1 or pre_grasp_position==-1):
            print('There is no way to plan it')
            sys.exit()

        path_traj = get_rrt_path_position2position(start_position=grasp_position, goal_position=pre_grasp_position, goal_sample_rate=100, contain_gripper=True, grasping_object=target_object)
        

        if(storage=='storage_left'):
            pre_sl_ps.position.x = lxs[lidx]
            pre_sl_ps.position.y = lys[lidx]
            pre_sl_ps.position.z = sl_ps.position.z + object_size_l[target_object][di%3] + 0.15

            pre_sl_position = arm.get_ik_estimate(pre_sl_ps)
            path_to_pre_sl_position = get_rrt_path_position2position(start_position=pre_grasp_position, goal_position=pre_sl_position, goal_sample_rate=100, contain_gripper=True, grasping_object=target_object)
            path_traj += path_to_pre_sl_position

            pre_sl_ps.position.z = sl_ps.position.z + object_size_l[target_object][di%3] + 0.03
            
            pre_sl_position2 = arm.get_ik_estimate(pre_sl_ps)
            path_to_pre_sl_position2 = get_rrt_path_position2position(start_position=pre_sl_position, goal_position=pre_sl_position2, goal_sample_rate=100, contain_gripper=True, grasping_object=target_object)
            path_traj += path_to_pre_sl_position2

            arm.moveJointTraj(path_traj, timeout=len(path_traj)*0.021)
            # rospy.sleep(len(path_traj)*0.021)

            arm.gripperOpen() # place

            path_traj = get_rrt_path_position2position(start_position=pre_sl_position2, goal_position=pre_sl_position, goal_sample_rate=100, contain_gripper=False, grasping_object=None)

            start_position = pre_sl_position
            lidx += 1

        elif(storage=='storage_right'):
            pre_sr_ps.position.x = rxs[ridx]
            pre_sr_ps.position.y = rys[ridx]
            pre_sr_ps.position.z = sr_ps.position.z + object_size_l[target_object][di%3] + 0.15

            pre_sr_position = arm.get_ik_estimate(pre_sr_ps)
            path_to_pre_sr_position = get_rrt_path_position2position(start_position=pre_grasp_position, goal_position=pre_sr_position, goal_sample_rate=100, contain_gripper=True, grasping_object=target_object)
            path_traj += path_to_pre_sr_position

            pre_sr_ps.position.z = sr_ps.position.z + object_size_l[target_object][di%3] + 0.03

            pre_sr_position2 = arm.get_ik_estimate(pre_sr_ps)
            path_to_pre_sr_position2 = get_rrt_path_position2position(start_position=pre_sr_position, goal_position=pre_sr_position2, goal_sample_rate=100, contain_gripper=True, grasping_object=target_object)
            path_traj += path_to_pre_sr_position2

            arm.moveJointTraj(path_traj, timeout=len(path_traj)*0.021)
            # rospy.sleep(len(path_traj)*0.021)

            arm.gripperOpen() # place

            path_traj = get_rrt_path_position2position(start_position=pre_sr_position2, goal_position=pre_sr_position, goal_sample_rate=100, contain_gripper=False, grasping_object=None)

            start_position = pre_sr_position
            ridx += 1

    end_time = timeit.default_timer()
    print("running time: {}...".format(end_time - start_time))