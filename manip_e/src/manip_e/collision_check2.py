#!/usr/bin/env python
"""
Copyright 2020 Daehyung Park

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

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

from my_complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient

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

storage_left_center = [0., 0.55, 0.6]
storage_right_center = [0., -0.55, 0.6]
storage_size = [0.45, 0.35]
step = [storage_size[0]/3-0.01, storage_size[1]/2+0.015]
lxs = [ ( storage_left_center[0] - storage_size[0]/3 + step[0]*(i%3) ) for i in range(6) ]
rxs = [ ( storage_right_center[0] - storage_size[0]/3  + step[0]*(i%3) ) for i in range(6) ]
lys = [ ( storage_left_center[1] + storage_size[1]/2 - 0.1 - step[1]*(i//3) ) for i in range(6) ]
rys = [ ( storage_right_center[1] - storage_size[1]/2 + 0.1 + step[1]*(i//3) ) for i in range(6) ]
lidx, ridx = 0, 0

# storage_right_top_position = [-1.8, -1.35, 1.1, -1.56, -1.57, -1.57]
# storage_left_top_position =  [1.5, -1.35, 1.1, -1.56, -1.57, -1.57]
storage_right_top_position = [-1.8, -1.35, 1.1, -1.56, -1.57, +0.3]
storage_left_top_position =  [1.5, -1.35, 1.1, -1.56, -1.57, -0.3]


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

object_size_l = {'book':(0.13, 0.03, 0.206), 'eraser':(0.135, 0.06, 0.05), 'snacks': (0.165, 0.06, 0.235), 'soap2':(0.065, 0.04, 0.105), 'biscuits':(0.19, 0.06, 0.15), 'glue':(0.054, 0.032, 0.133), 'soap':(0.14, 0.065, 0.1)}


class CollisionChecker(object):
    """Collision detection class

    This class provides an arm for object collision detection using
    Trimesh.
    """

    def __init__(self, arm_kdl, contain_gripper=False, grasping_object=None, grasping_direction=2,viz=True):
        """The constructor"""
        self.arm_kdl         = arm_kdl
        self.viz             = viz
        self.world_model     = None
        self.obj_geom_dict   = {}
        self.table_geom_dict = {}
        self.robot_geom_dict = {}        
        self._world_lock     = threading.RLock() ## world model lock        
        self.gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH').split(':')
        rospy.Subscriber("world_model", String, self.wm_callback)

        if viz:
            self.dsviz = ds.SceneDraw(frame='world', latch=False)
        
        self.world2baselink  = self.get_baselink_frame()
        self._object_manager = trimesh.collision.CollisionManager()
        self._table_manager = trimesh.collision.CollisionManager()
        self._robot_manager  = trimesh.collision.CollisionManager()

        self.contain_gripper = contain_gripper
        self.grasping_object = grasping_object
        self.grasping_direction = grasping_direction

        # for collision check
        self.init_manager()

    def wm_callback(self, msg):
        """ Subscribe the state of world model """
        with self._world_lock:
            self.world_model = json.loads(msg.data)

    def get_baselink_frame(self):
        """ Get the transformation from world to base_link """
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

    def init_manager(self):
        """ Initialize collision managers """
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.world_model is not None: break
            rate.sleep()

        for model in self.world_model:
            if model.find('ur5')>=0: continue
            if model.find('ground')>=0: continue

            if("box" in model):
                ret = self.get_trimesh_object("box")
                if ret is None: continue
                self.obj_geom_dict[model] = ret
                    
                geom  = self.obj_geom_dict[model]['geom']                
                state = self.world_model[model]['pose']
                T     = misc.array2KDLframe(state)*self.obj_geom_dict[model]['trans']

                self._object_manager.add_object(model,\
                                                geom,\
                                                misc.KDLframe2Matrix(T))
                rospy.loginfo("_object_manager: new object {}".format(model))

            else:
                ret = self.get_trimesh_object(model)
                if ret is None: continue
                self.obj_geom_dict[model] = ret
                    
                geom  = self.obj_geom_dict[model]['geom']                
                state = self.world_model[model]['pose']
                T     = misc.array2KDLframe(state)*self.obj_geom_dict[model]['trans']

                self._object_manager.add_object(model,\
                                                geom,\
                                                misc.KDLframe2Matrix(T))
                rospy.loginfo("_object_manager: new object {}".format(model))

        # table objects
        tables = ['storage_left', 'storage_right', 
        'cafe_table', 'cafe_table_right', 'cafe_table_left', 
        'ur5_base']
        poses = [[0.0, 0, 0, 0.0, -0.0, 0.0], [0.0, 0, 0, 0.0, -0.0, 0.0], 
                [0.0, 0.0, 0.755, 0.0, -0.0, 0.0], [0.0, 0, 0.58, 0.0, -0.0, 0.0], [0.0, 0, 0.58, 0.0, -0.0, 0.0], 
                [0.0, 0.0, 0.58, 0.0, -0.0, 0.0]]
        sizes = [[0.45, 0.35, 0.005], [0.45, 0.35, 0.005], 
        [0.913, 0.913, 0.04], [0.5, 0.5, 0.02], [0.5, 0.5, 0.02],
        [0.5, 0.5, 0.02]]
        for model, pose, size in zip(tables, poses, sizes):
            T     = misc.array2KDLframe(pose)
            ret = {'type': 'box',
                    'geom': trimesh.creation.box(size),
                    'trans': T,
                    'size': size}

            self.table_geom_dict[model] = ret
                
            geom  = self.table_geom_dict[model]['geom']                
            state = pose
            T     = misc.array2KDLframe(state)*self.table_geom_dict[model]['trans']

            self._table_manager.add_object(model,\
                                            geom,\
                                            misc.KDLframe2Matrix(T))
            rospy.loginfo("_table_manager: new object {}".format(model))

        # links of the robot
        self.robot_geom_dict = self.get_trimesh_arm()
        state = [0,0,0,0,0,0]
        mats = self.arm_kdl.forward_recursive(state)
        # print("robot_geom_dict:", self.robot_geom_dict.keys())
        
        for i, mat in enumerate(mats):
            ## if i >= len(state): continue
            link_name = self.arm_kdl.chain.getSegment(i).getName()
            # print("i:{}, name:{}".format(i, link_name))
            link_T = self.world2baselink * misc.mat2KDLframe(mat)
            # print(type(link_T))
            if not(link_name in self.robot_geom_dict.keys()): continue
            link_T *= self.robot_geom_dict[link_name]['trans']
            self._robot_manager.add_object(link_name,\
                                           self.robot_geom_dict[link_name]['geom'],\
                                           misc.KDLframe2Matrix(link_T))

            rospy.loginfo("_robot_manager: new link {}".format(link_name))
        
        if(self.contain_gripper):

            # imaginery gripper_link for rough collision check
            link_name = "gripper_link" # gripper
            tmp_T = self.robot_geom_dict['robotiq_coupler']['trans']
            vector_a = np.array([tmp_T.M.UnitZ()[0], tmp_T.M.UnitZ()[1], tmp_T.M.UnitZ()[2]])
            pose_l = misc.pose2array(misc.KDLframe2Pose(tmp_T))
            pose_l[:3] = pose_l[:3] + vector_a*0.1
            link_T = PyKDL.Frame(tmp_T.M, PyKDL.Vector( pose_l[0], pose_l[1], pose_l[2]))

            # size = [diameter along x axis, diameter along y axis, height]
            size = [0.1554, 0.1554, 0.2]
            ret = {'type': 'cylinder',
                'geom': trimesh.creation.cylinder(radius=0.0777, height=0.2),
                'trans': link_T,
                'size': size}

            self.robot_geom_dict[link_name] = ret
            self._robot_manager.add_object(link_name,\
                                           ret['geom'],\
                                           misc.KDLframe2Matrix(ret['trans']))
            rospy.loginfo("collision manager: new link {}".format(link_name))
                                               

    def update_manager(self):
        """ Update object states in the collision managers """ 
        for i, model in enumerate(self.world_model):
            if (model in self.obj_geom_dict.keys()): 

                # world model includes the bottom coordinate only
                # visual mesh and collision objects may have different frames. 

                # pose in gazebo (the bottom frame of objects)
                state = self.world_model[model]['pose']
                T     = misc.array2KDLframe(state) * self.obj_geom_dict[model]['trans']

                self._object_manager.set_transform(model,\
                                                   misc.KDLframe2Matrix(T))
                
                if self.viz:
                    self.rviz_pub(self.obj_geom_dict[model], T,
                                  color=[0,1.,0,0.5],
                                  num_id=100+i)

            elif (model in self.table_geom_dict.keys()): 

                # world model includes the bottom coordinate only
                # visual mesh and collision objects may have different frames. 

                # pose in gazebo (the bottom frame of objects)
                state = self.world_model[model]['pose']
                T     = misc.array2KDLframe(state) * self.table_geom_dict[model]['trans']

                self._table_manager.set_transform(model,\
                                                   misc.KDLframe2Matrix(T))
                
                if self.viz:
                    self.rviz_pub(self.table_geom_dict[model], T,
                                  color=[0,1.,0,0.5],
                                  num_id=100+i)

                
    def get_trimesh_object(self, name):
        """ Get an object dictionary that includes geometries and poses  """ 
        
        for model_path in self.gazebo_model_path:
            if model_path == '': continue
            for model_name in os.listdir(model_path):
                if name == model_name:
                    sdf = os.path.join(model_path, model_name, 'model.sdf')

                    file_handle = open(sdf, 'r')
                    file_contents = file_handle.read()
                    file_handle.close()                    
                    soup = BeautifulSoup(file_contents, 'xml')

                    collision_xmls = soup.find_all('collision')
                    if collision_xmls == [] or collision_xmls is None: continue
                    for collision_xml in collision_xmls:
                        box_size = collision_xml.find_all('box')
                        if box_size == []: continue
                        if collision_xml.pose is None:
                            pose = [0,0,0,0,0,0]
                        else:
                            pose = [float(i) for i in collision_xml.pose.text.split(' ')]
                        size = [float(i) for i in box_size[0].size.text.split(' ')]

                        T     = misc.array2KDLframe(pose)
                        return {'type': 'box',
                                'geom': trimesh.creation.box(size),
                                'trans': T,
                                'size': size}
                    
        return None

    
    def get_trimesh_arm(self):
        """ Get an arm object dictionary that includes geometries and poses  """ 
        rospack = rospkg.RosPack()    
        _urdf = urdf.Robot()
        description = _urdf.from_parameter_server("robot_description")
        robot_geom_dict = {}
        for link in description.links:
            if link.collision is None: continue
            ## if 'filename' in link.collision.geometry.keys():            
            if type(link.collision.geometry) is urdf_parser_py.urdf.Mesh:
                if link.collision.origin is None:
                    T = PyKDL.Frame()
                else:
                    pose  = link.collision.origin.xyz + link.collision.origin.rpy
                    T     = misc.array2KDLframe(pose)

                filename = link.collision.geometry.filename
                pkg  = filename.split('//')[1].split('/')[0]
                mesh = trimesh.load_mesh(os.path.join(rospack.get_path(pkg),
                                                          filename.split(pkg)[1][1:]))
                robot_geom_dict[link.name]\
                = {'type': 'mesh',
                   'geom': mesh,
                   'trans': T,
                   'mesh_filename': filename}
            elif type(link.collision.geometry) is urdf_parser_py.urdf.Box:
                if link.collision.origin is None:
                    T = PyKDL.Frame()
                else:
                    pose  = link.collision.origin.xyz + link.collision.origin.rpy
                    T     = misc.array2KDLframe(pose)
                mesh = trimesh.creation.box(link.collision.geometry.size)
                robot_geom_dict[link.name] \
                  = {'type': 'box',
                     'geom': mesh,
                     'trans': T,
                     'size': link.collision.geometry.size}

            else:
                rospy.logwarn("Collision Check: {} type is not implemented yet.".format(link.name))

        return robot_geom_dict
    

    def in_collision(self, state):
        """ Check if there is collision between an arm and environment. """

        mats = self.arm_kdl.forward_recursive(state)
        for i, mat in enumerate(mats):
            link_name = self.arm_kdl.chain.getSegment(i).getName()
            link_T = self.world2baselink * misc.mat2KDLframe(mat)

            if not(link_name in self.robot_geom_dict.keys()): continue
            link_T *= self.robot_geom_dict[link_name]['trans']

            self._robot_manager.set_transform(link_name,
                                              misc.KDLframe2Matrix(link_T))

            if self.viz:
                self.rviz_pub(self.robot_geom_dict[link_name],
                              link_T,
                              color=[1.,0,0,0.5],    
                              num_id=10+i)

        if(self.grasping_object is not None):
            link_T *= self.obj_geom_dict[self.grasping_object]['trans']
            vector_a = np.array([link_T.M.UnitZ()[0], link_T.M.UnitZ()[1], link_T.M.UnitZ()[2]])
            obj_pose_l = misc.pose2array(misc.KDLframe2Pose(link_T))
            grasp_pose = misc.list2Pose( np.concatenate((obj_pose_l[:3] - (object_size_l[self.grasping_object][self.grasping_direction%3]/2. - 0.03 + 0.18)*vector_a, obj_pose_l[3:])) )

            link_T = misc.pose2KDLframe(grasp_pose)
            if(self.grasping_direction%3==0):
                link_T.M.DoRotY(np.pi/2.)
            elif(self.grasping_direction%3==1):
                link_T.M.DoRotX(np.pi/2.)

            self._object_manager.set_transform(self.grasping_object,
                                              misc.KDLframe2Matrix(link_T))
            i+=1
            if self.viz:
                self.rviz_pub(self.obj_geom_dict[self.grasping_object],
                              link_T,
                              color=[1.,0,0,0.5],    
                              num_id=10+i)


            f1, cs1 = self._robot_manager.in_collision_other(self._object_manager, return_names=True)
            f2, cs2 = self._robot_manager.in_collision_other(self._table_manager, return_names=True)

            # (collision between robot arm and objects, collision between the grasping object and the other objects)
            return ( (f1 or f2, cs1|cs2), 
                (self._object_manager.in_collision_internal(return_names=True)) )
        else:
            f1, cs1 = self._robot_manager.in_collision_other(self._object_manager, return_names=True)
            f2, cs2 = self._robot_manager.in_collision_other(self._table_manager, return_names=True)
            
            return (f1 or f2, cs1|cs2)

    def rviz_pub(self, d, T, color=[1,0,0,0.5], num_id=10):
        """ Publish a visualization msg of an object """
        if d['type'] == 'mesh':
            l = misc.KDLframe2List(T)
            self.dsviz.pub_mesh(l[:3], l[3:],
                                [1,1,1],
                                color,
                                num_id,
                                d['mesh_filename'])
        elif d['type'] == 'box':
            l = misc.KDLframe2List(T)
            self.dsviz.pub_body(l[:3], l[3:],
                                d['size'],
                                color,
                                num_id,
                                Marker.CUBE)
        elif d['type'] == 'cylinder':
            l = misc.KDLframe2List(T)
            self.dsviz.pub_body(l[:3], l[3:],
                                d['size'],
                                color,
                                num_id,
                                Marker.CYLINDER)
        else:
            return rospy.logwarn("Collision Check: {} type is not implemented yet.".format(d['type']))


if __name__ == '__main__':
    rospy.init_node("collision_check_node")
    rospy.sleep(1)


    arm = UR5ArmClient(timeout_scale=1., sim=True)
    # # print(arm.getJointAngles())
    # # sys.exit()

    # # storage_right_top_position2 = [1.3, -1.9, 2.1, -2.0, -1.50, +1.34]

    # storage_right_top_position2 = [-2.06, -1.96, 2.14, -2.01, -1.486, +1.256]
    # # storage_right_top_position2 = [1.2, -1.96, 2.14, -2.01, -1.486, +1.256]
    # arm.moveJoint(storage_right_top_position2)
    
    # goal_pose = arm.fk_request(storage_right_top_position2)

    # ori = (arm.fk_request(storage_right_top_position2)).orientation
    # pose1 = Pose()
    # pose1.position.x = goal_pose.position.x
    # pose1.position.y = goal_pose.position.y
    
    # pose1.position.z = goal_pose.position.z + 0.12
    # pose1.orientation = ori
    # pre_storage_position = arm.get_ik_estimate(pose1)

    # # # storage_right_top_position2 = [-1.8, -1.8, 2.1, -2.1, -1.57, +np.pi/2]
    # arm.moveJoint(pre_storage_position)
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

    # start_state = [1.5, -1.8, -1.6, -1.0, 1.5, 0.5]
    # arm.moveJoint(start_state)
    # rospy.sleep(2.)

    # get task commands
    try:
        msg = rospy.wait_for_message("/task_commands", String)
        tsk = json.loads(msg.data)
    except rospy.ServiceException, e:
        print "There are no task commands: %s"%e
        sys.exit()


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

    arm.gripperOpen()
    arm_kdl = create_kdl_kin('base_link', 'gripper_link')
    
    for idx, (target_object, storage) in enumerate(sorted_objects):
        print("target: ", target_object)
        stdi = get_z_align_direction(target_object=target_object, world2base=world2base)

        if stdi==-1:
            print("I cannot grasp T.T")
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

            collision_check_manager = CollisionChecker(arm_kdl, contain_gripper=True, grasping_object=None, grasping_direction=None, viz=True)
            collision_check_manager.update_manager()
            f, cs = collision_check_manager.in_collision(pre_grasp_position)
            rospy.sleep(2.)
            print(f, cs)
            break