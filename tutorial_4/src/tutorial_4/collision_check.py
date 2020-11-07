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


class CollisionChecker(object):
    """Collision detection class

    This class provides an arm for object collision detection using
    Trimesh.
    """

    def __init__(self, arm_kdl, viz=False):
        """The constructor"""
        self.arm_kdl         = arm_kdl
        self.viz             = viz
        self.world_model     = None
        self.obj_geom_dict   = {}
        self.robot_geom_dict = {}        
        self._world_lock     = threading.RLock() ## world model lock        
        self.gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH').split(':')
        rospy.Subscriber("world_model", String, self.wm_callback)

        if viz:
            self.dsviz = ds.SceneDraw(frame='world', latch=False)
        
        self.world2baselink  = self.get_baselink_frame()
        self._object_manager = trimesh.collision.CollisionManager()
        self._robot_manager  = trimesh.collision.CollisionManager()
        
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

            ret = self.get_trimesh_object(model)
            if ret is None: continue
            self.obj_geom_dict[model] = ret
                
            geom  = self.obj_geom_dict[model]['geom']                
            state = self.world_model[model]['pose']
            T     = misc.array2KDLframe(state)*self.obj_geom_dict[model]['trans']

            self._object_manager.add_object(model,\
                                            geom,\
                                            misc.KDLframe2Matrix(T))
            rospy.loginfo("collision manager: new object {}".format(model))

        self.robot_geom_dict = self.get_trimesh_arm()
        state = [0,0,0,0,0,0]
        mats = self.arm_kdl.forward_recursive(state)

        for i, mat in enumerate(mats):
            ## if i >= len(state): continue
            link_name = self.arm_kdl.chain.getSegment(i).getName()
            link_T = self.world2baselink * misc.mat2KDLframe(mat)

            if not(link_name in self.robot_geom_dict.keys()): continue
            link_T *= self.robot_geom_dict[link_name]['trans']
            self._robot_manager.add_object(link_name,\
                                           self.robot_geom_dict[link_name]['geom'],\
                                           misc.KDLframe2Matrix(link_T))

            rospy.loginfo("collision manager: new link {}".format(link_name))
                                               

    def update_manager(self):
        """ Update object states in the collision managers """ 
        for i, model in enumerate(self.world_model):
            if model not in self.obj_geom_dict.keys(): continue

            # world model includes the bottom coordinate only
            # visual mesh and collision objects may have different frames. 

            # pose in gazebo (the bottom frame of objects)
            state = self.world_model[model]['pose']
            T     = misc.array2KDLframe(state) * self.obj_geom_dict[model]['trans']

            self._object_manager.set_transform(model,\
                                               misc.KDLframe2Matrix(T))
            print ("Updated {} in the collision manager".format(model))
            
            if self.viz:
                self.rviz_pub(self.obj_geom_dict[model], T,
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
            ## if i >= len(state): continue
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

        return self._robot_manager.in_collision_other(self._object_manager, return_names=True) 


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
        else:
            return rospy.logwarn("Collision Check: {} type is not implemented yet.".format(d['type']))

    

if __name__ == '__main__':
    rospy.init_node("test")
    rospy.sleep(1)

    arm_kdl = create_kdl_kin('base_link', 'gripper_link')
    manager = CollisionChecker(arm_kdl, viz=True)
    ## state = [0, -1.8, 1.0,0,0,0]
    state = [0, 0, 0,-3,-1.57,0]
    
    manager.update_manager()
    rospy.sleep(0.1)
    print manager.in_collision(state)
    rospy.sleep(0.1)
    manager.update_manager()
    rospy.sleep(0.1)
    print manager.in_collision(state)
    rospy.sleep(0.1)
    rospy.spin()
    
    ## from IPython import embed; embed(); sys.exit()
