#!/usr/bin/env python  
import os, sys
import json, simplejson
import rospy, rospkg
import xacro
import numpy as np

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse
import misc

rospack = rospkg.RosPack()
data_path = os.path.join(rospack.get_path('world_model_data'), 'data')
xacro_path = os.path.join(data_path, 'models/xacro')
sdf_path = os.path.join(rospack.get_path('manip_challenge'), 'data', 'models')

def add_storage(obj_name="storage1", obj_type="basket",
                    length=0.39, width=0.42, depth=0.02,
                    pos=[0,0,0], orient=[0,0,0,1], color="WoodPallet"):
    """
    add a tray object to GAZEBO
    """
    xacro_type  = obj_type
    object_name = obj_name
    object_pose = Pose()
    object_pose.position.x = pos[0]
    object_pose.position.y = pos[1]
    object_pose.position.z = pos[2]
    object_pose.orientation.x = orient[0] 
    object_pose.orientation.y = orient[1]
    object_pose.orientation.z = orient[2]
    object_pose.orientation.w = orient[3]

    # get a target object xml
    xacro_file = os.path.join(xacro_path, xacro_type+'.urdf.xacro')

    arg_list = []
    arg_list.append('name:='+object_name)
    arg_list.append('color:=' + str(color))
    arg_list.append('length:='+str(length))
    arg_list.append('width:='+str(width))
    arg_list.append('depth:='+str(depth))
    arg_list.append(xacro_file)
    
    spawn_urdf_object(arg_list, object_name, object_pose)
    return

def add_cube(obj_name='o4', obj_type='cube',
                pos=[0.92,-0.3,0], color='White'):
    """
    add a cube object to GAZEBO
    """
    xacro_type  = obj_type
    object_name = obj_name
    object_pose = Pose()
    object_pose.position.x = pos[0]
    object_pose.position.y = pos[1]
    object_pose.position.z = pos[2]
    object_pose.orientation.x = 0 
    object_pose.orientation.y = 0
    object_pose.orientation.z = 0
    object_pose.orientation.w = 1

    # get a target object xml
    xacro_file = os.path.join(xacro_path, xacro_type+'.urdf.xacro')

    arg_list = []
    arg_list.append('name:='+object_name)
    arg_list.append('color:=' + str(color))
    arg_list.append('length:=0.053')
    arg_list.append('width:=0.053')
    arg_list.append('height:=0.053')
    arg_list.append(xacro_file)

    spawn_urdf_object(arg_list, object_name, object_pose)
    return
    

def spawn_urdf_object(arg_list, object_name, object_pose):
    """
    Add object to GAZEBO
    """
    # get urdf(xml) from xacro
    opts, input_file_name = xacro.process_args(arg_list)
    xacro_output_memory = xacro.process_file(input_file_name, **vars(opts))
    xml = xacro_output_memory.toprettyxml(indent='  ')

    rospy.sleep(1)
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model',
                                     SpawnModel)

    req = SpawnModelRequest()
    req.model_name = object_name
    req.model_xml  = xml
    req.initial_pose = object_pose
    #req.reference_frame="world" # initial_pose is defined relative to the frame of this model/body
                                 # if left empty or "world", then gazebo world frame is used
                                 # if non-existent model/body is specified, an error is returned
                                 #   and the model is not spawned
    resp = spawn_model(req)
    
    rospy.set_param("world_description/"+object_name, xml)
    

def spawn_sdf_object(object_name, xyzrpy):
    """ """
    sdf_file = open(os.path.join(sdf_path, object_name, 'model.sdf'), 'r').read()

    object_pose = misc.list2Pose(xyzrpy)
    
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model',
                                     SpawnModel)    

    req = SpawnModelRequest()
    req.model_name = object_name
    req.model_xml  = sdf_file
    req.initial_pose = object_pose
    resp = spawn_model(req)
    
    rospy.set_param("world_description/"+object_name, sdf_file)
    
if __name__ == '__main__':

    rospy.init_node('change_world')
    rospy.sleep(5)
    
    ## add_cube(obj_name='r1', obj_type='cube',
    ##             pos=[0.92,-0.3,0])
    #spawn_sdf_object('soda_can', [0.92,0.5,0.6])

    rospy.loginfo("spawn sdf objects!!")
    spawn_sdf_object('book', [0.52,0.2,0.6, 0, 0, np.pi/4.])
    spawn_sdf_object('eraser', [0.52,-0.2,0.6, 0, 0, np.pi/4.])
    spawn_sdf_object('snacks', [0.72,0.3,0.6, 0, 0, np.pi/4.])
    spawn_sdf_object('soap2', [0.72,-0.0,0.6, 0, 0, np.pi/4.])
    spawn_sdf_object('biscuits', [0.72,-0.3,0.6, 0, 0, np.pi/4.])
    spawn_sdf_object('glue', [0.92,-0.2,0.6, 0, 0, np.pi/4.])
    spawn_sdf_object('soap', [0.92,0.2,0.6, 0, 0, np.pi/4.])
    
    ## spawn_sdf_object('dropbox', [1.92,0.1,0.6])
    ## spawn_sdf_object('short_table', [2.92,0.1,0.6])