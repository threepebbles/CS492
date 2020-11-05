#!/usr/bin/env python

import sys
import math
import numpy as np
from copy import copy, deepcopy
import threading, time

import rospy
import PyKDL
from riro_srvs.srv import String_Pose, String_PoseResponse, String_String, String_StringResponse, None_String, None_StringResponse
from gazebo_msgs.msg import ModelStates

import misc


class GazeboParser():

    def __init__(self):

        self._world_lock = threading.RLock() 
        self.wm = {}
       
        # subscriber
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gz_model_callback)
        
        # Service
        rospy.Service('get_object_pose', String_Pose, self._object_pose_srv)
        ## rospy.Service('get_object_grasp_pose', String_Pose, self._object_grasp_pose_srv)
        rospy.Service('get_object_height', String_Pose, self._object_height_srv)
        rospy.Service('get_object_range', String_String, self._object_range_srv)
        rospy.Service('get_object_rnd_pose', String_Pose, self._object_rnd_pose_srv)
        rospy.Service('get_object_close_pose', String_Pose, self._object_close_pose_srv)
        rospy.Service('get_object_in_hand', None_String, self._object_in_hand_srv)


    # -----------------------------------------------------------------------------------------
    # Subscribers
    # -----------------------------------------------------------------------------------------
    def _object_pose_srv(self, req):
        p = self.get_pose_from_world(req.data)
        assert p is not False, "{} pose is not available".format(req.data)
        msg = misc.list2Pose(p)        
        return String_PoseResponse(msg)  

    def _object_grasp_pose_srv(self, req):
        p = self.get_pose_from_world(req.data, return_grip=True)
        assert p is not False, "{} pose is not available".format(req.data)
        msg = misc.list2Pose(p)        
        return String_PoseResponse(msg)  

    def _object_height_srv(self, req):
        p = self.get_object_height(req.data)
        msg = misc.list2Pose([0,0,p,0,0,0])        
        return String_PoseResponse(msg)

    def _object_range_srv(self, req):
        return NotImplemented
        ## ret = self.get_object_range(req.data)
        ## return String_StringResponse( json.dumps(ret) )

    def _object_rnd_pose_srv(self, req):
        return NotImplemented

    def _object_close_pose_srv(self, req):
        return NotImplemented

    def _object_in_hand_srv(self, req):
        return NotImplemented
    
    def get_pose_from_world(self, name, return_top=False, return_grip=False):
        """ Return a pair of position and quaternion as a list """
        
        with self._world_lock:
            if len(self.wm)==0: return False
            frame = misc.array2KDLframe(self.wm[name]["pose"])

            if return_top:
                height = self.get_object_height(name)
                ## offset = PyKDL.Frame(PyKDL.Rotation.Identity(),
                ##                          PyKDL.Vector(0+0.003,0,height))                
                ## offset = PyKDL.Frame(PyKDL.Rotation.RotZ(-np.pi/4.),
                offset = PyKDL.Frame(PyKDL.Vector(0,0,height))
                frame *= offset

        return misc.KDLframe2List(frame)


    def gz_model_callback(self, msg):
        """ """
        model_names = msg.name
        model_poses = msg.pose
        
        with self._world_lock:            
            for i, model_name in enumerate(model_names):
                # exclude the robot, ground, default frames
                if model_name == "/": continue
                if model_name == "ground_plane": continue
                if model_name == "robot": continue
                if "default" in model_name: continue
                    
                # the reference frame is the world frame (=base_footprint in the begining)
                # which also equals to odom and base_model's xy in the begining
                pose = misc.pose2list(model_poses[i])
                pose = misc.list_quat2list_rpy(pose)

                updated_flag = False
                for j, obj in enumerate(self.wm):
                    ## if obj['type'] == 'na': continue
                    # check if the model model_name is in WM
                    if obj == model_name:
                        self.wm[obj]['pose'] = pose
                        updated_flag = True                        
                        break                

                if updated_flag is False:
                    self.wm[model_name] = {"pose": pose}

            
    def get_object_height(self, name):
        """
        """
        try:
            param_str = rospy.get_param("world_description/"+name)
        except:
            rospy.logerr("{} height is not available. Return zero.".format(name))
            return 0

        from bs4 import BeautifulSoup
        param_xml = BeautifulSoup(param_str, "xml")
        tags = param_xml.find_all("box")
        if len(tags)==0:
            rospy.logerr("{} collision model is not box. Return zero".format(name))
            return 0


        box_size = np.array(tags[0].contents[1].contents[0].encode('utf-8').split(" ")).astype(float)
        height = box_size[2]

        return height
        
    def get_object_range(self, name):
        return NotImplemented
        
        
    ## def run(self):
    ##     rate = rospy.Rate(10)
    ##     while not rospy.is_shutdown():
    ##         rate.sleep()
        



if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('--seed', action='store', type=int, 
                 default=int(time.time()),
                 help='input an integer random seed. (Default=current time)')
    opt, args = p.parse_args()

    np.random.seed(opt.seed)

    rospy.init_node("gazebo_parser")
    gp = GazeboParser()
    rospy.spin()
    
