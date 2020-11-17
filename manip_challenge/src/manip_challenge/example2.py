#!/usr/bin/env python
import sys, math, numpy as np
from copy import copy, deepcopy
import rospy
## import PyKDL
import misc

# msg: topic, srv: service

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse
from pprint import pprint

from gazebo_msgs.msg import ModelStates, LinkStates
index_of_model_states = {'ground_plane':0, 'ur5_base':1, 'cafe_table':2, 'cafe_table_left':3, 'cafe_table_right':4, 'storage_left':5, 'storage_right':6, 'robot':7, 'book':8, 'eraser':9, 'snacks':10, 'soap2':11, 'biscuits':12, 'glue':13, 'soap':14}
# index_of_link_states = {'ground_plane::link': 0, 'ur5_base::ur5_base::top_plate': 1, 'ur5_base::ur5_base::leg1': 2, 'ur5_base::ur5_base::leg2': 3, 'ur5_base::ur5_base::leg3': 4, 'ur5_base::ur5_base::leg4': 5, 'cafe_table::cafe_table::link': 6, 'cafe_table_left::ur5_base::top_plate': 7, 'cafe_table_left::ur5_base::leg1': 8, 'cafe_table_left::ur5_base::leg2': 9, 'cafe_table_left::ur5_base::leg3': 10, 'cafe_table_left::ur5_base::leg4': 11, 'cafe_table_right::ur5_base::top_plate': 12, 'cafe_table_right::ur5_base::leg1': 13, 'cafe_table_right::ur5_base::leg2': 14, 'cafe_table_right::ur5_base::leg3': 15, 'cafe_table_right::ur5_base::leg4': 16, 'storage_left::storage::storage-base': 17, 'storage_right::storage::storage-base': 18, 'robot::base_link': 19, 'robot::shoulder_link': 20, 'robot::upper_arm_link': 21, 'robot::forearm_link': 22, 'robot::wrist_1_link': 23, 'robot::wrist_2_link': 24, 'robot::wrist_3_link': 25, 'robot::robotiq_85_left_inner_knuckle_link': 26, 'robot::robotiq_85_left_finger_tip_link': 27, 'robot::robotiq_85_left_knuckle_link': 28, 'robot::robotiq_85_right_inner_knuckle_link': 29, 'robot::robotiq_85_right_finger_tip_link': 30, 'robot::robot::robotiq_85_right_knuckle_link': 31, 'book::book_link': 32, 'eraser::eraser_link': 33, 'snacks::snacks_link': 34, 'soap2::soap2_link': 35, 'biscuits::biscuits_link': 36, 'glue::glue_link': 37, 'soap::soap_link': 38}
obstacles_size = {'book':[0.13, 0.03, 0.206], 'eraser':[0.135, 0.06, 0.05], 'snacks':[0.165, 0.06, 0.235], 'soap2':[0.065, 0.04, 0.105], 'biscuits':[0.19, 0.06, 0.15], 'glue':[0.054, 0.032, 0.133], 'soap':[0.14, 0.065, 0.1]}


def model_states_callback(msg, subscriber):
    """ 
    A callback function that subscribes ?.

    Parameters
    ----------
    msg : ?
    """
    # print(msg)
    
    print (msg)
    


# subscribe once
def get_object_info(target_object, timeout=3.):
    try:
        msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout)
        # msg = rospy.wait_for_message('/gazebo/link_states', LinkStates, timeout)
        # print(len(msg.name))
        # for idx, name in enumerate(msg.name):
        #     print('\'{}\': {},'.format(name, idx), end=' ')
        # print(msg.pose)
        # pprint(dir(msg), indent=2)
        print(msg.pose[index_of_model_states[target_object]])
        
    except ROSException as e:
        rospy.logwarn('get_joint_state timed out after %1.1f s' % timeout)

    
if __name__ == '__main__':

    rospy.init_node('example1')
    rospy.sleep(1)

    target_object = "book"
    # rospy.wait_for_service('/get_object_pose')
    # pose_srv_req = rospy.ServiceProxy('/get_object_pose', String_Pose)
    # # print pose_srv_req
    # # You can send a query to obtain the pose of a selected object
    # obj_pose = pose_srv_req(target_object).pose
    # print obj_pose

    # rospy.wait_for_service('/get_object_height')
    # height_srv_req = rospy.ServiceProxy('/get_object_height', String_Pose)
    # obj_height = height_srv_req(target_object).pose
    # print obj_height

    # subscribe infinitely
    # model_state_subsriber = rospy.Subscriber('/gazebo/model_states',
    #                  ModelStates,
    #                  model_states_callback)

    get_object_info(target_object)
    
    # reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    # reset_simulation()
