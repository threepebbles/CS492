#!/usr/bin/env python
import sys, time
import rospy
import numpy as np, math
import PyKDL

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose
from complex_action_client import misc, min_jerk, quaternion as qt

import astar

from sklearn.neighbors import BallTree
from complex_action_client import misc
from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient
import tf

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

QUEUE_SIZE = 10

def get_object_frame(target_object):
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


def generate_obstacles(resolution=0.05):
    """ 
    A function to generate obstacles

    Parameters
    ----------
    resolution : float
        a value of grid resolution

    Returns
    -------
    obstacles : list
        a list of obstacle coordinates.
    obstacle_tree : BallTree 
        a BallTree object from Scikit-Learn. The tree includes
        the coordinates of obstacles.
    """
    world2base = get_base_frame()
    world2baselink = np.array(world2base.p)

    X,Y,Z = np.meshgrid(np.arange(-10,-5,resolution).astype(float),
                        np.arange(-10,-5,resolution).astype(float),
                        np.arange(-10,-5,resolution).astype(float))
    obstacles = np.vstack([np.ravel(X),np.ravel(Y),np.ravel(Z)]).T
    obstacle_tree = BallTree(obstacles)

    return obstacles, obstacle_tree

    
def test(arm, goal_pose):
    """ 
    A function to generate obstacle avoidance motion

    Parameters
    ----------
    arm : class object
        a class object to control the robot

    """

    # initialize variables
    resolution  = 0.01
    robot_size = 0.01    
    actions = [[-1,-1,-1], [-1,0,-1], [-1,1,-1], [0,-1,-1],
               [0,0,-1], [0,1,-1], [1,-1,-1], [1,0,-1], [1,1,-1],
               [-1,-1,0], [-1,0,0], [-1,1,0], [0,-1,0],
               [0,1,0], [1,-1,0], [1,0,0], [1,1,0],
               [-1,-1,1], [-1,0,1], [-1,1,1], [0,-1,1], [0,0,1],
               [0,1,1], [1,-1,1], [1,0,1], [1,1,1] ]
    actions = np.array(actions)*resolution
    grid_limits = [[-0.1,-0.7,-0.1],[0.8,0.7,1.0]]
    obstacles, obstacle_tree = generate_obstacles()
    
    start_pose = arm.getEndeffectorPose()
    # print("start_pose:\n", start_pose)
    # return

    start = [start_pose.position.x,start_pose.position.y,start_pose.position.z] 
    goal  = [goal_pose.position.x,goal_pose.position.y,goal_pose.position.z]

    # get the nearest pose on the grid before planning
    start_on_grid = astar.get_grid_pos(start, resolution, grid_limits )
    goal_on_grid = astar.get_grid_pos(goal, resolution, grid_limits )

    # run the planning algorithm
    path = astar.astar_planning(start_on_grid, goal_on_grid, actions, resolution, grid_limits, obstacle_tree, robot_size)
    print("len(path): {}".format(len(path)))
    pos_traj = path
    

    traj_len = len(pos_traj)
    pose_traj = []
    for i, pos in enumerate(pos_traj):
        p = Pose()
        # ------------------------------------------------------
        # Place your code here
        # ------------------------------------------------------
        # position
        p.position.x = pos[0]
        p.position.y = pos[1]
        p.position.z = pos[2]
        
        # orientation (You can use the SLERP function in quaternion.py)
        p.orientation = qt.slerp(start_pose.orientation, goal_pose.orientation, float((i+1.)/traj_len))
        # ------------------------------------------------------
        pose_traj.append(p)
    # print(goal_pose)

    # print("start_pose: {}\n, goal_pose: {}\n\n".format(start_pose, goal_pose))
    # print("traj: start_pose: {}\n, goal_pose: {}\n\n".format(pose_traj[0], pose_traj[-1]))
    arm.movePoseTrajectory(pose_traj, timeout=2.)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d', aspect='equal')
    # ax.set_xlim3d(grid_limits[0][0], grid_limits[1][0] )
    # ax.set_ylim3d(grid_limits[0][1], grid_limits[1][1] )
    # ax.set_zlim3d(grid_limits[0][2], grid_limits[1][2] )
    # for p in pose_traj:
    #     # print([p.position.x, p.position.y, p.position.z])
    #     # print([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    #     # plot current point?
    #     ax.plot([p.position.x],
    #                  [p.position.y],
    #                  [p.position.z], '.b')
    #     plt.pause(0.001)
    # plt.show()

    
        
if __name__ == '__main__':
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    rospy.sleep(1)

    # create action client
    arm = UR5ArmClient(timeout_scale=1., sim=True)

        # initialize the robot
    arm.moveJoint([-0.862410612, -1.30713835, 1.31642488, -1.69522468, -1.87213523, 0])
    arm.gripperOpen()   

    rospy.sleep(1)
    # print("angles: ", arm.getJointAngles())
    
    target_object = 'eraser'
    world2base = get_base_frame()
    world2obj = get_object_frame(target_object)
    base2obj  = world2base.Inverse() * world2obj
    base2obj.p[2] -= 0.04
    grasp_ps = misc.KDLframe2Pose(base2obj)
    base2obj.p[2] += 0.06
    pre_grasp_ps = misc.KDLframe2Pose(base2obj)

    print("going to {}".format(target_object))

    test(arm, grasp_ps)
    rospy.sleep(2)
    arm.gripperClose()
    # print(arm.getEndeffectorPose())

    # move into storage_right
    world2storage_right  = get_storage_frame('storage_right')
    base2storage_right = world2base.Inverse() * world2storage_right
    storage_right_grasp_ps = misc.KDLframe2Pose(base2storage_right)
    storage_right_grasp_ps.position.z += 0.06

    print("going to storage..")
    test(arm, storage_right_grasp_ps)
    arm.gripperOpen()

    rospy.spin()
    

