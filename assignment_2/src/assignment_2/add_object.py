#!/usr/bin/env python  
import os, sys
import rospy
import numpy as np, math

from manip_challenge import add_object as ao


if __name__ == '__main__':

    rospy.init_node('change_world')
    rospy.sleep(5)

    rospy.loginfo("spawn sdf objects!!")

    world2baselink = np.array([0,0,0.594])
    
    grid_size = 0.1
    X,Y,Z = np.meshgrid(np.arange(0.3,0.8,grid_size).astype(float),
                        np.arange(0.,0.1,grid_size).astype(float),
                        np.arange(0,0.25,grid_size).astype(float))
    obstacles = np.vstack([np.ravel(X),np.ravel(Y),np.ravel(Z)]).T
    obstacles += world2baselink

    for i, pos in enumerate(obstacles):
        print pos
        ao.spawn_sdf_object('sphere', [pos[0],pos[1],pos[2], 0, 0, 0], name="obstacle"+str(i))
