import math
import random

import matplotlib.pyplot as plt
import numpy as np

from operator import add
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

# for collision check
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


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, position):
            self.position = [position[i] for i in range(len(position))]
            self.path = []
            self.parent = None

    def __init__(self,
                 start_position,
                 goal_position,
                 obstacle_list,
                 grid_limits,
                 expand_dis=3.0, # step size
                 path_resolution=0.1, # grid size
                 goal_sample_rate=5,
                 max_iter=500,
                 dimension=2,
                 extend_size=100,
                 animation=False,

                 **kwargs):
        """
        Setting Parameter

        start_position:Start Position [x,y,z,...]
        goal_position:Goal Position [x,y,z,...]
        obstacle_list: the list of geometric informations of the obstacles
        grid_limits: Random Sampling Area [minx, miny, minz, ...], [maxx, maxy, maxz, ...]

        """
        self.start_node = self.Node(start_position)
        self.end_node = self.Node(goal_position)
        self.grid_limits = grid_limits
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.dimension = dimension
        self.extend_size = extend_size
        self.animation = animation

        
        if(self.dimension==6):
            self.contain_gripper = kwargs['contain_gripper']
            self.grasping_object = kwargs['grasping_object']
            
            self.arm_kdl = create_kdl_kin('base_link', 'gripper_link')
            self.collision_check_manager = collision_check.CollisionChecker(self.arm_kdl, contain_gripper=self.contain_gripper, grasping_object=self.grasping_object, viz=True)

            # update a collision manager for objects
            self.collision_check_manager.update_manager()
            rospy.sleep(0.1)


        if self.dimension==3 and self.animation:
            self.fig = plt.figure()

    def planning(self):
        """
        rrt path planning

        animation: flag for animation on or off
        """
            
        self.node_list = [self.start_node]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]    
            
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if not self.check_collision(new_node):
                self.node_list.append(new_node)

            # if self.animation:
            #     self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1]) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end_node,
                                        self.expand_dis)
                
                if not self.check_collision(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

            # if self.animation:
                # self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.position)
        d, ds = self.calc_distance_and_orientation(new_node, to_node)
        ds = [di*self.path_resolution for di in ds]

        new_node.path = [new_node.position]

        if extend_length > d:
            extend_length = d

        n_expand = int(math.floor(extend_length / self.path_resolution))

        for _ in range(n_expand):
            new_node.position = list(map(add, new_node.position, ds))
            new_node.path.append(new_node.position)

        d, _ = self.calc_distance_and_orientation(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path.append(to_node.position)
            new_node.position = to_node.position

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [self.end_node.position]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.position)
            node = node.parent
        path.append(node.position)

        return path

    def calc_dist_to_goal(self, target_node):
        u = np.array(target_node.position)
        v = np.array(self.end_node.position)

        d = np.linalg.norm(u-v)
        return d

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node([random.uniform(self.grid_limits[0][i], self.grid_limits[1][i]) for i in range(self.dimension)])
        else:  # goal point sampling
            rnd = self.Node(self.end_node.position)
        return rnd

    def point_resolution(self, p, grid_limits=None):
        """
        point resolution with respect to extend_size
        """
        if (grid_limits==None):
            grid_limits = self.grid_limits

        new_p = []
        for i in range(self.dimension):
            new_p.append( (p[i] - grid_limits[0][i])*self.extend_size + grid_limits[0][i]*self.extend_size )
        return np.array(new_p)

    def draw_graph(self, rnd=None):
        if(self.dimension==2):
            plt.clf()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if rnd is not None:
                plt.plot(rnd.position[0], rnd.position[1], "^k")
            for node in self.node_list:
                if node.parent:
                    plt.plot([row[0] for row in node.path], [row[1] for row in node.path], "-g")

            for (ox, oy, size) in self.obstacle_list:
                # plot circle
                color="-b"
                deg = list(range(0, 360, 5))
                deg.append(0)
                xl = [ox + size * math.cos(np.deg2rad(d)) for d in deg]
                yl = [oy + size * math.sin(np.deg2rad(d)) for d in deg]
                plt.plot(xl, yl, color)

            plt.plot(self.start_node.position[0], self.start_node.position[1], "xr")
            plt.plot(self.end_node.position[0], self.end_node.position[1], "xr")
            plt.axis("equal")
            plt.axis([-2, 15, -2, 15])
            plt.grid(True)
            plt.pause(0.01)

        elif(self.dimension==3):
            self.ax = self.fig.add_subplot(111, projection='3d')

            self.ax.set_xlim3d(self.grid_limits[0][0]*self.extend_size, self.grid_limits[1][0]*self.extend_size)
            self.ax.set_ylim3d(self.grid_limits[0][1]*self.extend_size, self.grid_limits[1][1]*self.extend_size)
            self.ax.set_zlim3d(self.grid_limits[0][2]*self.extend_size, self.grid_limits[1][2]*self.extend_size)

            for (p1, p2, p3, p4, p5, p6, p7, p8) in self.obstacle_list:
                # plot cuboid obstacles
                vtcs = np.array([self.point_resolution(p1), self.point_resolution(p2), 
                    self.point_resolution(p3), self.point_resolution(p4), 
                    self.point_resolution(p5), self.point_resolution(p6), 
                    self.point_resolution(p7), self.point_resolution(p8)])
                
                self.ax.scatter3D(vtcs[:, 0], vtcs[:, 1], vtcs[:, 2])
                faces = [[vtcs[0], vtcs[1], vtcs[2], vtcs[3]], [vtcs[0], vtcs[4], vtcs[7], vtcs[3]], 
                        [vtcs[4], vtcs[5], vtcs[6], vtcs[7]], [vtcs[7], vtcs[6], vtcs[2], vtcs[3]], 
                        [vtcs[6], vtcs[5], vtcs[1], vtcs[2]], [vtcs[4], vtcs[5], vtcs[1], vtcs[0]]]
                
                self.ax.add_collection3d(Poly3DCollection(faces, 
                    facecolors='cyan', linewidths=1, edgecolors='cyan', alpha=.25))
                
            if rnd is not None:
                plt.plot([rnd.position[0]*self.extend_size], [rnd.position[1]*self.extend_size], [rnd.position[2]*self.extend_size], "^k")

            for node in self.node_list:
                if node.parent:
                    plt.plot([row[0]*self.extend_size for row in node.path], 
                        [row[1]*self.extend_size for row in node.path], 
                        [row[2]*self.extend_size for row in node.path], "-g")

            
            plt.plot([self.start_node.position[0]*self.extend_size, self.end_node.position[0]*self.extend_size], 
                [self.start_node.position[1]*self.extend_size, self.end_node.position[1]*self.extend_size],
                [self.start_node.position[2]*self.extend_size, self.end_node.position[2]*self.extend_size],
                 "xr")
            plt.pause(0.01)

    """ 
    find the index of nearest node nearest from rnd_node in node_list 
    """ 
    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [np.linalg.norm(np.array(node.position) - np.array(rnd_node.position)) for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    def check_collision(self, node):
        """
        collision: return True
        not collision: return False
        """
        if node is None:
            return False

        # x,y
        if self.dimension==2:
            path_x = [row[0] for row in node.path]
            path_y = [row[1] for row in node.path]
            # circle obstacles
            for (ox, oy, size) in self.obstacle_list:
                dx_list = [ox - x for x in path_x]
                dy_list = [oy - y for y in path_y]
                d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

                if min(d_list) <= size**2:
                    return False  # collision
            return True  # safe
        # x,y,z
        elif self.dimension==3:
            # check collision with the obstacles
            path_x = [row[0] for row in node.path]
            path_y = [row[1] for row in node.path]
            path_z = [row[2] for row in node.path]

            # cuboid obstacles
            for (p1, p2, p3, p4, p5, p6, p7, p8) in self.obstacle_list:
                for (x, y, z) in zip(path_x, path_y, path_z):
                    # ref: https://math.stackexchange.com/questions/1472049/check-if-a-point-is-inside-a-rectangular-shaped-area-3d
                    
                    i=p2-p1
                    j=p4-p1
                    k=p5-p1
                    v = np.array([x,y,z])-p1
                    
                    if (0<=np.dot(v,i) and np.dot(v,i)<=np.dot(i,i) and
                        0<=np.dot(v,j) and np.dot(v,j)<=np.dot(j,j) and
                        0<=np.dot(v,k) and np.dot(v,k)<=np.dot(k,k)):
                        return False # collision
            return True

        # joint angles
        elif self.dimension==6:
            # check if an arm collides with objects
            # flag==True: collision

            if (self.grasping_object is None):
                flag, cs = self.collision_check_manager.in_collision(node.position)
                # print(flag, cs)
                if(self.contain_gripper==True):
                    # print("containing")
                    return flag
                else:
                    # print("not containing")
                    cnt = len(cs)
                    for el in cs:
                        if(el[0]=="gripper_link" or el[1]=="gripper_link"):
                            cnt -= 1
                    return (cnt!=0)
            else:
                # (collision between robot arm and objects, collision between the grasping object and the other objects)
                (flag1, cs1), (flag2, cs2) = self.collision_check_manager.in_collision(node.position)
                cnt1 = len(cs1)
                for el in cs1:
                    if(el[0]==self.grasping_object or el[1]==self.grasping_object):
                        cnt1 -= 1
                
                flag2 = False
                for el in cs2:
                    if(el[0]!=self.grasping_object and el[1]!=self.grasping_object):
                        flag2 = True
                        break
                return ((cnt1!=0) or (flag2))

            
            

    @staticmethod
    def calc_distance_and_orientation(from_node, to_node):
        u = np.array(from_node.position)
        v = np.array(to_node.position)

        d = np.linalg.norm(u-v)
        if d==0: ds = [0 for _ in range(len(u))]
        else: ds = (v-u)/d
        
        return d, ds


# def main():
#     print("start " + __file__)

#     dimension = 3
#     if dimension==2:
#         obstacle_list = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
#                         (9, 5, 2)]  # [x, y, radius]
#         start_position = [0., 0.]
#         goal_position = [6.0, 10.0]
#         observation_space_low = [-2, -2]
#         observation_space_high = [15, 15]
#         grid_limits = [observation_space_low, observation_space_high]

#     elif dimension==3:
#         obstacle_list = []  # (p1, p2, ..., p8)
#         start_position = [0., 0., 0.]
#         goal_position = [6.0, 10.0, 10.]
#         observation_space_low = [-2., -2., -2.]
#         observation_space_high = [15., 15., 15.]
#         grid_limits = [observation_space_low, observation_space_high]


#     show_animation = True

#     rrt = RRT(
#         start_position=start_position,
#         goal_position=goal_position,
#         obstacle_list=obstacle_list,
#         grid_limits=grid_limits,
#         expand_dis=1.0, # step size
#         path_resolution=0.1, # grid size
#         goal_sample_rate=5,
#         max_iter=500,
#         dimension=dimension,
#         extend_size=100,
#         animation=show_animation)

#     path = rrt.planning()
#     print("len(path):", len(path))

#     if path is None:
#         print("Cannot find path")
#     else:
#         print("found path!!")

#         # Draw final path
#         if show_animation:
#             rrt.draw_graph()
#             if(rrt.dimension==2):
#                 plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
#                 plt.grid(True)
#                 plt.pause(0.01)  # Need for Mac
#                 plt.show()
#             elif(rrt.dimension==3):
#                 plt.plot([x for (x, y, z) in path], [y for (x, y, z) in path], [z for (x, y, z) in path], '-r')
#                 plt.grid(True)
#                 plt.pause(0.01)  # Need for Mac
#                 plt.show()


# if __name__ == '__main__':
#     main()
