#!/usr/bin/env python
import math
import random
import numpy as np
from operator import add

# for collision check
import collision_check2
import rospy

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
                 expand_dis=0.03, # step size
                 path_resolution=0.01, # grid size
                 goal_sample_rate=5,
                 max_iter=5000,
                 dimension=6,

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

        self.contain_gripper = kwargs['contain_gripper']
        self.grasping_object = kwargs['grasping_object']
        self.grasping_direction = kwargs['grasping_direction']
        
        self.arm_kdl = collision_check2.create_kdl_kin('base_link', 'gripper_link')
        self.collision_check_manager = collision_check2.CollisionChecker(self.arm_kdl, contain_gripper=self.contain_gripper, grasping_object=self.grasping_object, grasping_direction=self.grasping_direction, viz=True)
        
        # update a collision manager for objects
        self.collision_check_manager.update_manager()
        # rospy.sleep(0.1)

    def planning(self):
        """
        rrt path planning

        """
            
        self.node_list = [self.start_node]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]    
            
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if not self.check_collision(new_node):
                self.node_list.append(new_node)

            if self.calc_dist_to_goal(self.node_list[-1]) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end_node,
                                        self.expand_dis)
                
                if not self.check_collision(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

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
        x = random.randint(0, 100)
        if x > self.goal_sample_rate:
            rnd = self.Node([random.uniform(self.grid_limits[0][i], self.grid_limits[1][i]) for i in range(self.dimension)])
        elif x > self.goal_sample_rate-15:
            rnd = self.Node([a+b for a,b in zip(self.end_node.position, [0, -0.3*np.pi, -0.8*np.pi, 0., 0., 0.])])
        else:  # goal point sampling
            rnd = self.Node(self.end_node.position)
        return rnd

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

        self.collision_check_manager.update_manager()
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
                if(el[0]!=self.grasping_object and el[1]!=self.grasping_object): continue
                d, _ = self.calc_distance_and_orientation(node, self.start_node)
                if(d<0.03): continue
                
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
