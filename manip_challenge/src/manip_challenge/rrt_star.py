import math
import random

import matplotlib.pyplot as plt
import numpy as np

from operator import add
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

from complex_action_client.arm_client_ur5_robotiq_2F_85 import UR5ArmClient
from complex_action_client import misc, min_jerk, quaternion as qt

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse

class RRT_STAR:
    """
    Class for RRT star planning
    """

    class Node():
        """
        RRT Node
        """
        def __init__(self, position):
            self.position = [position[i] for i in range(len(position))]
            self.path = []
            self.cost = 0.0
            self.parent = None

    def __init__(self,
                 start_position,
                 goal_position,
                 obstacle_list,
                 grid_limits,

                 arm = None,
                 expand_dis=3.0, # step size
                 path_resolution=0.1, # grid size
                 goal_sample_rate=5,
                 max_iter=500,
                 dimension=2,
                 extend_size=100,
                 connect_circle_dist = 5.0,
                 search_until_max_iter=False,
                 animation=False):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacle_list:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.arm = arm
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
        self.connect_circle_dist = connect_circle_dist
        self.search_until_max_iter = search_until_max_iter
        self.animation = animation

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
            d, _ = self.calc_distance_and_orientation(new_node, nearest_node)
            new_node.cost = nearest_node.cost + d

            if self.check_valid_position(new_node, arm=self.arm, dimension=self.dimension) \
                and self.check_collision(new_node, self.obstacle_list, dimension=self.dimension):
                near_inds = self.find_near_nodes(new_node)

                node_with_updated_parent = self.choose_parent(new_node, near_inds)

                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if self.animation:
                self.draw_graph(rnd_node)

            if ((not self.search_until_max_iter) and new_node): # if reach goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)
        print("reached max iteration")
        last_index = self.search_best_goal_node()
        if(last_index is not None):
            return self.generate_final_course(last_index)

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

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list, dimension=self.dimension):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.end_node)
            if self.check_valid_position(t_node, arm=self.arm, dimension=self.dimension) \
                and self.check_collision(t_node, self.obstacle_list, dimension=self.dimension):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)

        dist_list = [sum((np.array(node.position) - np.array(new_node.position))**2) for node in self.node_list]

        near_inds = [dist_list.index(di) for di in dist_list if di <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list, dimension=self.dimension)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.position = edge_node.position
                near_node.cost = edge_node.cost
                near_node.path = edge_node.path
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_orientation(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def point_resolution(self, p):
        new_p = []
        for i in range(self.dimension):
            new_p.append( (p[i] - self.grid_limits[0][i])*self.extend_size + self.grid_limits[0][i]*self.extend_size )
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
                self.plot_circle(ox, oy, size)

            plt.plot(self.start_node.position[0], self.start_node.position[1], "xr")
            plt.plot(self.end_node.position[0], self.end_node.position[1], "xr")
            plt.axis("equal")
            plt.axis([-2, 15, -2, 15])
            plt.grid(True)
            plt.pause(0.01)

        elif(self.dimension==3):
            # plt.clf()
            self.ax = self.fig.add_subplot(111, projection='3d')   

            self.ax.set_xlim3d(self.grid_limits[0][0]*self.extend_size, self.grid_limits[1][0]*self.extend_size)
            self.ax.set_ylim3d(self.grid_limits[0][1]*self.extend_size, self.grid_limits[1][1]*self.extend_size)
            self.ax.set_zlim3d(self.grid_limits[0][2]*self.extend_size, self.grid_limits[1][2]*self.extend_size)

            for (p1, p2, p3, p4, p5, p6, p7, p8) in self.obstacle_list:
                # plot cuboid
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
                plt.plot([rnd.position[0]*self.extend_size], 
                    [rnd.position[1]*self.extend_size], 
                    [rnd.position[2]*self.extend_size], "^k")

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
            
    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    """ 
    find the index of nearest node nearest from rnd_node in node_list 
    """ 
    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [np.linalg.norm(np.array(node.position) - np.array(rnd_node.position)) for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacle_list, dimension=2):
        if node is None:
            return False

        if dimension==2:
            path_x = [row[0] for row in node.path]
            path_y = [row[1] for row in node.path]

            # circle obstacles
            for (ox, oy, size) in obstacle_list:
                dx_list = [ox - x for x in path_x]
                dy_list = [oy - y for y in path_y]
                d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

                if min(d_list) <= size**2:
                    return False  # collision
            return True  # safe

        elif dimension==3:
            path_x = [row[0] for row in node.path]
            path_y = [row[1] for row in node.path]
            path_z = [row[2] for row in node.path]

            # cuboid obstacles
            for (p1, p2, p3, p4, p5, p6, p7, p8) in obstacle_list:
                for (x, y, z) in zip(path_x, path_y, path_z):
                    # ref: https://math.stackexchange.com/questions/1472049/check-if-a-point-is-inside-a-rectangular-shaped-area-3d
                    
                    i=p2-p1
                    j=p4-p1
                    k=p5-p1
                    v = np.array([x,y,z])-p1
                    
                    if (0<=np.dot(v,i) and np.dot(v,i)<=np.dot(i,i) and
                        0<=np.dot(v,j) and np.dot(v,j)<=np.dot(j,j) and
                        0<=np.dot(v,k) and np.dot(v,k)<=np.dot(k,k)):
                        return False # collition
                    
            return True
    
    @staticmethod
    def check_valid_position(node, arm=None, dimension=2):
        # whether the position of the node can be reached by the robot arm or not
        if dimension==2:
            return True

        elif dimension==3:
            if arm==None:
                return True

            pose = Pose(position=Point(x=node.position[0], y=node.position[1], z=node.position[2]))
            return arm.ik_request(pose)

    @staticmethod
    def calc_distance_and_orientation(from_node, to_node):
        u = np.array(from_node.position)
        v = np.array(to_node.position)

        d = np.linalg.norm(u-v)
        if d==0: ds = [0 for _ in range(len(u))]
        else: ds = (v-u)/d
        
        return d, ds


def main():
    print("start " + __file__)

    dimension = 3

    if dimension==2:
        ############## 2D ######################
        # ====Search Path with RRT====
        obstacle_list = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                        (9, 5, 2)]  # [x, y, radius]
        # Set Initial parameters
        start_position = [0., 0.]
        goal_position = [6.0, 10.0]
        observation_space_low = [-2, -2]
        observation_space_high = [15, 15]
        grid_limits = [observation_space_low, observation_space_high]
        ############## 2D ######################
    elif dimension==3:
        ############## 3D ######################
        # ====Search Path with RRT====
        obstacle_list = []  # [x, y, z, radius]
        # Set Initial parameters
        start_position = [0., 0., 0.]
        goal_position = [6.0, 10.0, 10.]
        observation_space_low = [-2., -2., -2.]
        observation_space_high = [15., 15., 15.]
        grid_limits = [observation_space_low, observation_space_high]
        ############## 3D ######################


    my_rrt = RRT_STAR(
        start_position=start_position,
        goal_position=goal_position,
        obstacle_list=obstacle_list,
        grid_limits=grid_limits,
        expand_dis=1.0, # step size
        path_resolution=0.1, # grid size
        goal_sample_rate=10,
        max_iter=500,
        dimension=dimension,
        extend_size=100,
        animation=True)

    show_animation = True

    path = my_rrt.planning()
    print("len(path):", len(path))

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            my_rrt.draw_graph()
            if(my_rrt.dimension==2):
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.01)  # Need for Mac
                plt.show()
            elif(my_rrt.dimension==3):
                plt.plot([x for (x, y, z) in path], [y for (x, y, z) in path], [z for (x, y, z) in path], '-r')
                plt.grid(True)
                plt.pause(0.01)  # Need for Mac
                plt.show()


if __name__ == '__main__':
    main()
