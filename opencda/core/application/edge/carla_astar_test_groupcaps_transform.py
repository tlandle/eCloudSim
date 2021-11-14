
"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math

import matplotlib.pyplot as plt

from collab_sandbox import Traffic
from collab_sandbox import Car

import numpy as np 
import itertools

import scipy.io
import matplotlib.pyplot as plt
import matplotlib
import time
import timeit
from sklearn.cluster import AgglomerativeClustering
#from pypapi import events, papi_high as high 

from k_means_constrained import KMeansConstrained
import carla
from transform_utils import transform_processor
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

import pickle

show_animation = True


class AStarPlannerClient:

    def __init__(self):
      self.client = carla.Client('localhost', 2000)
      self.client.set_timeout(4.0)
      self.world = self.client.load_world('Town06')
      self.algorithm_latency = []

    def get_actors(self):
      return self._world.get_actors()

    def get_vehicles(self):
      return self.world.get_actors().filter("*vehicle*")

    def get_waypoints(self):
      return self.world.get_map().generate_waypoints(10)

    def get_four_lane_waypoints(self):
      dao = GlobalRoutePlannerDAO(self.world.get_map(), 2)
      grp = GlobalRoutePlanner(dao)
      grp.setup()
      waypoints = self.world.get_map().generate_waypoints(10)
      a = carla.Location(waypoints[343].transform.location)
      b = carla.Location(waypoints[1116].transform.location)
      c = carla.Location(waypoints[344].transform.location)
      d = carla.Location(waypoints[1117].transform.location)
      e = carla.Location(waypoints[345].transform.location)
      f = carla.Location(waypoints[1118].transform.location)
      g = carla.Location(waypoints[346].transform.location)
      j = carla.Location(waypoints[1119].transform.location)

      w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w2 = grp.trace_route(c, d) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w3 = grp.trace_route(e, f) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w4 = grp.trace_route(g, j) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      waypoints = w1+w2+w3+w4
      return waypoints

    def get_four_lane_waypoints_dict(self):
      dao = GlobalRoutePlannerDAO(self.world.get_map(), 2)
      grp = GlobalRoutePlanner(dao)
      grp.setup()
      waypoints_dict = {}
      waypoints = self.world.get_map().generate_waypoints(10)
      a = carla.Location(waypoints[343].transform.location)
      b = carla.Location(waypoints[1116].transform.location)
      c = carla.Location(waypoints[344].transform.location)
      d = carla.Location(waypoints[1117].transform.location)
      e = carla.Location(waypoints[345].transform.location)
      f = carla.Location(waypoints[1118].transform.location)
      g = carla.Location(waypoints[346].transform.location)
      j = carla.Location(waypoints[1119].transform.location)

      w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w2 = grp.trace_route(c, d) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w3 = grp.trace_route(e, f) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w4 = grp.trace_route(g, j) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      waypoints_dict[1] = {}
      waypoints_dict[2] = {}
      waypoints_dict[3] = {}
      waypoints_dict[4] = {}
      waypoints_dict[1]['x'] = []
      waypoints_dict[2]['x'] = []
      waypoints_dict[3]['x'] = []
      waypoints_dict[4]['x'] = []
      waypoints_dict[1]['y'] = []
      waypoints_dict[2]['y'] = []
      waypoints_dict[3]['y'] = []
      waypoints_dict[4]['y'] = []


      for waypoint in w1:
        waypoints_dict[1]['x'].append(waypoint[0].transform.location.x)
        waypoints_dict[1]['y'].append(waypoint[0].transform.location.y)

      for waypoint in w2:
        waypoints_dict[2]['x'].append(waypoint[0].transform.location.x)
        waypoints_dict[2]['y'].append(waypoint[0].transform.location.y)

      for waypoint in w3:
        waypoints_dict[3]['x'].append(waypoint[0].transform.location.x)
        waypoints_dict[3]['y'].append(waypoint[0].transform.location.y)

      for waypoint in w4:
        waypoints_dict[4]['x'].append(waypoint[0].transform.location.x)
        waypoints_dict[4]['y'].append(waypoint[0].transform.location.y)


      return waypoints_dict


    def set_vehicle_position(vehicle_id, location):
      actor = self.world.get_actors().filter(id)
      actor.set_location(location)


class AStarPlanner:

    def __init__(self, cars, ov, oy, resolution, rr=1, cars_on_road=None, slicenum=0):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        v = []
        x_start = []
        vt = []
        y = []

        for i in cars:
            v.append(i.v)
            vt.append(i.target_velocity)
            x_start.append(i.pos_x)
            y.append(i.lane)

        self.v = np.array(v)
        self.vt = np.array(vt)
        self.x_start = np.array(x_start)
        self.y = np.array(y)

        self.resolution = resolution
        self.rr = rr
        self.min_v, self.min_y = 0, 0
        self.max_v, self.max_y = 0, 0
        self.obstacle_map = None
        self.v_width, self.y_width = 0, 0
        self.motion_v, self.motion_y = self.get_motion_model(len(cars))
        self.calc_obstacle_map(ov, oy)

        self.cars_on_road = cars_on_road
        self.slicenum=slicenum

        # lanenum_each = np.zeros((max_y-1,1))
        # for i in range(0,len(node.v)):

        # print(len(self.motion_v))
        # sys.exit()

        # print(np.array(self.motion_y))

    class Node:
        def __init__(self, sv, sy, x_start, vt, cost, parent_index):

            self.v = np.array(sv)  # index of grid
            self.y = np.array(sy)  # index of grid
            self.x_start = np.array(x_start)
            self.x_tracked = None
            self.vt = np.array(vt)

            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.v) + "," + str(self.y) + "," + str(
                self.x_tracked) + "," + str(self.cost)

        def length_of_path(self,node_set,pathlen=0):
            if self.parent_index == -1:
                pathlen += 1 
                return pathlen
            else:
                pathlen += 1
                return node_set[self.parent_index].length_of_path(node_set=node_set,pathlen=pathlen)

    def planning(self):
        """
        A star path search
        input:
            s_v: start v position [m/s]
            s_y: start y position [m]
            gv: goal v position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        sv = self.v 
        sy = self.y 
        gy = self.y 
        gv = self.vt

        start_node = self.Node(self.calc_xy_index(sv, self.min_v),
                               self.calc_xy_index(sy, self.min_y), self.x_start, self.vt, 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gv, self.min_v),
                              self.calc_xy_index(gy, self.min_y), self.x_start, self.vt, 0.0, -1)

        start_node.x_tracked = self.x_start

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        empty_flag = 0

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                empty_flag = 1
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.v = current.v
                goal_node.y = current.y 
                goal_node.x_tracked = current.x_tracked
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(open_set[
                                                                         o],goal_node))
            current = open_set[c_id]

            # show graph
            # if show_animation:  # pragma: no cover
            #     plt.plot(self.calc_grid_position(current.x, self.min_x),
            #              self.calc_grid_position(current.y, self.min_y), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #                                  lambda event: [exit(
            #                                      0) if event.key == 'escape' else None])
            #     if len(closed_set.keys()) % 10 == 0:
            #         plt.pause(0.001)

            if (current.length_of_path(node_set=closed_set) >= 4): #Was 4 #current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.v = current.v
                goal_node.y = current.y 
                goal_node.x_tracked = current.x_tracked
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            # for i, _ in enumerate(self.motion):
            for i in range(0,len(self.motion_v)):
                for j in range(0,len(self.motion_y)):
                    # print(np.array(self.motion_v[i]))
                    # print(self.motion_y)
                    # sys.exit()
                    node = self.Node(current.v + np.array(self.motion_v[i]),
                                     current.y + np.array(self.motion_y[j]), self.x_start, self.vt,
                                     current.cost, c_id)
                    node.cost = self.calc_heuristic(node,current) #+ np.sum(abs(closed_set[c_id].y - node.y))
                    node.x_tracked = current.x_tracked + (node.v * 0.2)
                    node.x_tracked = node.x_tracked.astype(int)

                    n_id = self.calc_grid_index(node)

                    # If the node is not safe, do nothing
                    if not self.verify_node(node):
                        if node.y.any() > 1:
                            print("Node Not Viable: ", node.__str__())
                        continue

                    if n_id in closed_set:
                        continue

                    if n_id not in open_set:
                        open_set[n_id] = node  # discovered a new node
                        # print(node.__str__())
                        #print(np.sum(abs(node.v - node.vt)))
                    else:
                        if open_set[n_id].cost > node.cost:
                            # This path is the best until now. record it
                            open_set[n_id] = node

        rv, ry, rx = self.calc_final_path(goal_node, closed_set)

        return rv, ry, rx

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rv, ry = [self.calc_grid_position(goal_node.v, self.min_v)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        rx = []
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rv.append(self.calc_grid_position(n.v, self.min_v))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            rx.append(n.x_tracked)
            parent_index = n.parent_index

        return rv, ry, rx

    @staticmethod
    def calc_heuristic(n1,n2=0):
        w = 10.0  # weight of heuristic
        d = 0
        for i in range(0,len(n1.v)):
            # if n2 != 0:
            d += w * abs(n1.v[i] - n1.vt[i]) + 0.5*abs(n1.y[i]-n2.y[i])# math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        # print("Index: ", index)
        # print("Position: ", min_position)
        pos = index * self.resolution # + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        for i in range(0,len(position)):
            position[i] = round((position[i]) / self.resolution)
        return position

    def calc_grid_index(self, node):
        coord_tracked = []
        for i in range(0,len(node.x_tracked)):
            coord_tracked.append(node.x_tracked[i])
            coord_tracked.append(node.y[i])
        str1 = ''.join(str(e) for e in coord_tracked)
        return hash(str1)
        #return (node.x_tracked) + (node.y - self.min_y)

    def verify_node(self, node):

        for i in range(0,len(node.v)):
            px = self.calc_grid_position(node.v[i], self.min_v)
            py = self.calc_grid_position(node.y[i], self.min_y)

            if px < self.min_v:
                return False
            elif py < self.min_y:
                return False
            elif px >= self.max_v:
                # print("Px False")
                return False
            elif py >= self.max_y:
                # if py > 1:
                #     print("Y False ", py)
                return False

            # if self.obstacle_map[node.v][node.y]:
            #     return False

        #collision check
        for i in range(0,len(node.v)):
            for j in range(i+1,len(node.v)):
                if node.y[i] == node.y[j] and abs(node.x_tracked[j]-node.x_tracked[i]) <= 10:
                    # print("False for constraint")
                    return False
        # collision check: Other cars
        if self.cars_on_road is not None:
            for i in range(0,len(node.v)):
                for j in self.cars_on_road:
                    if j.slice != self.slicenum:
                        if node.y[i] == j.lane and abs(node.x_tracked[i]-j.pos_x) <= 10:
                            return False
                        if (j.intentions == "Lane Change -1" and node.y[i] == j.lane-1) and abs(node.x_tracked[i]-j.pos_x) <= 10:
                            return False
                        if (j.intentions == "Lane Change 1" and node.y[i] == j.lane+1) and abs(node.x_tracked[i]-j.pos_x) <= 10:
                            return False                   
        #Lane Diversity requirement

        return True

    def calc_obstacle_map(self, ov, oy):

        self.min_v = round(min(ov))
        self.min_y = round(min(oy))
        self.max_v = round(max(ov))
        self.max_y = round(max(oy))
        # print("min_x:", self.min_v)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_v)
        # print("max_y:", self.max_y)

        self.v_width = round((self.max_v - self.min_v) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("v_width:", self.v_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.v_width)]
        for iv in range(self.v_width):
            v = self.calc_grid_position(iv, self.min_v)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iov, ioy in zip(ov, oy):
                    d = math.hypot(iov - v, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[iv][iy] = True
                        break

    @staticmethod
    def get_motion_model(numcars):

        motion_v_atomic = [-1,0,1] #np.arange(-1,1,step=1)
        motion_y_atomic = [-1,0,1]
        motion_y = []
        motion_v = []
        for element in itertools.product(motion_y_atomic, motion_y_atomic):
            motion_y.append(element)
        for element in itertools.product(motion_v_atomic, motion_v_atomic):
            motion_v.append(element)

        # motion_y_atomic = motion_y_atomic + motion_y_atomic + motion_y_atomic
        # motion_v_atomic = motion_v_atomic + motion_v_atomic + motion_v_atomic

        # motion_y_final = itertools.permutations(motion_y_atomic,r=3)
        # motion_v_final = itertools.permutations(motion_v_atomic,r=3)
        motion_y_final = motion_y_atomic
        motion_v_final = motion_v_atomic

        for i in range(0,numcars-1):
            motion_y_interim = []
            for j in motion_y_final:
                for k in motion_y_atomic:
                    motion_y_interim.append(np.append(j,k))
            motion_y_final = motion_y_interim

        for i in range(0,numcars-1):
            motion_v_interim = []
            for j in motion_v_final:
                for k in motion_v_atomic:
                    motion_v_interim.append(np.append(j,k))
            motion_v_final = motion_v_interim

        # for i in motion_y_atomic:
        #     for j in motion_y_atomic:
        #         for k in motion_y_atomic:
        #             motion_y_final.append(np.array([i,j,k]))
        # for i in motion_v_atomic:
        #     for j in motion_v_atomic:
        #         for k in motion_v_atomic:
        #             motion_v_final.append(np.array([i,j,k]))

        # for element in itertools.product(motion_y, motion_y_atomic):
        #     motion_y_final.append(element)
        # for element in itertools.product(motion_v, motion_v_atomic):
        #     motion_v_final.append(element)

        return motion_v_final, motion_y_final

def get_states_carlist(car_list):
    carnum = 0
    for i in car_list:
        print("X Coordinate of ", carnum, " Is: ", i.pos_x)
        print("Y Coordinate of ", carnum, " Is: ", i.lane)
        print("Velocity of ", carnum, " Is: ", i.v)
        carnum += 1

def get_slice_plans(Traffic_Tracker, ov, oy, slice_length=15, map_length=1000):

    slice_list = []
    lanechange_command = []
    vel_array = []

    for i in range(0,int(map_length/slice_length) + 1):
        slice_list.append([])
        lanechange_command.append([])
        vel_array.append([])

    for i in Traffic_Tracker.cars_on_road:
        i.slice = int(i.pos_x/slice_length)
        slice_list[i.slice].append(i)

    for i in range(int(map_length/slice_length)-1,-1,-1):
        if len(slice_list[i]) >= 2:
            # get_states_carlist(slice_list[i])
            a_star = AStarPlanner(slice_list[i], ov, oy, 1, 1.0, Traffic_Tracker.cars_on_road, i)
            rv, ry, rx_tracked = a_star.planning()

            lanechange_command[i] = ry[-2]
            vel_array[i] = rv[-2]

    return slice_list, lanechange_command, vel_array

def find_nearest(array, value):
    array = np.asarray(array)
    idx = np.nanargmin((np.abs(array - value)))
    return array[idx], idx

def save_obj(obj, name ):
    with open('obj/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)

def main(): #Example scenario test for CARLA waypoints
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    client = AStarPlannerClient()

    # set obstacle positions
    ov, oy = [], [] #Limits for velocity (ov) and lane (y), alter for different lane limits/velocity limits (REQUIRES CLEANING FOR GENERALIZATION)
    for i in range(0, 25):
        ov.append(i)
        oy.append(0.0)
    for i in range(0, 4):
        ov.append(25.0)
        oy.append(i)
    for i in range(0, 26):
        ov.append(i)
        oy.append(4.0)
    for i in range(0, 5):
        ov.append(0.0)
        oy.append(i)

    #waypoints = load_obj('waypoints')
    waypoints = client.get_four_lane_waypoints_dict()
    print(waypoints.keys())

    processor = transform_processor(waypoints)
    _, _ = processor.process_waypoints_bidirectional(0)
    inverted = processor.process_forward(0)
    dt = 0.2
    numlanes = 4

    Car_1 = Car(dt,numlanes,inverted[0][0,0],inverted[0][1,0],5) #Last arg is vehicle velocity, can be loaded up from a file.
    Car_2 = Car(dt,numlanes,inverted[1][0,0],inverted[1][1,0],5) 
    Car_3 = Car(dt,numlanes,inverted[2][0,0],inverted[2][1,0],5) 
    Car_4 = Car(dt,numlanes,inverted[3][0,0],inverted[3][1,0],5)

    Car_1.target_velocity = 22
    Car_2.target_velocity = 21
    Car_3.target_velocity = 15
    Car_4.target_velocity = 15

    Car_2.target_lane = 0

    carlist = [Car_1, Car_2, Car_3, Car_4]

    a_star = AStarPlanner(carlist, ov, oy, 1, robot_radius)
    rv, ry, rx_tracked = a_star.planning()

    print(rv)
    print(ry)
    print(rx_tracked)

    x_res = rx_tracked[-2]
    y_res = ry[-2]
    waypoints_rev = {1 : [], 2 : [], 3 : [], 4 : []}
    processed_array = []
    for i in range(0,4):
        processed_array.append(np.array([[x_res[i]],[y_res[i]]]))
    print(processed_array)

    back = processor.process_back(processed_array)

if __name__ == '__main__':
    main()

