# -*- coding: utf-8 -*-

"""Platooning Manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import uuid
import weakref

import carla
import matplotlib.pyplot as plt
import numpy as np

import opencda.core.plan.drive_profile_plotting as open_plt
from opencda.core.application.edge.astar_test_groupcaps_transform import *

class EdgeManager(object):
    """
    Edge manager. Used to manage all vehicle managers under control of the edge

    Parameters
    ----------
    config_yaml : dict
        The configuration dictionary for edge.

    cav_world : opencda object
        CAV world that stores all CAV information.

    Attributes
    ----------
    pmid : int
        The  platooning manager ID.
    vehicle_manager_list : list
        A list of all vehciel managers within the platoon.
    destination : carla.location
        The destiantion of the current plan.
    """

    def __init__(self, config_yaml, cav_world):

        self.edgeid = str(uuid.uuid1())
        self.vehicle_manager_list = []
        #self.target_speed = config_yaml['target_speed']`
      #self.locations = []
        self.destination = None
        # Query the vehicle locations and velocities + target velocities
        self.spawn_x = []
        self.spawn_y = []
        self.spawn_v = [] # probably 0s but can be target vel too
        self.xcars = None
        self.ycars = None
        self.x_states = None
        self.y_states = None
        self.tv = None
        self.v = None
        self.target_velocities = None
        self.Traffic_Tracker = None

        self.cav_world = weakref.ref(cav_world)()

    def start_edge(self):
      for vehicle_manager in self.vehicle_manager_list:
          self.spawn_x.append(vehicle_manager.vehicle.get_location().x)
          self.spawn_y.append(vehicle_manager.vehicle.get_location().y)
          self.spawn_v.append(vehicle_manager.vehicle.get_velocity())
          vehicle_manager.agent.get_local_planner().get_waypoint_buffer().clear() # clear waypoint buffer at start
      dt = .200
      numlanes = 4
      numcars = 4
      Traffic_Tracker = Traffic(dt,numlanes,numcars=4,map_length=200,x_initial=self.spawn_x,y_initial=self.spawn_y,v_initial=self.spawn_v)
    def get_four_lane_waypoints_dict(self):
      grp = GlobalRoutePlanner(self.world.get_map(), 2)
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



    def add_member(self, vehicle_manager, leader=False):
        """
        Add memeber to the current platooning

        Parameters
        __________
        leader : boolean
            Indicator of whether this cav is a leader.

        vehicle_manager : opencda object
            The vehicle manager class.
        """
        self.vehicle_manager_list.append(vehicle_manager)

    def get_route_waypoints(self, destination):
      self.start_waypoint = self._map.get_waypoint(start_location)

        # make sure the start waypoint is behind the vehicle
        if self._ego_pos:
            cur_loc = self._ego_pos.location
            cur_yaw = self._ego_pos.rotation.yaw
            _, angle = cal_distance_angle(
                self.start_waypoint.transform.location, cur_loc, cur_yaw)

            while angle > 90:
                self.start_waypoint = self.start_waypoint.next(1)[0]
                _, angle = cal_distance_angle(
                    self.start_waypoint.transform.location, cur_loc, cur_yaw)

        end_waypoint = self._map.get_waypoint(end_location)
        if end_reset:
            self.end_waypoint = end_waypoint

        route_trace = self._trace_route(self.start_waypoint, end_waypoint)

    def get_map_waypoints(self):
      
      

    def set_destination(self, destination):
        """
        Set destination of the vehicle managers in the platoon.
        """
        self.destination = destination
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].set_destination(
                self.vehicle_manager_list[i].vehicle.get_location(),
                destination, clean=True)

    def update_information(self):
        """
        Update CAV world information for every member in the list.
        """
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].update_info()
        print("Updated Info")

    def algorithm_step(self):
        self.locations = []
        print("started Algo step")
        slice_list, vel_array, lanechange_command = get_slices_clustered(Traffic_Tracker, numcars)

        for i in range(len(slice_list)-1,-1,-1): #Iterate through all slices
            if len(slice_list[i]) >= 2: #If the slice has more than one vehicle, run the graph planner. Else it'll move using existing
            #responses - slow down on seeing a vehicle ahead that has slower velocities, else hit target velocity. 
            #Somewhat suboptimal, ideally the other vehicle would be
            #folded into existing groups. No easy way to do that yet.
                print("Slicing")
                a_star = AStarPlanner(slice_list[i], ov, oy, grid_size, robot_radius, Traffic_Tracker.cars_on_road, i)
                rv, ry, rx_tracked = a_star.planning()
                if len(ry) >= 2: #If there is some planner result, then we move ahead on using it
                    lanechange_command[i] = ry[-2]
                    vel_array[i] = rv[-2]
                else: #If the planner returns an empty list, continue as before - use emergency responses.
                    lanechange_command[i] = ry[0]
                    vel_array[i] = ry[0]

        print("Sliced")
        for i in range(len(slice_list)-1,-1,-1): #Relay lane change commands and new velocities to vehicles where needed
            if len(slice_list[i]) >= 1 and len(lanechange_command[i]) >= 1:
                carnum = 0
                for car in slice_list[i]:
                    if lanechange_command[i][carnum] > car.lane:
                        car.intentions = "Lane Change 1"
                    elif lanechange_command[i][carnum] < car.lane:
                        car.intentions = "Lane Change -1"
                    car.v = vel_array[i][carnum]
                    carnum += 1

        Traffic_Tracker.time_tick(mode='Graph') #Tick the simulation

        print("Success capsule")

        #Recording location and state
        x_states, y_states, tv, v = Traffic_Tracker.ret_car_locations()
        xcars = np.hstack((xcars, x_states))
        ycars = np.hstack((ycars, y_states))
        target_velocities = np.hstack((target_velocities,tv))
        velocities = np.hstack((velocities,v))
        ###########################################
 
        waypoints_rev = {1 : np.empty((2,0)), 2 : np.empty((2,0)), 3 : np.empty((2,0)), 4 : np.empty((2,0))}
        for i in range(1,xcars.shape[1]):
          processed_array = []
          for j in range(0,numcars):
            x_res = xcars[j,i]
            y_res = ycars[j,i]
            processed_array.append(np.array([[x_res],[y_res]]))
          back = processor.process_back(processed_array)
          waypoints_rev[1] = np.hstack((waypoints_rev[1],back[0]))
          waypoints_rev[2] = np.hstack((waypoints_rev[2],back[1]))
          waypoints_rev[3] = np.hstack((waypoints_rev[3],back[2]))
          waypoints_rev[4] = np.hstack((waypoints_rev[4],back[3]))

        print(waypoints_rev)
        car_locations = {1 : [], 2 : [], 3 : [], 4 : []}

        for car, car_array in waypoints_rev.items():
          for i in range(0,len(car_array[0])):
            location = carla.Transform(carla.Location(x=car_array[0][i], y=car_array[1][i], z=0.0), carla.Rotation())
            self.locations.append(location)



 
    def run_step(self):
        """
        Run one control step for each vehicles.

        Returns
        -------
        control_list : list
            The control command list for all vehicles.
        """

        # run algorithm
        self.algorithm_step()
        print("completed Algorithm Step")
        # output algorithm waypoints to waypoint buffer of each vehicle
        i = 0
        for vehicle_manager in vehicle_manager_list:
          waypoint_buffer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
          waypoint_buffer.clear()
          waypoint_buffer.append(waypoints_rev[i])
          i += 1
          

        control_list = []
        for i in range(len(self.vehicle_manager_list)):
            control = self.vehicle_manager_list[i].run_step(
                self.target_speed)
            control_list.append(control)

        for (i, control) in enumerate(control_list):
            self.vehicle_manager_list[i].vehicle.apply_control(control)

        return control_list

    def evaluate(self):
        """
        Used to save all members' statistics.

        Returns
        -------
        figure : matplotlib.figure
            The figure drawing performance curve passed back to save to
            the disk.

        perform_txt : str
            The string that contains all evaluation results to print out.
        """

        velocity_list = []
        acceleration_list = []
        time_gap_list = []
        distance_gap_list = []

        perform_txt = ''

        for i in range(len(self.vehicle_manager_list)):
            vm = self.vehicle_manager_list[i]
            debug_helper = vm.agent.debug_helper

            # we need to filter out the first 100 data points
            # since the vehicles spawn at the beginning have
            # no velocity and thus make the time gap close to infinite

            velocity_list += debug_helper.speed_list
            acceleration_list += debug_helper.acc_list
            time_gap_list += debug_helper.time_gap_list
            distance_gap_list += debug_helper.dist_gap_list

            time_gap_list_tmp = \
                np.array(debug_helper.time_gap_list)
            time_gap_list_tmp = \
                time_gap_list_tmp[time_gap_list_tmp < 100]
            distance_gap_list_tmp = \
                np.array(debug_helper.dist_gap_list)
            distance_gap_list_tmp = \
                distance_gap_list_tmp[distance_gap_list_tmp < 100]

            perform_txt += '\n Platoon member ID:%d, Actor ID:%d : \n' % (
                i, vm.vehicle.id)
            perform_txt += 'Time gap mean: %f, std: %f \n' % (
                np.mean(time_gap_list_tmp), np.std(time_gap_list_tmp))
            perform_txt += 'Distance gap mean: %f, std: %f \n' % (
                np.mean(distance_gap_list_tmp), np.std(distance_gap_list_tmp))

        figure = plt.figure()

        plt.subplot(411)
        open_plt.draw_velocity_profile_single_plot(velocity_list)

        plt.subplot(412)
        open_plt.draw_acceleration_profile_single_plot(acceleration_list)

        plt.subplot(413)
        open_plt.draw_time_gap_profile_singel_plot(time_gap_list)

        plt.subplot(414)
        open_plt.draw_dist_gap_profile_singel_plot(distance_gap_list)

        label = []
        for i in range(1, len(velocity_list) + 1):
            label.append('Leading Vehicle, id: %d' %
                         int(i - 1) if i == 1 else 'Platoon member, id: %d' %
                         int(i - 1))

        figure.legend(label, loc='upper right')

        return figure, perform_txt

    def destroy(self):
        """
        Destroy edge vehicles actors inside simulation world.
        """
        for vm in self.vehicle_manager_list:
            vm.destroy()
