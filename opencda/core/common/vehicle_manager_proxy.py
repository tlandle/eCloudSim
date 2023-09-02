# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import uuid
import subprocess
import random
import socket
import json
import psutil

import carla
import numpy as np

from opencda.core.actuation.control_manager \
    import ControlManager
from opencda.core.application.platooning.platoon_behavior_agent\
    import PlatooningBehaviorAgent
from opencda.core.common.v2x_manager \
    import V2XManager
from opencda.core.sensing.localization.localization_manager \
    import LocalizationManager
from opencda.core.sensing.perception.perception_manager \
    import PerceptionManager
from opencda.core.plan.behavior_agent \
    import BehaviorAgent
from opencda.core.common.data_dumper import DataDumper
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.client_debug_helper import ClientDebugHelper

RESULT_SUCCESS = 0 # Step ran ok
RESULT_ERROR = 1 # Step resulted in an error
RESULT_END = 2 # Step resulted in the vehicle simulation ending

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]

class VehicleManagerProxy(object):
    """
    TODO: update
    """

    def __init__(
            self,
            vehicle_index,
            #conn,
            config_yaml,
            application,
            carla_map,
            cav_world,
            carla_version,
            current_time='',
            data_dumping=False):

        if 'single_cav_list' in config_yaml['scenario']:
            self.cav_config = config_yaml['scenario']['single_cav_list'][vehicle_index]
        elif 'edge_list' in config_yaml['scenario']:
            # TODO: support multiple edges...
            self.cav_config = config_yaml['scenario']['edge_list'][0]['members'][vehicle_index]
        else:
            assert(False, "no known vehicle indexing format found")
        self.cav_world = cav_world
        self.data_dumping = data_dumping
        self.application = application
        self.current_time = current_time

        self.initialize_process(config_yaml)

        # an unique uuid for this vehicle
#        self.vid = str(uuid.uuid1())
        self.carla_map = carla_map

        # Use sockets for interprocess communication between OpenCDA and each vehicle
        #self._socket = conn
        self.debug_helper = ClientDebugHelper(0)

    def start_vehicle(self, actor_id, vid):
        # Send the START message to the vehicle with simulation parameters
        self.vid = vid # message["vid"] # Vehicle sends back the uuid id we use as unique identifier

        # print("eCloud debug | actor_id: " + str(actor_id))

        vehicle = self.world.get_actor(actor_id)
        self.vehicle = vehicle

        # retrieve the configure for different modules
        sensing_config = self.cav_config['sensing']
        behavior_config = self.cav_config['behavior']
        control_config = self.cav_config['controller']
        v2x_config = self.cav_config['v2x']
        # v2x module
        self.v2x_manager = V2XManager(self.cav_world, v2x_config, self.vid)
        # localization module
        self.localizer = LocalizationManager(
            vehicle, sensing_config['localization'], self.carla_map)
        # perception module - proxy should never use perception
        sensing_config['perception']['activate'] = False
        sensing_config['perception']['camera_visualize'] = False
        sensing_config['perception']['lidar_visualize'] = False
        self.perception_manager = PerceptionManager(
            vehicle, sensing_config['perception'], self.cav_world,
            self.data_dumping)

        # behavior agent
        self.agent = None
        if 'platooning' in self.application:
            platoon_config = self.cav_config['platoon']
            self.agent = PlatooningBehaviorAgent(
                self.vehicle,
                self,
                self.v2x_manager,
                behavior_config,
                platoon_config,
                self.carla_map)
        else:
            self.agent = BehaviorAgent(self.vehicle, self.carla_map, behavior_config)

        # Control module
        self.controller = ControlManager(control_config)

        if self.data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          self.vehicle.id,
                                          save_time=self.current_time)
        else:
            self.data_dumper = None

        self.cav_world.update_vehicle_manager(self)

    def initialize_process(self, config_yaml):
        simulation_config = config_yaml['world']

        # set random seed if stated
        if 'seed' in simulation_config:
            np.random.seed(simulation_config['seed'])
            random.seed(simulation_config['seed'])

        self.client = \
            carla.Client(CARLA_IP, simulation_config['client_port'])
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

    def is_process_running(self, processName):
        '''
        Check if there is any running process that contains the given name processName.
        '''
        #Iterate over the all the running process
        for proc in psutil.process_iter():
            try:
                # Check if process name contains the given name string.
                if processName.lower() in proc.name().lower():
                    return True

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass

        print(f"{processName} is no longer running...")
        return False

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        if self.is_process_running("CarlaUE4"):
            self.perception_manager.destroy()
            self.localizer.destroy()
            self.vehicle.destroy()
            #self._socket.close()
