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

RESULT_SUCCESS = 0 # Step ran ok
RESULT_ERROR = 1 # Step resulted in an error
RESULT_END = 2 # Step resulted in the vehicle simulation ending

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]

class VehicleManagerProxy(object):
    """
    A class manager to embed different modules with vehicle together.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    config_yaml : dict
        The configuration dictionary of this CAV.

    application : list
        The application category, currently support:['single','platoon'].

    carla_map : carla.Map
        The CARLA simulation map.

    cav_world : opencda object
        CAV World.

    current_time : str
        Timestamp of the simulation beginning.

    data_dumping : bool
        Indicates whether to dump sensor data during simulation.

    Attributes
    ----------
    v2x_manager : opencda object
        The current V2X manager.

    localizer : opencda object
        The current localization manager.

    perception_manager : opencda object
        The current V2X perception manager.

    agent : opencda object
        The current carla agent that handles the basic behavior
         planning of ego vehicle.

    controller : opencda object
        The current control manager.

    data_dumper : opencda object
        Used for dumping sensor data.
    """

    def __init__(
            self,
            vehicle_index,
            #conn,
            config_file,
            application,
            carla_map,
            cav_world,
            carla_version,
            current_time='',
            data_dumping=False):

        config_yaml = load_yaml(config_file)
        self.cav_config = config_yaml['scenario']['single_cav_list'][vehicle_index]
        self.cav_world = cav_world
        self.data_dumping = data_dumping

        self.initialize_process(config_yaml)

        # an unique uuid for this vehicle
#        self.vid = str(uuid.uuid1())
        self.carla_map = carla_map

        # Use sockets for interprocess communication between OpenCDA and each vehicle
        #self._socket = conn

    def start_vehicle(self, actor_id, vid):
        # Send the START message to the vehicle with simulation parameters       
        self.vid = vid # message["vid"] # Vehicle sends back the uuid id we use as unique identifier

        print("eCloud debug | actor_id: " + str(actor_id))

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
        # perception module
        self.perception_manager = PerceptionManager(
            vehicle, sensing_config['perception'], self.cav_world,
            self.data_dumping)

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

    def set_destination(
            self,
            start_location,
            end_location,
            clean=False,
            end_reset=True):
        """
        Set global route.

        Parameters
        ----------
        start_location : carla.location
            The CAV start location.

        end_location : carla.location
            The CAV destination.

        clean : bool
             Indicator of whether clean waypoint queue.

        end_reset : bool
            Indicator of whether reset the end location.

        Returns
        -------
        """
        print("OpenCDA: set_destination")

        message = { "cmd": "set_destination",
                    "params": {
                    "start": {"x": start_location.x, "y": start_location.y, "z": start_location.z},
                    "end": {"x": end_location.x, "y": end_location.y, "z": end_location.z},
                    "clean": clean, "reset": end_reset
                    }
        }
        self._socket.send(json.dumps(message).encode('utf-8'))
        resp = json.loads(self._socket.recv(1024).decode('utf-8'))
        return

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """
        print("OpenCDA: update_info called")

        self._socket.send(json.dumps({"cmd": "update_info"}).encode('utf-8'))
        resp = json.loads(self._socket.recv(1024).decode('utf-8'))
        return 

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """
        print("OpenCDA: run_step")

        target_speed, target_pos = self.agent.run_step(target_speed)
        control = self.controller.run_step(target_speed, target_pos)

        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      self.agent)

        return control

#    def apply_control(self, control):
#        """
#        Apply the controls to the vehicle
#        """
#        self.vehicle.apply_control(control)

    def do_tick(self):
        """
        Tells the vehicle that it should advance a single step in the simulation
        """
        success = RESULT_SUCCESS
        self._socket.send(json.dumps({"cmd": "TICK"}).encode('utf-8'))
        resp = json.loads(self._socket.recv(1024).decode('utf-8'))
        if (resp["resp"] == "DONE"):
            success = RESULT_END
        return success

    def end_step(self):
        """
        Sends the END command to the vehicle to tell it to end gracefully
        """
        self._socket.send(json.dumps({"cmd": "END"}).encode('utf-8'))
        resp = json.loads(self._socket.recv(1024).decode('utf-8'))

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
        self._socket.close()
