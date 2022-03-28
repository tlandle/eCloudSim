# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import uuid
import subprocess
import random
import zmq

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
            config_file,
            application,
            carla_map,
            cav_world,
            carla_version,
            current_time='',
            data_dumping=False):

        config_yaml = load_yaml(config_file)
        cav_config = config_yaml['scenario']['single_cav_list'][vehicle_index]

        self.initialize_process(config_yaml)

        # an unique uuid for this vehicle
#        self.vid = str(uuid.uuid1())
        self.carla_map = carla_map

        # Use zmq for interprocess communication between OpenCDA and each vehicle
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)

        print(f"OpenCDA: Spawning process {config_file}...")
#        self.client_process = subprocess.Popen(f"python3 -u ./opencda/core/common/vehicle_manager.py -t {config_file} -i {vehicle_index} -a {application} -v {carla_version}", shell=True)

        # Connect to the vehicle and send the START message
        self._socket.connect(f"tcp://localhost:{vehicle_index+5555}")

        message =   {
                        "cmd": "start",
                        "params": {
                            "scenario": config_file,
                            "vehicle": vehicle_index,
                            "application": application,
                            "version": carla_version
                        } 
                    }
        self._socket.send_json(message)
        message = self._socket.recv_json()
        print(f"OpenCDA: received {message}")
        actor_id = message["actor_id"] # Vehicle sends back the actor id so we can get a handle to the actor in Carla        
        self.vid = message["vid"] # Vehicle sends back the uuid id we use as unique identifier

        vehicle = self.world.get_actor(actor_id)
        self.vehicle = vehicle

        # retrieve the configure for different modules
        sensing_config = cav_config['sensing']
        behavior_config = cav_config['behavior']
        control_config = cav_config['controller']
        v2x_config = cav_config['v2x']
        # v2x module
        self.v2x_manager = V2XManager(cav_world, v2x_config, self.vid)
        # localization module
        self.localizer = LocalizationManager(
            vehicle, sensing_config['localization'], carla_map)
        # perception module
        self.perception_manager = PerceptionManager(
            vehicle, sensing_config['perception'], cav_world,
            data_dumping)

        cav_world.update_vehicle_manager(self)

    def initialize_process(self, config_yaml):
        simulation_config = config_yaml['world']

        # set random seed if stated
        if 'seed' in simulation_config:
            np.random.seed(simulation_config['seed'])
            random.seed(simulation_config['seed'])

        self.client = \
            carla.Client('localhost', simulation_config['client_port'])
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
#        message = f"set_destination {start_location} {end_location} {clean} {end_reset}"
        self._socket.send(b"set_destination")
        self._socket.recv()
        message = { "start": {"x": start_location.x, "y": start_location.y, "z": start_location.z},
                    "end": {"x": end_location.x, "y": end_location.y, "z": end_location.z},
                    "clean": clean, "reset": end_reset}
        print("OpenCDA: %s" % message)
        self._socket.send_pyobj(message)
        self._socket.recv()
        return

        self.agent.set_destination(
            start_location, end_location, clean, end_reset)

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """

        print("OpenCDA: update_info called")
        self._socket.send(b"update_info")
        self._socket.recv()
        return 
        # localization
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection
        objects = self.perception_manager.detect(ego_pos)

        # update ego position and speed to v2x manager,
        # and then v2x manager will search the nearby cavs
        self.v2x_manager.update_info(ego_pos, ego_spd)

        self.agent.update_information(ego_pos, ego_spd, objects)
        # pass position and speed info to controller
        self.controller.update_info(ego_pos, ego_spd)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """
        print("OpenCDA: entered run_step")
        target_speed, target_pos = self.agent.run_step(target_speed)
        control = self.controller.run_step(target_speed, target_pos)

        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      self.agent)

        print("OpenCDA: after run_step")
        return control

    def apply_control(self, control):
        """
        TODO ecloud Added to separate Carla vehicle access from the scenario
        Apply the controls to the vehicle
        """
        self.vehicle.apply_control(control)

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
