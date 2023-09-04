# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import random
import uuid
import opencda.logging_ecloud
import logging
import time
import random

import carla
import numpy as np

from opencda.core.common.cav_world import CavWorld
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

import coloredlogs, logging
logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]

class VehicleManager(object):
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
            vehicle=None,
            config_yaml=None,
            vehicle_index=None,
            config_file=None,
            application=['single'],
            carla_map=None,
            cav_world=None,
            carla_version='0.9.12',
            current_time='',
            data_dumping=False):

        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())

        # ORIGINAL FLOW

        if vehicle != None and config_yaml != None and carla_map != None:

            cav_config = config_yaml
            self.vehicle = vehicle
            self.carla_map = carla_map
            self.run_distributed = False

        # eCLOUD BEGIN

        elif vehicle_index != None:

            self.run_distributed = True
            if config_file != None:
                self.scenario_params = load_yaml(config_file)
            elif config_yaml != None:
                self.scenario_params = config_yaml
            else:
                assert(False, "need to pass a YAML file or dict")

            self.initialize_process()
            self.carla_version = carla_version

            # By default, we use lincoln as our cav model.
            default_model = 'vehicle.lincoln.mkz2017' \
                if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'

            cav_vehicle_bp = \
                self.world.get_blueprint_library().find(default_model)

            # if the spawn position is a single scalar, we need to use map
            # helper to transfer to spawn transform
            if 'single_cav_list' in self.scenario_params['scenario']:
                cav_config = self.scenario_params['scenario']['single_cav_list'][vehicle_index]
            elif 'edge_list' in self.scenario_params['scenario']:
                # TODO: support multiple edges... 
                cav_config = self.scenario_params['scenario']['edge_list'][0]['members'][vehicle_index]
                logger.debug(cav_config)
            else:
                assert(False, "no known vehicle indexing format found")
            
            spawned = False
            while not spawned:
                try:
                    if 'spawn_explicit' in cav_config:
                        self.spawn_transform = carla.Transform(
                        carla.Location(
                            x=cav_config['spawn_position'][0],
                            y=cav_config['spawn_position'][1],
                            z=cav_config['spawn_position'][2]),
                        carla.Rotation(
                            pitch=cav_config['spawn_position'][5],
                            yaw=cav_config['spawn_position'][4],
                            roll=cav_config['spawn_position'][3]))    
                    elif 'spawn_special' not in cav_config:
                        spawn_points = self.world.get_map().get_spawn_points()
                        self.spawn_transform = spawn_points[random.randint(0, len(spawn_points))]
                        self.spawn_location = carla.Location(
                                x=self.spawn_transform.location.x,
                                y=self.spawn_transform.location.y,
                                z=self.spawn_transform.location.z)
                    elif config_file != None:
                        assert( False, "['spawn_special'] not supported in eCloud currently")

                    self.cav_destination = {}
                    self.cav_destination['x'] = cav_config['destination'][0]
                    self.cav_destination['y'] = cav_config['destination'][1]

                    cav_vehicle_bp.set_attribute('color', '0, 0, 255')
                    self.vehicle = self.world.spawn_actor(cav_vehicle_bp, self.spawn_transform)
                    spawned = True
                except Exception as e:
                    logger.debug("spawn collision. retrying")
                    continue
            # teleport vehicle to desired spawn point
            # self.vehicle.set_transform(spawn_transform)
            # self.world.tick()

        else:
            assert( False, "need to provide some known config" )

        # eCLOUD END    

        self.debug_helper = ClientDebugHelper(0)
        # retrieve the configure for different modules
        sensing_config = cav_config['sensing']
        behavior_config = cav_config['behavior']
        control_config = cav_config['controller']
        v2x_config = cav_config['v2x']

        # v2x module
        self.v2x_manager = V2XManager(cav_world, v2x_config, self.vid)
        # localization module
        self.localizer = LocalizationManager(
            self.vehicle, sensing_config['localization'], self.carla_map)
        # perception module
        self.perception_manager = PerceptionManager(
            self.vehicle, sensing_config['perception'], cav_world,
            data_dumping)

        # behavior agent
        self.agent = None
        if 'platooning' in application:
            platoon_config = cav_config['platoon']
            self.agent = PlatooningBehaviorAgent(
                self.vehicle,
                self,
                self.v2x_manager,
                behavior_config,
                platoon_config,
                self.carla_map)
        else:
            self.agent = BehaviorAgent(self.vehicle, self.carla_map, behavior_config, is_dist=self.run_distributed)

        # Control module
        self.controller = ControlManager(control_config)

        if data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          self.vehicle.id,
                                          save_time=current_time)
        else:
            self.data_dumper = None

        cav_world.update_vehicle_manager(self)

    def is_close_to_scenario_destination(self):
        """
        Check if the current ego vehicle's position is close to destination

        Returns
        -------
        flag : boolean
            It is True if the current ego vehicle's position is close to destination

        """
        ego_pos = self.vehicle.get_location()
        flag = abs(ego_pos.x - self.cav_destination['x']) <= 10 and \
            abs(ego_pos.y - self.cav_destination['y']) <= 10
        return flag

    def initialize_process(self):
        simulation_config = self.scenario_params['world']

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

        self.agent.set_destination(
            start_location, end_location, clean, end_reset)

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """
        # localization
        start_time = time.time()
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()
        end_time = time.time()
        logging.debug("Localizer time: %s" %(end_time - start_time)) 
        self.debug_helper.update_localization_time((end_time-start_time)*1000)

        # object detection
        start_time = time.time()
        objects = self.perception_manager.detect(ego_pos)
        end_time = time.time()
        logging.debug("Perception time: %s" %(end_time - start_time))
        self.debug_helper.update_perception_time((end_time-start_time)*1000)



        # update ego position and speed to v2x manager,
        # and then v2x manager will search the nearby cavs
        start_time = time.time()
        self.v2x_manager.update_info(ego_pos, ego_spd)
        end_time = time.time()
        logging.debug("v2x manager update info time: %s" %(end_time - start_time)) 

        start_time = time.time()
        self.agent.update_information(ego_pos, ego_spd, objects)
        end_time = time.time()
        logging.debug("Agent Update info time: %s" %(end_time - start_time))
        self.debug_helper.update_agent_update_info_time((end_time-start_time)*1000)

        # pass position and speed info to controller
        start_time = time.time()
        self.controller.update_info(ego_pos, ego_spd)
        end_time = time.time()
        logging.debug("Controller update time: %s" %(end_time - start_time))
        self.debug_helper.update_controller_update_info_time((end_time-start_time)*1000)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """

        # eCLOUD - must check FIRST to ensure sim doesn't try to progress a DONE vehicle
        if target_speed == -1 and self.run_distributed:
            logger.info("run_step: simulation is over")
            return None # -1 indicates the simulation is over. TODO Need a const here.

        pre_vehicle_step_time = time.time()
        target_speed, target_pos = self.agent.run_step(target_speed)
        end_time = time.time()
        logging.debug("Agent step time: %s" %(end_time - pre_vehicle_step_time))

        control = self.controller.run_step(target_speed, target_pos)
        post_vehicle_step_time = time.time()
        logging.debug("Controller step time: %s" %(post_vehicle_step_time - end_time))
        logging.debug("Vehicle step time: %s" %(post_vehicle_step_time - pre_vehicle_step_time))
        self.debug_helper.update_controller_step_time((post_vehicle_step_time - end_time)*1000)
        self.debug_helper.update_vehicle_step_time((post_vehicle_step_time - pre_vehicle_step_time)*1000)
        self.debug_helper.update_agent_step_time((end_time - pre_vehicle_step_time)*1000)        
 
        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      self.agent)

        return control

    def apply_control(self, control):
        """
        Apply the controls to the vehicle
        """
        start_time = time.time()
        self.vehicle.apply_control(control)
        end_time = time.time()
        self.debug_helper.update_control_time((end_time - start_time)*1000)

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
