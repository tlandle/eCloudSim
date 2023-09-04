# -*- coding: utf-8 -*-
"""
Utilize scenario manager to manage CARLA simulation construction. This script
is used for carla simulation only, and if you want to manage the Co-simulation,
please use cosim_api.py.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
from queue import Queue
import random
from sqlite3 import connect
import sys
from random import shuffle
import socket
import time
import json
import random
import copy
import hashlib
import os
import asyncio
import subprocess
import signal

# gRPC
from concurrent.futures import ThreadPoolExecutor, thread
import coloredlogs, logging
import threading
import time
from typing import Iterable
from queue import Queue
import heapq
from google.protobuf.timestamp_pb2 import Timestamp

from google.protobuf.json_format import MessageToJson
import grpc

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc
#end gRPC

import carla
import numpy as np
import pandas as pd
import pickle

import matplotlib.pyplot as plt
#import k_means_constrained

from opencda.core.common.vehicle_manager_proxy import VehicleManagerProxy
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.core.application.platooning.platooning_manager import \
    PlatooningManager
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.customized_map_api import \
    load_customized_world, bcolors
from opencda.core.application.edge.edge_manager import \
     EdgeManager
from opencda.sim_debug_helper import SimDebugHelper
from opencda.client_debug_helper import ClientDebugHelper
from opencda.scenario_testing.utils.yaml_utils import load_yaml
import opencda.core.plan.drive_profile_plotting as open_plt
logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

TIMEOUT_S = 10
TIMEOUT_MS = TIMEOUT_S * 1000

def car_blueprint_filter(blueprint_library, carla_version='0.9.11'):
    """
    Exclude the uncommon vehicles from the default CARLA blueprint library
    (i.e., isetta, carlacola, cybertruck, t2).

    Parameters
    ----------
    blueprint_library : carla.blueprint_library
        The blueprint library that contains all models.

    carla_version : str
        CARLA simulator version, currently support 0.9.11 and 0.9.12. We need
        this as since CARLA 0.9.12 the blueprint name has been changed a lot.

    Returns
    -------
    blueprints : list
        The list of suitable blueprints for vehicles.
    """

    if carla_version == '0.9.11':
        logger.debug('old version')
        blueprints = [
            blueprint_library.find('vehicle.audi.a2'),
            blueprint_library.find('vehicle.audi.tt'),
            blueprint_library.find('vehicle.dodge_charger.police'),
            blueprint_library.find('vehicle.jeep.wrangler_rubicon'),
            blueprint_library.find('vehicle.chevrolet.impala'),
            blueprint_library.find('vehicle.mini.cooperst'),
            blueprint_library.find('vehicle.audi.etron'),
            blueprint_library.find('vehicle.mercedes-benz.coupe'),
            blueprint_library.find('vehicle.bmw.grandtourer'),
            blueprint_library.find('vehicle.toyota.prius'),
            blueprint_library.find('vehicle.citroen.c3'),
            blueprint_library.find('vehicle.mustang.mustang'),
            blueprint_library.find('vehicle.tesla.model3'),
            blueprint_library.find('vehicle.lincoln.mkz2017'),
            blueprint_library.find('vehicle.seat.leon'),
            blueprint_library.find('vehicle.nissan.patrol'),
            blueprint_library.find('vehicle.nissan.micra'),
        ]

    else:
        blueprints = [
            blueprint_library.find('vehicle.audi.a2'),
            blueprint_library.find('vehicle.audi.tt'),
            blueprint_library.find('vehicle.dodge.charger_police'),
            blueprint_library.find('vehicle.dodge.charger_police_2020'),
            blueprint_library.find('vehicle.dodge.charger_2020'),
            blueprint_library.find('vehicle.jeep.wrangler_rubicon'),
            blueprint_library.find('vehicle.chevrolet.impala'),
            blueprint_library.find('vehicle.mini.cooper_s'),
            blueprint_library.find('vehicle.audi.etron'),
            blueprint_library.find('vehicle.mercedes.coupe'),
            blueprint_library.find('vehicle.mercedes.coupe_2020'),
            blueprint_library.find('vehicle.bmw.grandtourer'),
            blueprint_library.find('vehicle.toyota.prius'),
            blueprint_library.find('vehicle.citroen.c3'),
            blueprint_library.find('vehicle.ford.mustang'),
            blueprint_library.find('vehicle.tesla.model3'),
            blueprint_library.find('vehicle.lincoln.mkz_2017'),
            blueprint_library.find('vehicle.lincoln.mkz_2020'),
            blueprint_library.find('vehicle.seat.leon'),
            blueprint_library.find('vehicle.nissan.patrol'),
            blueprint_library.find('vehicle.nissan.micra'),
        ]

    return blueprints

class ScenarioManager:
    """
    The manager that controls simulation construction, backgound traffic
    generation and CAVs spawning.

    Parameters
    ----------
    scenario_params : dict
        The dictionary contains all simulation configurations.

    carla_version : str
        CARLA simulator version, it currently supports 0.9.11 and 0.9.12

    xodr_path : str
        The xodr file to the customized map, default: None.

    town : str
        Town name if not using customized map, eg. 'Town06'.

    apply_ml : bool
        Whether need to load dl/ml model(pytorch required) in this simulation.

    Attributes
    ----------
    client : carla.client
        The client that connects to carla server.

    world : carla.world
        Carla simulation server.

    origin_settings : dict
        The origin setting of the simulation server.

    cav_world : opencda object
        CAV World that contains the information of all CAVs.

    carla_map : carla.map
        Carla HD Map.

    """

    tick_id = 0 # current tick counter
    waypoint_buffer_overrides = []

    vehicle_managers = {}
    vehicles = {} # vehicle_index -> tuple (actor_id, vid)
    vehicle_count = 0

    carla_version = None
    application = ['single']
    scenario = None
    ecloud_server = None

    debug_helper = SimDebugHelper(0)

    SHORT_SLEEP = 0.005

    async def server_unpack_debug_data(self, stub_):
        ecloud_update = await stub_.Server_GetVehicleUpdates(ecloud.Empty())
        for vehicle_update in ecloud_update.vehicle_update:
            vehicle_manager_proxy = ScenarioManager.vehicle_managers[ vehicle_update.vehicle_index ]
            vehicle_manager_proxy.localizer.debug_helper.deserialize_debug_info( vehicle_update.loc_debug_helper )
            vehicle_manager_proxy.agent.debug_helper.deserialize_debug_info( vehicle_update.planer_debug_helper )
            vehicle_manager_proxy.debug_helper.deserialize_debug_info(vehicle_update.client_debug_helper)

    async def server_do_tick(self, stub_, update_):
        response = await stub_.Server_DoTick(update_)
        start_time = time.time()
        count = 1
        while 1:
            await asyncio.sleep(ScenarioManager.SHORT_SLEEP)
            ping = await stub_.Server_Ping(ecloud.Empty())
            if ping.tick_id == 1:
                end_time = time.time()
                logger.info(f"polled {count} times over a total of {(end_time - start_time)*1000}ms")
                break

            if update_.command == ecloud.Command.REQUEST_DEBUG_INFO:
                await self.server_unpack_debug_data(stub_)

            count += 1

        return response
    
    async def server_start_scenario(self, stub_, update_):
        await stub_.Server_StartScenario(update_)
        start_time = time.time()
        logger.info(f"pushed scenario start")

        count = 1
        registered_vehicles = 0
        while 1:
            ping = await stub_.Server_Ping(ecloud.Empty())
            await asyncio.sleep(ScenarioManager.SHORT_SLEEP)
            if count % 20 == 0:
                logger.info(f"waiting for registration to complete")

            if ping.tick_id > registered_vehicles:
                registered_vehicles = ping.tick_id
                logger.info(f"{registered_vehicles} have been registered - ticking world")
                self.tick_world()

            if ping.tick_id == self.vehicle_count:
                end_time = time.time()
                logger.info(f"polled {count} times over a total of {(end_time - start_time)*1000}ms")
                self.tick_world()
                break
            count += 1

        logger.info(f"vehicle registration complete")

        response = await stub_.Server_GetVehicleUpdates(ecloud.Empty())

        logger.info(f"vehicle registration data received")

        return response

    async def server_end_scenario(self, stub_, update_):
        response = await stub_.Server_EndScenario(update_)
    
        return response

    debug_helper = SimDebugHelper(0)
    
    def __init__(self, scenario_params,
                 apply_ml,
                 carla_version,
                 xodr_path=None,
                 town=None,
                 cav_world=None,
                 config_file=None):
                 
        server_log_level = 1 if logger.getEffectiveLevel() == logging.DEBUG else 0
        try:
            ecloud_pid = subprocess.check_output(['pgrep','ecloud'])
        except subprocess.CalledProcessError as e:
            if e.returncode > 1:
                raise
            ecloud_pid = None
        if ecloud_pid != None:
            logger.info(f'killing exiting ecloud gRPC server process')
            subprocess.run(['pkill','-9','ecloud'])

        self.ecloud_server_process = subprocess.Popen(['./ecloud',f'--log_level={server_log_level}'])

        self.scenario_params = scenario_params
        self.carla_version = carla_version
        self.config_file = config_file
        self.perception = scenario_params['perception_active'] if 'perception_active' in scenario_params else False
        self.run_distributed = scenario_params['distributed'] if 'distributed' in scenario_params else False

        simulation_config = scenario_params['world']
        
        cav_world.update_scenario_manager(self)

        random.seed(time.time())

        # set random seed if stated
        if 'seed' in simulation_config:
            np.random.seed(simulation_config['seed'])
            random.seed(simulation_config['seed'])

        self.client = \
            carla.Client(CARLA_IP, simulation_config['client_port'])
        self.client.set_timeout(10.0)

        if xodr_path:
            self.world = load_customized_world(xodr_path, self.client)
        elif town:
            try:
                self.world = self.client.load_world(town)
            except RuntimeError:
                logger.error(
                    f"{bcolors.FAIL} %s is not found in your CARLA repo! "
                    f"Please download all town maps to your CARLA "
                    f"repo!{bcolors.ENDC}" % town)
        else:
            self.world = self.client.get_world()

        if not self.world:
            sys.exit('World loading failed')

        self.origin_settings = self.world.get_settings()
        new_settings = self.world.get_settings()

        if simulation_config['sync_mode']:
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = \
                simulation_config['fixed_delta_seconds']
        else:
            sys.exit(
                'ERROR: Current version only supports sync simulation mode')

        self.world.apply_settings(new_settings)

        # set weather
        weather = self.set_weather(simulation_config['weather'])
        self.world.set_weather(weather)

        self.cav_world = cav_world
        self.carla_map = self.world.get_map()
        self.apply_ml = apply_ml

        # eCLOUD BEGIN
        if self.run_distributed:
            self.apply_ml = False
            if apply_ml == True:
                assert( False, "ML should only be run on the distributed clients")

            channel = grpc.aio.insecure_channel(
            target="[::]:50052",
            options=[
                ("grpc.lb_policy_name", "pick_first"),
                ("grpc.enable_retries", 0),
                ("grpc.keepalive_timeout_ms", TIMEOUT_MS),
                ],
            )
            self.ecloud_server = ecloud_rpc.EcloudStub(channel)

            if 'single_cav_list' in scenario_params['scenario']:
                self.vehicle_count = len(scenario_params['scenario']['single_cav_list'])
            elif 'edge_list' in scenario_params['scenario']:
                # TODO: support multiple edges...
                self.vehicle_count = len(scenario_params['scenario']['edge_list'][0]['members'])
            else:
                assert(False, "no known vehicle indexing format found")

            ScenarioManager.debug_helper.update_sim_start_timestamp(time.time())

            self.scenario = json.dumps(scenario_params) #self.config_file
            self.carla_version = self.carla_version

            server_request = ecloud.SimulationState()
            server_request.test_scenario = self.scenario
            server_request.application = self.application[0]
            server_request.version = self.carla_version
            server_request.state = ecloud.State.START
            server_request.tick_id = self.tick_id
            server_request.vehicle_index = self.vehicle_count # bit of a hack to use vindex as count here

            print("start vehicle containers")
            ecloud_update = asyncio.get_event_loop().run_until_complete(self.server_start_scenario(self.ecloud_server, server_request))

            logger.debug(f"unpacking ecloud_update...")

            # unpack the update - which will contain a repeated list of updates from the indivudal containers
            for vehicle_update in ecloud_update.vehicle_update:
                logger.debug(f"vehicle {vehicle_update.vehicle_index} | actor_id: {vehicle_update.actor_id} & vid: {vehicle_update.vid}")
                vehicle_tuple = ( vehicle_update.actor_id, vehicle_update.vid )
                self.vehicles[f"vehicle_{vehicle_update.vehicle_index}"] = vehicle_tuple

            self.world.tick()

            logger.debug("eCloud debug: pushed START")

        # eCLOUD END

    @staticmethod
    def set_weather(weather_settings):
        """
        Set CARLA weather params.

        Parameters
        ----------
        weather_settings : dict
            The dictionary that contains all parameters of weather.

        Returns
        -------
        The CARLA weather setting.
        """
        weather = carla.WeatherParameters(
            sun_altitude_angle=weather_settings['sun_altitude_angle'],
            cloudiness=weather_settings['cloudiness'],
            precipitation=weather_settings['precipitation'],
            precipitation_deposits=weather_settings['precipitation_deposits'],
            wind_intensity=weather_settings['wind_intensity'],
            fog_density=weather_settings['fog_density'],
            fog_distance=weather_settings['fog_distance'],
            fog_falloff=weather_settings['fog_falloff'],
            wetness=weather_settings['wetness']
        )
        return weather

    # BEGIN Core OpenCDA
    def create_vehicle_manager(self, application,
                               map_helper=None,
                               data_dump=False):
        """
        Create a list of single CAVs.
        Parameters
        ----------
        application : list
            The application purpose, a list, eg. ['single'], ['platoon'].
        map_helper : function
            A function to help spawn vehicle on a specific position in
            a specific map.
        data_dump : bool
            Whether to dump sensor data.
        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        print('Creating single CAVs.')
        # By default, we use lincoln as our cav model.
        default_model = 'vehicle.lincoln.mkz2017' \
            if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'

        cav_vehicle_bp = \
            self.world.get_blueprint_library().find(default_model)
        single_cav_list = []

        for i, cav_config in enumerate(
                self.scenario_params['scenario']['single_cav_list']):

            # if the spawn position is a single scalar, we need to use map
            # helper to transfer to spawn transform
            if 'spawn_special' not in cav_config:
                spawn_transform = carla.Transform(
                    carla.Location(
                        x=cav_config['spawn_position'][0],
                        y=cav_config['spawn_position'][1],
                        z=cav_config['spawn_position'][2]),
                    carla.Rotation(
                        pitch=cav_config['spawn_position'][5],
                        yaw=cav_config['spawn_position'][4],
                        roll=cav_config['spawn_position'][3]))
            else:
                spawn_transform = map_helper(self.carla_version,
                                             *cav_config['spawn_special'])

            cav_vehicle_bp.set_attribute('color', '0, 0, 255')
            vehicle = self.world.spawn_actor(cav_vehicle_bp, spawn_transform)

            # create vehicle manager for each cav
            vehicle_manager = VehicleManager(
                vehicle=vehicle, config_yaml=cav_config, application=application,
                carla_map=self.carla_map, cav_world=self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump)

            self.world.tick()

            vehicle_manager.v2x_manager.set_platoon(None)

            destination = carla.Location(x=cav_config['destination'][0],
                                         y=cav_config['destination'][1],
                                         z=cav_config['destination'][2])
            vehicle_manager.update_info()
            vehicle_manager.set_destination(
                vehicle_manager.vehicle.get_location(),
                destination,
                clean=True)

            single_cav_list.append(vehicle_manager)

        return single_cav_list

    def create_platoon_manager(self, map_helper=None, data_dump=False):
        """
        Create a list of platoons.

        Parameters
        ----------
        map_helper : function
            A function to help spawn vehicle on a specific position in a
            specific map.

        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        logger.info('Creating platoons/')
        platoon_list = []
        self.cav_world = CavWorld(self.apply_ml)

        # we use lincoln as default choice since our UCLA mobility lab use the
        # same car
        default_model = 'vehicle.lincoln.mkz2017' \
            if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'

        cav_vehicle_bp = \
            self.world.get_blueprint_library().find(default_model)

        # create platoons
        for i, platoon in enumerate(
                self.scenario_params['scenario']['platoon_list']):
            platoon_manager = PlatooningManager(platoon, self.cav_world)
            for j, cav in enumerate(platoon['members']):
                if 'spawn_special' not in cav:
                    spawn_transform = carla.Transform(
                        carla.Location(
                            x=cav['spawn_position'][0],
                            y=cav['spawn_position'][1],
                            z=cav['spawn_position'][2]),
                        carla.Rotation(
                            pitch=cav['spawn_position'][5],
                            yaw=cav['spawn_position'][4],
                            roll=cav['spawn_position'][3]))
                else:
                    spawn_transform = map_helper(self.carla_version,
                                                 *cav['spawn_special'])

                cav_vehicle_bp.set_attribute('color', '0, 0, 255')
                vehicle = self.world.spawn_actor(cav_vehicle_bp,
                                                 spawn_transform)

                # create vehicle manager for each cav
                vehicle_manager = VehicleManager(
                    vehicle, cav, ['platooning'],
                    self.carla_map, self.cav_world,
                    current_time=self.scenario_params['current_time'],
                    data_dumping=data_dump)

                # add the vehicle manager to platoon
                if j == 0:
                    platoon_manager.set_lead(vehicle_manager)
                else:
                    platoon_manager.add_member(vehicle_manager, leader=False)

            self.world.tick()
            destination = carla.Location(x=platoon['destination'][0],
                                         y=platoon['destination'][1],
                                         z=platoon['destination'][2])

            platoon_manager.set_destination(destination)
            platoon_manager.update_member_order()
            platoon_list.append(platoon_manager)

        return platoon_list


    def spawn_vehicles_by_list(self, tm, traffic_config, bg_list):
        """
        Spawn the traffic vehicles by the given list.

        Parameters
        ----------
        tm : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        bg_list : list
            The list contains all background traffic.

        Returns
        -------
        bg_list : list
            Update traffic list.
        """

        blueprint_library = self.world.get_blueprint_library()

        ego_vehicle_random_list = car_blueprint_filter(blueprint_library,
                                                       self.carla_version)
        # if not random select, we always choose lincoln.mkz with green color
        default_model = 'vehicle.lincoln.mkz2017' \
            if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'
        ego_vehicle_bp = blueprint_library.find(default_model)

        for i, vehicle_config in enumerate(traffic_config['vehicle_list']):
            spawn_transform = carla.Transform(
                carla.Location(
                    x=vehicle_config['spawn_position'][0],
                    y=vehicle_config['spawn_position'][1],
                    z=vehicle_config['spawn_position'][2]),
                carla.Rotation(
                    pitch=vehicle_config['spawn_position'][5],
                    yaw=vehicle_config['spawn_position'][4],
                    roll=vehicle_config['spawn_position'][3]))

            if not traffic_config['random']:
                ego_vehicle_bp.set_attribute('color', '0, 255, 0')

            else:
                ego_vehicle_bp = random.choice(ego_vehicle_random_list)

                color = random.choice(
                    ego_vehicle_bp.get_attribute('color').recommended_values)
                ego_vehicle_bp.set_attribute('color', color)

            vehicle = self.world.spawn_actor(ego_vehicle_bp, spawn_transform)
            vehicle.set_autopilot(True, 8000)

            if 'vehicle_speed_perc' in vehicle_config:
                tm.vehicle_percentage_speed_difference(
                    vehicle, vehicle_config['vehicle_speed_perc'])
            tm.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

            bg_list.append(vehicle)

        return bg_list

    def spawn_vehicle_by_range(self, tm, traffic_config, bg_list):
        """
        Spawn the traffic vehicles by the given range.

        Parameters
        ----------
        tm : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        bg_list : list
            The list contains all background traffic.

        Returns
        -------
        bg_list : list
            Update traffic list.
        """
        blueprint_library = self.world.get_blueprint_library()

        ego_vehicle_random_list = car_blueprint_filter(blueprint_library,
                                                       self.carla_version)
        # if not random select, we always choose lincoln.mkz with green color
        default_model = 'vehicle.lincoln.mkz2017' \
            if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'
        ego_vehicle_bp = blueprint_library.find(default_model)

        spawn_ranges = traffic_config['range']
        spawn_set = set()
        spawn_num = 0

        for spawn_range in spawn_ranges:
            spawn_num += spawn_range[6]
            x_min, x_max, y_min, y_max = \
                math.floor(spawn_range[0]), math.ceil(spawn_range[1]), \
                math.floor(spawn_range[2]), math.ceil(spawn_range[3])

            for x in range(x_min, x_max, int(spawn_range[4])):
                for y in range(y_min, y_max, int(spawn_range[5])):
                    location = carla.Location(x=x, y=y, z=0.3)
                    way_point = self.carla_map.get_waypoint(location).transform

                    spawn_set.add((way_point.location.x,
                                   way_point.location.y,
                                   way_point.location.z,
                                   way_point.rotation.roll,
                                   way_point.rotation.yaw,
                                   way_point.rotation.pitch))
        count = 0
        spawn_list = list(spawn_set)
        shuffle(spawn_list)

        while count < spawn_num:
            if len(spawn_list) == 0:
                break

            coordinates = spawn_list[0]
            spawn_list.pop(0)

            spawn_transform = carla.Transform(carla.Location(x=coordinates[0],
                                                             y=coordinates[1],
                                                             z=coordinates[
                                                                   2] + 0.3),
                                              carla.Rotation(
                                                  roll=coordinates[3],
                                                  yaw=coordinates[4],
                                                  pitch=coordinates[5]))
            if not traffic_config['random']:
                ego_vehicle_bp.set_attribute('color', '0, 255, 0')

            else:
                ego_vehicle_bp = random.choice(ego_vehicle_random_list)

                color = random.choice(
                    ego_vehicle_bp.get_attribute('color').recommended_values)
                ego_vehicle_bp.set_attribute('color', color)

            vehicle = \
                self.world.try_spawn_actor(ego_vehicle_bp, spawn_transform)

            if not vehicle:
                continue

            vehicle.set_autopilot(True, 8000)
            tm.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

            if 'ignore_lights_percentage' in traffic_config:
                tm.ignore_lights_percentage(vehicle,
                                            traffic_config[
                                                'ignore_lights_percentage'])

            # each vehicle have slight different speed
            tm.vehicle_percentage_speed_difference(
                vehicle,
                traffic_config['global_speed_perc'] + random.randint(-30, 30))

            bg_list.append(vehicle)
            count += 1

        return bg_list

    def create_traffic_carla(self):
        """
        Create traffic flow.

        Returns
        -------
        tm : carla.traffic_manager
            Carla traffic manager.

        bg_list : list
            The list that contains all the background traffic vehicles.
        """
        logger.info('Spawning CARLA traffic flow.')
        traffic_config = self.scenario_params['carla_traffic_manager']
        tm = self.client.get_trafficmanager()

        tm.set_global_distance_to_leading_vehicle(
            traffic_config['global_distance'])
        tm.set_synchronous_mode(traffic_config['sync_mode'])
        tm.set_osm_mode(traffic_config['set_osm_mode'])
        tm.global_percentage_speed_difference(
            traffic_config['global_speed_perc'])

        bg_list = []

        if isinstance(traffic_config['vehicle_list'], list):
            bg_list = self.spawn_vehicles_by_list(tm,
                                                  traffic_config,
                                                  bg_list)

        else:
            bg_list = self.spawn_vehicle_by_range(tm, traffic_config, bg_list)

        logger.info('CARLA traffic flow generated.')
        return tm, bg_list
    
    def close(self, spectator=None):
        """
        Simulation close.
        """
        # restore to origin setting
        if self.run_distributed:
            if spectator != None:
                logger.info("destroying specator CAV")
                try:
                    spectator.destroy()
                except:
                    logger.error("failed to destroy single CAV")
                    
            subprocess.Popen(['pkill','-9','CarlaUE4'])
            sys.exit(0)

        self.world.apply_settings(self.origin_settings)
        logger.debug(f"world state restored...")

    # END Core OpenCDA

    # -------------------------------------------------------

    # BEGIN eCloud
    def create_distributed_vehicle_manager(self, application,
                               map_helper=None,
                               data_dump=False):
        """
        Create a list of single CAVs.

        Parameters
        ----------
        application : list
            The application purpose, a list, eg. ['single'], ['platoon'].

        map_helper : function
            A function to help spawn vehicle on a specific position in
            a specific map.

        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        logger.info('Creating single CAVs.')
        single_cav_list = []

        config_yaml = load_yaml(self.config_file)
        for vehicle_index, _ in enumerate(
                self.scenario_params['scenario']['single_cav_list']):
            logger.debug(f"Creating VehiceManagerProxy for vehicle {vehicle_index}")

            # create vehicle manager for each cav
            vehicle_manager_proxy = VehicleManagerProxy(
                vehicle_index, config_yaml, application,
                self.carla_map, self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump, carla_version=self.carla_version)
            logger.debug("finished creating VehiceManagerProxy")

            # self.tick_world()

            # send gRPC with START info
            self.application = application

            # update the vehicle manager
            # keep a tuple of actor_id and vid in a list based on vehicle_index

            actor_id = self.vehicles[f"vehicle_{vehicle_index}"][0]
            vid = self.vehicles[f"vehicle_{vehicle_index}"][1]

            logger.debug(f"starting vehicle {vehicle_index} | actor_id: {actor_id} | vid: {vid}")

            vehicle_manager_proxy.start_vehicle(actor_id, vid)

            vehicle_manager_proxy.v2x_manager.set_platoon(None)
            logger.debug("set platoon on vehicle manager")

            single_cav_list.append(vehicle_manager_proxy)
            self.vehicle_managers[vehicle_index] = vehicle_manager_proxy

        self.tick_world()
        logger.info("Finished creating vehicle managers and returning cav list")
        return single_cav_list

    def create_edge_manager(self, application,
                            map_helper=None,
                            data_dump=False,
                            world_dt=0.03,
                            edge_dt=0.20,
                            search_dt=2.00):
        """
        Create a list of edges.

        Parameters
        ----------
        map_helper : function
            A function to help spawn vehicle on a specific position in a
            specific map.
        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """

        # TODO: needs to support multiple edges.
        # Probably a more significant refactor,
        # since I think each edge wants its own gRPC server

        logger.info('Creating edge CAVs.')
        edge_list = []

        config_yaml = load_yaml(self.config_file)
        # create edges
        for e, edge in enumerate(
                self.scenario_params['scenario']['edge_list']):
            edge_manager = EdgeManager(edge, self.cav_world, world_dt=world_dt, edge_dt=edge_dt, search_dt=search_dt)
            for vehicle_index, cav in enumerate(edge['members']):

                logger.debug(f"Creating VehiceManagerProxy for vehicle {vehicle_index}")

                # create vehicle manager for each cav
                vehicle_manager = VehicleManagerProxy(
                    vehicle_index, config_yaml, application,
                    self.carla_map, self.cav_world,
                    current_time=self.scenario_params['current_time'],
                    data_dumping=data_dump, carla_version=self.carla_version)
                logger.debug("finished creating VehiceManagerProxy")

                # self.tick_world()

                # send gRPC with START info
                self.application = application

                # update the vehicle manager
                # keep a tuple of actor_id and vid in a list based on vehicle_index
                actor_id = self.vehicles[f"vehicle_{vehicle_index}"][0]
                vid = self.vehicles[f"vehicle_{vehicle_index}"][1]

                logger.debug(f"starting vehicle {vehicle_index} | actor_id: {actor_id} | vid: {vid}")

                vehicle_manager.start_vehicle(actor_id, vid)
                vehicle_manager.v2x_manager.set_platoon(None)

                # add the vehicle manager to platoon
                edge_manager.add_member(vehicle_manager)
                self.vehicle_managers[vehicle_index] = vehicle_manager

            self.tick_world()
            destination = carla.Location(x=edge['destination'][0],
                                         y=edge['destination'][1],
                                         z=edge['destination'][2])

            edge_manager.set_destination(destination)
            edge_manager.start_edge()
            edge_list.append(edge_manager)

        return edge_list

    def tick_world(self):
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...
        """
        pre_world_tick_time = time.time()
        self.world.tick()
        post_world_tick_time = time.time()
        logger.debug("World tick completion time: %s" %(post_world_tick_time - pre_world_tick_time))
        ScenarioManager.debug_helper.update_world_tick((post_world_tick_time - pre_world_tick_time)*1000)

    def tick(self):
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...
        """
        self.tick_world()

    # just use tick logic here; need something smarter if we want per-vehicle data
    # could also just switch to a "broadcast message "
    def broadcast_message(self, message_type = ecloud.Command.TICK):
        """
        Request all clients send debug data - broadcasts a message to all vehicles

        just using the tick_id; as noted, we should change to message ID to make this more generic

        returns bool
        """
        #TODO change tick_id to msg_id
        pre_client_tick_time = time.time()
        self.tick_id = self.tick_id + 1

        sim_state_update = ecloud.SimulationState()
        sim_state_update.state = ecloud.State.ACTIVE
        sim_state_update.tick_id = self.tick_id
        sim_state_update.command = message_type
        if message_type == ecloud.Command.TICK:
            for waypoint_buffer_proto in self.waypoint_buffer_overrides:
                #logger.debug(waypoint_buffer_proto.SerializeToString())
                sim_state_update.all_waypoint_buffers.extend([waypoint_buffer_proto])

        logger.debug(f"Getting timestamp")
        sim_state_update.tstamp.GetCurrentTime()
        logger.debug(f"Added Timestamp") 

        asyncio.get_event_loop().run_until_complete(self.server_do_tick(self.ecloud_server, sim_state_update))

        if message_type == ecloud.Command.TICK:
            self.waypoint_buffer_overrides.clear()

        post_client_tick_time = time.time()
        logger.info("Client tick completion time: %s" %(post_client_tick_time - pre_client_tick_time))
        if self.tick_id > 1: # discard the first tick as startup is a major outlier
            ScenarioManager.debug_helper.update_client_tick((post_client_tick_time - pre_client_tick_time)*1000)

        return True

    def broadcast_tick(self):
        """
        Tick the server - broadcasts a message to all vehicles

        returns bool
        """
        return self.broadcast_message(ecloud.Command.TICK)


    def add_waypoint_buffer_to_tick(self, waypoint_buffer): #, vehicle_index=None, vid=None, actor_id=None):
        """
        adds a waypoint buffer for a specific vehicle to the current tick message

        currently assumes the scenario has constructed a WaypointBuffer protobuf with explicit vehicle_index UID

        returns bool
        """
        assert( len(self.waypoint_buffer_overrides) == 0 )

        # TODO: clone?
        self.waypoint_buffer_overrides = waypoint_buffer

        return True

    def end(self):
        """
        broadcast end to all vehicles
        """
        sim_state_update = ecloud.SimulationState()
        sim_state_update.state = ecloud.State.ENDED
        sim_state_update.command = ecloud.Command.END
        
        asyncio.get_event_loop().run_until_complete(self.server_end_scenario(self.ecloud_server, sim_state_update))

        logger.debug(f"pushed END")

        for i in range(0, 5):
            time.sleep(1)
            logger.debug("scenario ending in %d", 5 - i)

        os.kill(self.ecloud_server_process.pid, signal.SIGTERM)

    def do_pickling(self, column_key, flat_list, file_path):
        data_df = pd.DataFrame(flat_list, columns = [f'{column_key}_ms'])
        data_df['num_cars'] = self.vehicle_count
        data_df['run_timestamp'] = pd.Timestamp.today().strftime('%Y-%m-%d %X')
        data_df = data_df[['num_cars', f'{column_key}_ms', 'run_timestamp']]

        data_df_path = f'./{file_path}/df_{column_key}'
        try:
            picklefile = open(data_df_path, 'rb+')
            current_data_df = pickle.load(picklefile)  #unpickle the dataframe
        except:
            picklefile = open(data_df_path, 'wb+')
            current_data_df = pd.DataFrame(columns=['num_cars', f'{column_key}_ms', 'run_timestamp'])

        picklefile = open(data_df_path, 'wb+')
        data_df = pd.concat([current_data_df, data_df], axis=0, ignore_index=True)

        # pickle the dataFrame
        pickle.dump(data_df, picklefile)
        logger.info(data_df)
        #close file
        picklefile.close()

    def evaluate_agent_data(self, cumulative_stats_folder_path):
        PLANER_AGENT_STEPS = 12
        all_agent_data_lists = [[] for _ in range(PLANER_AGENT_STEPS)]
        for _, vehicle_manager_proxy in self.vehicle_managers.items():
            agent_data_list = vehicle_manager_proxy.agent.debug_helper.get_agent_step_list()
            for idx, sub_list in enumerate(agent_data_list):
                all_agent_data_lists[idx].append(sub_list)

        #logger.debug(all_agent_data_lists)

        for idx, all_agent_sub_list in enumerate(all_agent_data_lists):
            all_client_data_list_flat = np.array(all_agent_sub_list)
            if all_client_data_list_flat.any():
                all_client_data_list_flat = np.hstack(all_client_data_list_flat)
            else:
                all_client_data_list_flat = all_client_data_list_flat.flatten()
            data_key = f"agent_step_list_{idx}"
            self.do_pickling(data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_network_data(self, cumulative_stats_folder_path):
        all_network_data_lists(sum(ScenarioManager.debug_helper.network_time_dict.values()))

        logger.debug(all_agent_data_lists)

        for idx, all_agent_sub_list in enumerate(all_agent_data_lists):
            all_client_data_list_flat = np.array(all_agent_sub_list)
            if all_client_data_list_flat.any():
                all_client_data_list_flat = np.hstack(all_client_data_list_flat)
            else:
                all_client_data_list_flat = all_client_data_list_flat.flatten()
            data_key = f"agent_step_list_{idx}"
            self.do_pickling(data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_client_data(self, client_data_key, cumulative_stats_folder_path):
        all_client_data_list = []
        for _, vehicle_manager_proxy in self.vehicle_managers.items():
            client_data_list = vehicle_manager_proxy.debug_helper.get_debug_data()[client_data_key]
            all_client_data_list.append(client_data_list)

        #logger.debug(all_client_data_list)

        all_client_data_list_flat = np.array(all_client_data_list)
        if all_client_data_list_flat.any():
            all_client_data_list_flat = np.hstack(all_client_data_list_flat)
        else:
            all_client_data_list_flat = all_client_data_list_flat.flatten()
        self.do_pickling(client_data_key, all_client_data_list_flat, cumulative_stats_folder_path)
       
    def evaluate(self, excludes_list = None):
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

            perform_txt = ''

            cumulative_stats_folder_path = './evaluation_outputs/cumulative_stats_no_perception'
            if self.perception:
                cumulative_stats_folder_path = './evaluation_outputs/cumulative_stats_with_perception'

            if not os.path.exists(cumulative_stats_folder_path):
                os.makedirs(cumulative_stats_folder_path)

            self.evaluate_agent_data(cumulative_stats_folder_path)

            client_helper = ClientDebugHelper(0)
            debug_data_lists = client_helper.get_debug_data().keys()
            for list_name in debug_data_lists:
                if excludes_list is not None and list_name in excludes_list:
                    continue
                self.evaluate_client_data(list_name, cumulative_stats_folder_path)

            # ___________Client Step time__________________________________
            client_tick_time_list = ScenarioManager.debug_helper.client_tick_time_list
            client_tick_time_list_flat = np.concatenate(client_tick_time_list)
            if client_tick_time_list_flat.any():
                client_tick_time_list_flat = np.hstack(client_tick_time_list_flat)
            else:
                client_tick_time_list_flat = client_tick_time_list_flat.flatten()
            client_step_time_key = 'client_step_time'
            self.do_pickling(client_step_time_key, client_tick_time_list_flat, cumulative_stats_folder_path)

            # ___________World Step time_________________________________
            world_tick_time_list = ScenarioManager.debug_helper.world_tick_time_list
            world_tick_time_list_flat = np.concatenate(world_tick_time_list)
            if world_tick_time_list_flat.any():
                world_tick_time_list_flat = np.hstack(world_tick_time_list_flat)
            else:
                world_tick_time_list_flat = world_tick_time_list_flat.flatten()
            world_step_time_key = 'world_step_time'
            self.do_pickling(world_step_time_key, world_tick_time_list_flat, cumulative_stats_folder_path)

            # ___________Total simulation time ___________________
            sim_start_time = ScenarioManager.debug_helper.sim_start_timestamp
            sim_end_time = time.time()
            total_sim_time = (sim_end_time - sim_start_time) # total time in seconds
            perform_txt += f"Total Simulation Time: {total_sim_time}"

            sim_time_df_path = f'./{cumulative_stats_folder_path}/df_total_sim_time'
            try:
                picklefile = open(sim_time_df_path, 'rb+')
                sim_time_df = pickle.load(picklefile)  #unpickle the dataframe
            except:
                picklefile = open(sim_time_df_path, 'wb+')
                sim_time_df = pd.DataFrame(columns=['num_cars', 'time_s', 'run_timestamp'])

            picklefile = open(sim_time_df_path, 'wb+')
            sim_time_df = pd.concat([sim_time_df, pd.DataFrame.from_records \
                ([{"num_cars": self.vehicle_count, \
                    "time_s": total_sim_time, \
                    "run_timestamp": pd.Timestamp.today().strftime('%Y-%m-%d %X') }])], \
                    ignore_index=True)

            # pickle the dataFrame
            pickle.dump(sim_time_df, picklefile)
            print(sim_time_df)
            #close file
            picklefile.close()

            # plotting
            figure = plt.figure()

            plt.subplot(411)
            open_plt.draw_world_tick_time_profile_single_plot(world_tick_time_list)

            # plt.subplot(412)
            # open_plt.draw_algorithm_time_profile_single_plot(algorithm_time_list)

            return figure, perform_txt
    
    # END eCloud
