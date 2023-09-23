# -*- coding: utf-8 -*-
# pylint: disable=locally-disabled, line-too-long, invalid-name, broad-exception-caught
"""
Utilize scenario manager to manage CARLA simulation construction. This script
is used for carla simulation only, and if you want to manage the Co-simulation,
please use cosim_api.py.
"""
# Author: Jordan Rapp <jrapp7@gatech.edu> - eCloud
#         Runsheng Xu <rxx3386@ucla.edu> - OpenCDA
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import sys
import time
import json
import random
import os
import asyncio
import subprocess
import signal
import logging

import grpc
import carla
import numpy as np
#import k_means_constrained
from google.protobuf.timestamp_pb2 import Timestamp

from ecloud.core.common.vehicle_manager_proxy import VehicleManagerProxy
from ecloud.core.common.vehicle_manager import VehicleManager
from ecloud.core.application.platooning.platooning_manager import PlatooningManager
from ecloud.core.common.cav_world import CavWorld
from ecloud.scenario_testing.utils.customized_map_api import load_customized_world
from ecloud.core.application.edge.edge_manager import EdgeManager
from ecloud.sim_debug_helper import SimDebugHelper
import ecloud.globals as ecloud_globals
from ecloud.globals import EnvironmentConfig
from ecloud.core.common.ecloud_config import EcloudConfig
from ecloud.ecloud_server.ecloud_comms import EcloudComms, EcloudServerComms, ecloud_run_push_server

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

CARLA_IP = None
ECLOUD_IP = None

logger = logging.getLogger("ecloud")

def car_blueprint_filter(blueprint_library, _='0.9.12'):
    """
    Exclude the uncommon vehicles from the default CARLA blueprint library
    (i.e., isetta, carlacola, cybertruck, t2).

    Parameters
    ----------
    blueprint_library : carla.blueprint_library
        The blueprint library that contains all models.

    Returns
    -------
    blueprints : list
        The list of suitable blueprints for vehicles.
    """
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
    client_node_count = 1

    vehicle_managers = {}
    vehicle_count = 0

    carla_version = ecloud_globals.__carla_version__
    application = ['single']
    scenario = None
    ecloud_server = None
    is_edge = False
    vehicle_state = ecloud.VehicleState.REGISTERING

    debug_helper = SimDebugHelper(0)
    sm_start_tstamp = Timestamp()
    SPECTATOR_INDEX = ecloud_globals.__spectator_index__

    debug_helper = SimDebugHelper(0)

    def __init__(self, scenario_params,
                 apply_ml,
                 carla_version='0.9.12', # TODO global
                 xodr_path=None,
                 town=None,
                 cav_world=None,
                 distributed=False,
                 log_level=0,
                 ecloud_config=None,
                 run_carla=False):

        self.sm_start_tstamp.GetCurrentTime()

        global CARLA_IP
        global ECLOUD_IP

        CARLA_IP = EnvironmentConfig.get_carla_ip()
        ECLOUD_IP = EnvironmentConfig.get_ecloud_ip()

        # TODO: move these to EcloudConfig
        self.scenario_params = scenario_params
        self.carla_version = carla_version
        self.perception = scenario_params['perception_active'] if 'perception_active' in scenario_params else False

        # empty initializers
        self.carla_process = None
        self.ecloud_server_process = None
        self.push_q = None
        self.push_server = None
        self.comms_manager = None

        if ecloud_config is None:
            self.ecloud_config = EcloudConfig(scenario_params)
        else:
            self.ecloud_config = ecloud_config
        self.ecloud_config.set_log_level(log_level)
        self.scenario_params['scenario']['ecloud']['log_level'] = self.ecloud_config.get_log_level() # send to Clients

        simulation_config = scenario_params['world']

        self.run_distributed = distributed
        if distributed and ( ECLOUD_IP == 'localhost' or ECLOUD_IP == CARLA_IP ):
            server_log_level = 0 if self.ecloud_config.get_log_level() == logging.DEBUG else \
                                1 if self.ecloud_config.get_log_level() == logging.WARNING else 2 # 1: WARNING | 2: ERROR
            logger.info("setting server log level to %s", server_log_level)
            try:
                ecloud_pid = subprocess.check_output(['pgrep','ecloud_server'])
            except subprocess.CalledProcessError as e:
                if e.returncode > 1:
                    logger.error("exception trying to check for running server %s", e)
                    if ecloud_config.fatal_errors:
                        raise
                ecloud_pid = None
            if ecloud_pid is not None:
                logger.info('killing existing ecloud gRPC server process')
                subprocess.run(['pkill','-9','ecloud_server'], check=False)

            # PERF Profiling
            # self.ecloud_server_process = subprocess.Popen(['sudo','perf','record','-g','./opencda/ecloud_server/ecloud_server',f'--minloglevel={server_log_level}'], stderr=sys.stdout.buffer)
            # TODO move path to globals
            self.ecloud_server_process = subprocess.Popen([ecloud_globals.__ecloud_server_path__,
                                                           f'--minloglevel={server_log_level}'],
                                                           stderr=sys.stdout.buffer)

        if run_carla and ( CARLA_IP == 'localhost' or ECLOUD_IP == CARLA_IP ):
            try:
                carla_pid = subprocess.check_output(['pgrep','CarlaUE4'])

            except subprocess.CalledProcessError as e:
                if e.returncode > 1:
                    logger.error("failed to check for running Carla - %s", e)
                carla_pid = None

            if carla_pid is not None:
                logger.info('killing existing Carla instance')
                subprocess.run(['pkill','-9','Carla'], check=True)

            time.sleep(1)
            print('spawning Carla') # TODO: this MUST be a print call and NOT a log call for some reason
            self.carla_process = subprocess.Popen(['./CarlaUE4.sh',f'{run_carla}'],
                                                  cwd='/opt/carla-simulator/',
                                                  start_new_session=True,
                                                  stderr=sys.stdout.buffer)
            print("waiting for Carla to start up", end=" ")
            for _ in range(5):
                print(".", end=" ")
                time.sleep(1)
            print("")

        cav_world.update_scenario_manager(self)

        random.seed(time.time())
        if 'seed' in simulation_config:
            np.random.seed(simulation_config['seed'])
            random.seed(simulation_config['seed'])

        self.client = carla.Client(CARLA_IP, simulation_config['client_port'])
        self.client.set_timeout(EcloudComms.TIMEOUT_S)

        if xodr_path:
            self.world = load_customized_world(xodr_path, self.client)

        elif town:
            try:
                self.world = self.client.load_world(town)

            except Exception as e:
                logger.critical("%s - %s is not found in your CARLA repo! Please download all town maps to your CARLA repo!", e, town)

        else:
            self.world = self.client.get_world()

        if not self.world:
            logger.critical("world loading failed")
            sys.exit(1)

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

        if 'ecloud' in scenario_params['scenario'] and 'num_cars' in scenario_params['scenario']['ecloud']:
            assert 'edge_list' not in scenario_params['scenario'] # edge requires explicit
            self.vehicle_count = scenario_params['scenario']['ecloud']['num_cars']
            logger.debug("'ecloud' in YAML specified %s cars", self.vehicle_count)

        elif 'single_cav_list' in scenario_params['scenario']:
            self.vehicle_count = len(scenario_params['scenario']['single_cav_list'])

        elif 'edge_list' in scenario_params['scenario']:
            # TODO: support multiple edges...
            self.is_edge = True
            self.vehicle_count = len(scenario_params['scenario']['edge_list'][0]['members'])

        else:
            assert False, "no known vehicle indexing format found"

        if self.run_distributed:
            assert apply_ml is False, "ML should only be run on the distributed clients"
            self.apply_ml = False

            channel = grpc.aio.insecure_channel(
                        target=f"{ECLOUD_IP}:{ecloud_globals.__server_port__}",
                        options=EcloudComms.GRPC_OPTIONS)
            self.ecloud_server = ecloud_rpc.EcloudStub(channel)

            self.debug_helper.update_sim_start_timestamp(time.time())

            self.scenario = json.dumps(scenario_params)
            self.carla_version = self.carla_version

        # eCLOUD END

        else: # sequential
            self.debug_helper.update_sim_start_timestamp(time.time())

    async def run_comms(self):
        '''
        start up the local event listener gRPC server

        push scenario data up to the eCloud gRPC server
        '''
        self.push_q = asyncio.Queue()
        self.push_server = asyncio.create_task(ecloud_run_push_server(ecloud_globals.__push_api_port__, self.push_q))

        await asyncio.sleep(1) # this yields CPU to allow the PushServer to start

        server_request = ecloud.SimulationInfo()
        server_request.test_scenario = self.scenario
        server_request.application = self.application[0]
        server_request.version = self.carla_version
        server_request.vehicle_index = self.vehicle_count # bit of a hack to use vindex as count here
        server_request.is_edge = self.is_edge

        self.comms_manager = EcloudServerComms(vehicle_managers=self.vehicle_managers,
                                               debug_helper=self.debug_helper,
                                               is_edge=self.is_edge,
                                               push_q=self.push_q,
                                               sm_start_tstamp=self.sm_start_tstamp,
                                               vehicle_count=self.vehicle_count)

        await self.comms_manager.server_start_scenario(self.ecloud_server, server_request)

        self.world.tick()

        # fetch the distinct number of client nodes for this scenario run
        self.client_node_count = self.comms_manager.get_node_count()

        logger.debug("scenario started")

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
        logger.info('Creating single CAVs.')
        single_cav_list = []
        for vehicle_index in range(self.vehicle_count):

            # create vehicle manager for each cav
            vehicle_manager = VehicleManager(
                vehicle_index=vehicle_index, carla_world=self.world,
                config_yaml=self.scenario_params, application=application,
                carla_map=self.carla_map, cav_world=self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump, map_helper=map_helper,
                location_type=self.ecloud_config.get_location_type())

            self.world.tick()

            vehicle_manager.v2x_manager.set_platoon(None)

            vehicle_manager.update_info()
            vehicle_manager.set_destination(
                vehicle_manager.vehicle.get_location(),
                vehicle_manager.destination_location,
                clean=True)

            single_cav_list.append(vehicle_manager)
            self.vehicle_managers[vehicle_index] = vehicle_manager

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

        # create platoons
        count = 0
        for _, platoon in enumerate(
                self.scenario_params['scenario']['platoon_list']):
            platoon_manager = PlatooningManager(platoon, self.cav_world)
            for j, _ in enumerate(platoon['members']):
                # create vehicle manager for each cav
                vehicle_manager = VehicleManager(
                    vehicle_index=count, carla_world=self.world,
                    config_yaml=self.scenario_params, application=['platoon'],
                    carla_map=self.carla_map, cav_world=self.cav_world,
                    current_time=self.scenario_params['current_time'],
                    data_dumping=data_dump, map_helper=map_helper,
                    location_type=self.ecloud_config.get_location_type())

                self.world.tick()

                # add the vehicle manager to platoon
                if j == 0:
                    platoon_manager.set_lead(vehicle_manager)
                else:
                    platoon_manager.add_member(vehicle_manager, leader=False)
                self.vehicle_managers[count] = vehicle_manager
                count += 1

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
        default_model = 'vehicle.lincoln.mkz_2017'
        ego_vehicle_bp = blueprint_library.find(default_model)

        for _, vehicle_config in enumerate(traffic_config['vehicle_list']):
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
        default_model = 'vehicle.lincoln.mkz_2017'
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
        random.shuffle(spawn_list)

        while count < spawn_num:
            if len(spawn_list) == 0:
                break

            coordinates = spawn_list[0]
            spawn_list.pop(0)

            spawn_transform = carla.Transform(carla.Location(
                                                        x=coordinates[0],
                                                        y=coordinates[1],
                                                        z=coordinates[2] + 0.3),
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

    # END Core OpenCDA

    def close(self, spectator=None):
        """
        Simulation close.
        """
        # restore to origin setting
        if self.run_distributed:
            if spectator is not None:
                logger.info("destroying specator CAV")
                try:
                    spectator.destroy()
                except Exception as e:
                    logger.error("failed to destroy single CAV - %s", e)

            subprocess.Popen(['pkill','-9','CarlaUE4'])
            sys.exit(0)

        self.world.apply_settings(self.origin_settings)
        logger.debug("world state restored...")

    # BEGIN eCloud

    def create_distributed_vehicle_manager(self, application, data_dump=False) -> list:
        """
        Create a list of single CAVs proxies.

        Vehicle Manager Proxies are at lighter weight container class that contains
        an Actor Proxy. These can be used as the Spectator vehicle, but mostly they server
        as a container for debug helpers that we can dump data 1:1 after receiving it from
        distributed Clients that have the heavier weight Vehicle Manager class object

        Parameters
        ----------
        application : list
            The application purpose, a list, eg. ['single'], ['platoon'].

        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager proxies
        """
        logger.info('Creating single CAVs proxies')
        single_cav_list = []

        config_yaml = self.scenario_params
        for vehicle_index in range(self.vehicle_count):
            logger.debug("Creating VehiceManagerProxy for vehicle %s", vehicle_index)

            # create vehicle manager for each cav
            vehicle_manager_proxy = VehicleManagerProxy(
                vehicle_index, config_yaml, application,
                self.carla_map, self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump, location_type=self.ecloud_config.get_location_type())
            logger.debug("finished creating VehiceManagerProxy")

            # self.tick_world()

            # send gRPC with START info
            self.application = application

            vehicle_manager_proxy.start_vehicle()

            vehicle_manager_proxy.v2x_manager.set_platoon(None) # empty call
            logger.debug("set platoon on vehicle manager") # empty call

            single_cav_list.append(vehicle_manager_proxy)
            self.vehicle_managers[vehicle_index] = vehicle_manager_proxy

        self.tick_world()
        logger.info("Finished creating vehicle manager proxies and returning cav list")
        return single_cav_list

    def create_edge_manager(self,
                            application,
                            data_dump=False,
                            world_dt=ecloud_globals.__world_dt__,
                            edge_dt=ecloud_globals.__edge_dt__,
                            search_dt=ecloud_globals.__edge_search_t__):
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

        config_yaml = self.scenario_params
        # create edges
        for _, edge in enumerate(
                self.scenario_params['scenario']['edge_list']):
            edge_manager = EdgeManager(edge, self.cav_world, carla_client=self.client, world_dt=world_dt, edge_dt=edge_dt, search_dt=search_dt)
            for vehicle_index, _ in enumerate(edge['members']):

                logger.debug("Creating VehiceManagerProxy for vehicle %s", vehicle_index)

                # create vehicle manager for each cav
                vehicle_manager = VehicleManagerProxy(
                    vehicle_index, config_yaml, application,
                    self.carla_map, self.cav_world,
                    current_time=self.scenario_params['current_time'],
                    data_dumping=data_dump)
                logger.debug("finished creating VehiceManagerProxy")

                # self.tick_world()

                # send gRPC with START info
                self.application = application

                vehicle_manager.start_vehicle()
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

    def tick_world(self) -> None:
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...
        """
        pre_world_tick_time = time.time()
        self.world.tick()
        post_world_tick_time = time.time()
        logger.info("World tick completion time: %s", (post_world_tick_time - pre_world_tick_time))
        self.debug_helper.update_world_tick((post_world_tick_time - pre_world_tick_time)*1000)

    def tick(self) -> None:
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...
        """
        self.tick_world()

    # just use tick logic here; need something smarter if we want per-vehicle data
    # could also just switch to a "broadcast message "
    def broadcast_message(self, command = ecloud.Command.TICK) -> bool:
        """
        Request all clients send debug data - broadcasts a message to all vehicles

        just using the tick_id; as noted, we should change to message ID to make this more generic

        returns bool
        """
        pre_client_tick_time = time.time()
        self.tick_id = self.tick_id + 1

        if command == ecloud.Command.REQUEST_DEBUG_INFO:
            self.vehicle_state = ecloud.VehicleState.DEBUG_INFO_UPDATE

        tick = ecloud.Tick()
        tick.tick_id = self.tick_id
        tick.command = command

        logger.debug("getting timestamp")
        self.sm_start_tstamp.GetCurrentTime()
        logger.debug("added Timestamp")

        asyncio.get_event_loop().run_until_complete(self.comms_manager.server_do_tick(self.ecloud_server, tick))

        post_client_tick_time = time.time()
        logger.info("Client tick completion time: %s", (post_client_tick_time - pre_client_tick_time))
        if self.tick_id > 1: # discard the first tick as startup is a major outlier
            self.debug_helper.update_client_tick((post_client_tick_time - pre_client_tick_time)*1000)

        return True

    def broadcast_tick(self) -> bool:
        """
        Tick the server - broadcasts a message to all vehicles

        returns bool
        """
        self.vehicle_state = ecloud.VehicleState.TICK_OK
        return self.broadcast_message(ecloud.Command.TICK)


    def push_waypoint_buffer(self, waypoint_buffer) -> bool:
        """
        adds a waypoint buffer for a specific vehicle to the current tick message

        currently assumes the scenario has constructed a WaypointBuffer protobuf with explicit vehicle_index UID

        returns bool
        """
        edge_wp = ecloud.EdgeWaypoints()
        for wpb_proto in waypoint_buffer:
            #logger.debug(waypoint_buffer_proto.SerializeToString())
            edge_wp.all_waypoint_buffers.extend([wpb_proto])

        asyncio.get_event_loop().run_until_complete(self.comms_manager.server_push_waypoints(self.ecloud_server, edge_wp))

        return True

    def end(self) -> None:
        """
        broadcast end to all vehicles
        """
        start_time = time.time()
        self.vehicle_state = ecloud.VehicleState.TICK_DONE
        asyncio.get_event_loop().run_until_complete(self.comms_manager.server_end_scenario(self.ecloud_server))

        logger.info("pushed END")

        if self.run_distributed and ( ECLOUD_IP == 'localhost' or ECLOUD_IP == CARLA_IP ):
            os.kill(self.ecloud_server_process.pid, signal.SIGTERM)

        self.debug_helper.shutdown_time_ms = time.time() - start_time

    # END eCloud
