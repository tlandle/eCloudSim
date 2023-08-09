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

# gRPC
from concurrent.futures import ThreadPoolExecutor, thread
import coloredlogs, logging
import threading
import time
from typing import Iterable
from queue import Queue
import heapq

from google.protobuf.json_format import MessageToJson
import grpc

# sys.path.append('../../protos/')

import sim_api_pb2 as sim_state
import sim_api_pb2_grpc as rpc
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

# eCLOUD BEGIN

message_stack = []

# eCLOUD END

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
        Car;a HD Map.

    """

    # eCLOUD BEGIN
    
    connections_received = 0
    tick_id = 0 # current tick counter
    sim_state_responses = [[]] # list of responses per tick - e.g. sim_state_responses[tick_id = 1] = [veh_id = 1, veh_id = 2]
    sim_state_completions = [] # list of veh_ids that are complete

    waypoint_buffer_overrides = []

    tick_complete = threading.Event()
    set_sim_active = threading.Event()
    set_sim_started = threading.Event()
    lock = threading.Lock()
    
    vehicles = {} # vehicle_index -> tuple (actor_id, vid)
    vehicle_count = 0

    carla_version = None
    application = ['single']
    scenario = None

    pushed_message = threading.Event()
    popped_message = threading.Event()

    message_stack = []

    server_run = True
    server = None

    vehicle_managers = {}
    vehicle_index = 0

    class OpenCDA(rpc.OpenCDAServicer):

        def __init__(self, q, message_stack):
            self._id_counter = 0
            self._q = q
            self._message_stack = message_stack
            random.seed(time.time())    

        def SimulationStateStream(self, request, context):
            last_tick_id = 0
            has_printed = False

            logger.debug(f"request vehicle_index {request.vehicle_index}")    

            stack_index = 0 
            count = 0
            big_count = 0
            logger.debug("before while stack_index: " + str(stack_index))
            while ScenarioManager.server_run:
                #print("eCloud debug: listening for new messages in queue")
                #ScenarioManager.pushed_message.wait(timeout=None)
                if not self._q.empty():
                    sim_state_update = self._q.get()
                    logger.debug("Popped sim_state_update:\n" + str(sim_state_update))
                    if type(sim_state_update) == type(sim_state.SimulationState()):
                        logger.debug("SimulationStateStream message_stack - message id: " + str(sim_state_update.message_id) + " | command: " + str(sim_state_update.command) + " | tick: " + str(sim_state_update.tick_id))
                        with ScenarioManager.lock:
                            found = False
                            for message in ScenarioManager.message_stack:
                                state = sim_state.SimulationState()
                                state.ParseFromString(message)
                                if state.message_id == sim_state_update.message_id:
                                    found = True
                            if not found:
                                #print("new message:")
                                #print(sim_state_update.SerializeToString()) 
                                logger.debug("appended new message to stack")       
                                ScenarioManager.message_stack.append(sim_state_update.SerializeToString())

                    ScenarioManager.pushed_message.clear()
                    ScenarioManager.popped_message.set()  

                #print("message_stack has " + str(len(self._message_stack)) + " messages...")
                #print("stack_index" + str(stack_index))
                while stack_index < len(ScenarioManager.message_stack):
                    m = ScenarioManager.message_stack[stack_index]
                    message = sim_state.SimulationState()
                    message.ParseFromString(m)    
                    logger.debug("in while stack_index: " + str(stack_index))
                    logger.debug("SimulationStateStream yield - message id: " + str(message.message_id) + " | command: " + str(message.command) + " | tick: " + str(message.tick_id))

                    stack_index += 1

                    yield message
                
                count += 1
                time.sleep(0.1)
                if count % 1000 == 0:
                    count = 0
                    big_count += 1
                    logger.debug("looping " + str(big_count) + "...")

            logger.debug(f"SimulationStateStream complete")            

        def SendUpdate(self, request: sim_state.VehicleUpdate, context):

            # need to case handle based on response type... but we don't necessarily *need* this for acks (for now...)
            if request.vehicle_state == sim_state.VehicleState.OK:
                pass

            elif request.vehicle_state == sim_state.VehicleState.TICK_OK:

                with ScenarioManager.lock:
                    logger.debug(f"received TICK_OK from vehicle {request.vehicle_index}")
                    # make sure to add the tick_id to the root list when we do the tick
                    ScenarioManager.sim_state_responses[request.tick_id].append(request.vehicle_index)

            elif request.vehicle_state == sim_state.VehicleState.DEBUG_INFO_UPDATE:

                vehicle_manager_proxy = ScenarioManager.vehicle_managers[ request.vehicle_index ]
                # deserialize localization data
                loc_debug_helper = vehicle_manager_proxy.localizer.debug_helper
                loc_debug_helper.deserialize_debug_info( request.loc_debug_helper )
                # deserialize planer data
                planer_debug_helper = vehicle_manager_proxy.agent.debug_helper
                planer_debug_helper.deserialize_debug_info( request.planer_debug_helper )

            elif request.vehicle_state == sim_state.VehicleState.TICK_DONE:

                with ScenarioManager.lock:
                    logger.debug(f"received TICK_DONE from vehicle {request.vehicle_index}")
                    # make sure to add the tick_id to the root list when we do the tick
                    if request.vehicle_index not in ScenarioManager.sim_state_completions:
                        ScenarioManager.sim_state_completions.append(request.vehicle_index)

                    vehicle_manager_proxy = ScenarioManager.vehicle_managers[ request.vehicle_index ]
                    # deserialize localization data
                    loc_debug_helper = vehicle_manager_proxy.localizer.debug_helper
                    loc_debug_helper.deserialize_debug_info( request.loc_debug_helper )
                    # deserialize planer data
                    planer_debug_helper = vehicle_manager_proxy.agent.debug_helper
                    planer_debug_helper.deserialize_debug_info( request.planer_debug_helper )

                # this means sim is done - end??    

            elif request.vehicle_state == sim_state.VehicleState.ERROR:

                pass
                # TODO handle graceful termination

            if len(ScenarioManager.sim_state_responses[request.tick_id]) == ScenarioManager.vehicle_count or \
               ( ( len(ScenarioManager.sim_state_responses[request.tick_id]) + len(ScenarioManager.sim_state_completions) ) == ScenarioManager.vehicle_count ):
                logger.debug(f"TICK_COMPLETE for {request.tick_id}")
                ScenarioManager.tick_complete.set()    
            
            return sim_state.Empty()   

        
        def RegisterVehicle(self, request: sim_state.VehicleUpdate, context):
            #register the vehicle
            if request.vehicle_state == sim_state.VehicleState.REGISTERING:
                logger.debug("got a registration update")
                response = sim_state.SimulationState()
                response.state = sim_state.State.NEW
                response.tick_id = 0
                response.vehicle_index = ScenarioManager.connections_received
                response.message_id = str(hashlib.sha256(response.SerializeToString()).hexdigest())
                logger.debug("RegisterVehicle - REGISTERING - message id: " + str(response.message_id))
                with ScenarioManager.lock:
                    ScenarioManager.connections_received += 1
                return response  

            if request.vehicle_state == sim_state.VehicleState.CARLA_UPDATE:
                logger.debug("got a carla update")
                response = sim_state.SimulationState()
                response.state = sim_state.State.START # do we need a new state? like "registering"?
                response.tick_id = 0
                response.vehicle_index = request.vehicle_index
                logger.debug(f"Request vehicle_index: " + str(request.vehicle_index) + " | actor_id: " + str(request.actor_id) + " | vid: " + str(request.vid)) 
                ScenarioManager.vehicles[f"vehicle_{request.vehicle_index}"] = ( request.actor_id, request.vid )
                response.message_id = str(hashlib.sha256(response.SerializeToString()).hexdigest())
                logger.debug("RegisterVehicle - CARLA_UPDATE - message id: " + str(response.message_id))
                return response                    

    def serve(self, q: Queue(), message_stack, address: str) -> None:
        ScenarioManager.server = grpc.server(ThreadPoolExecutor(max_workers=200))
        rpc.add_OpenCDAServicer_to_server(self.OpenCDA(q, message_stack), ScenarioManager.server)
        ScenarioManager.server.add_insecure_port(address)
        ScenarioManager.server.start()
        logger.info(f"Server serving at {address}")
        ScenarioManager.server.wait_for_termination()

    # eCLOUD END

    def __init__(self, scenario_params,
                 apply_ml,
                 carla_version,
                 xodr_path=None,
                 town=None,
                 cav_world=None,
                 config_file=None):
        self.scenario_params = scenario_params
        self.carla_version = carla_version
        self.config_file = config_file
        self.perception = scenario_params['perception_active'] if 'perception_active' in scenario_params else False
        self.run_distributed = scenario_params['distributed'] if 'distributed' in scenario_params else False

        simulation_config = scenario_params['world']

        self.debug_helper = SimDebugHelper(0)
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

            # gRPC hello block begin
            self.message_queue = Queue()
            self.message_stack = []
            self.server_thread = threading.Thread(target=self.serve, args=(self.message_queue, self.message_stack, "[::]:50051",))
            self.server_thread.start()

            if 'single_cav_list' in scenario_params['scenario']:
                ScenarioManager.vehicle_count = len(scenario_params['scenario']['single_cav_list'])
            elif 'edge_list' in scenario_params['scenario']:
                # TODO: support multiple edges... 
                ScenarioManager.vehicle_count = len(scenario_params['scenario']['edge_list'][0]['members'])
            else:
                assert(False, "no known vehicle indexing format found")

            print("start vehicle containers")

            while ScenarioManager.connections_received < ScenarioManager.vehicle_count:
                #time.sleep(1)
                logger.info(f"received {ScenarioManager.connections_received} registrations. sim_api sleeping...")
                #should we wait for a threading event instead?

            print("vehicles registered, running simulation...")
            self.debug_helper.update_sim_start_timestamp(time.time())

            ScenarioManager.scenario = json.dumps(scenario_params) #self.config_file
            ScenarioManager.carla_version = self.carla_version
        
            # signal server to put ACTIVE on the wire
            #ScenarioManager.set_sim_started.set()

            sim_state_update = sim_state.SimulationState()
            sim_state_update.test_scenario = ScenarioManager.scenario
            sim_state_update.application = ScenarioManager.application[0]
            sim_state_update.version = ScenarioManager.carla_version
            sim_state_update.message_id = str(hashlib.sha256(sim_state_update.SerializeToString()).hexdigest())

            sim_state_update.state = sim_state.State.START
            sim_state_update.tick_id = ScenarioManager.tick_id
            self.message_queue.put(sim_state_update)

            logger.debug("eCloud debug: pushed START")

            ScenarioManager.pushed_message.set()

            ScenarioManager.popped_message.wait(timeout=None)
            ScenarioManager.popped_message.clear()

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

        logger.debug("main thread setting simulation active")
        #ScenarioManager.set_sim_active.set()

        for i, cav_config in enumerate(
                self.scenario_params['scenario']['single_cav_list']):
            logger.debug(f"Creating VehiceManagerProxy for vehicle {i}")

            # create vehicle manager for each cav
            vehicle_manager = VehicleManagerProxy(
                self.vehicle_index, self.config_file, application,
                self.carla_map, self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump, carla_version=self.carla_version)
            logger.debug("finished creating VehiceManagerProxy")

            # send gRPC with START info
            ScenarioManager.application = application

            # update the vehicle manager
            # keep a tuple of actor_id and vid in a list based on vehicle_index
            actor_id = None
            vid = None

            while len(ScenarioManager.vehicles) < ScenarioManager.vehicle_count:
                #time.sleep(1)
                logger.info("waiting for Carla data: number of vehicles: %d" %(len(ScenarioManager.vehicles)))

            actor_id = ScenarioManager.vehicles[f"vehicle_{i}"][0]
            vid = ScenarioManager.vehicles[f"vehicle_{i}"][1]

            logger.debug("starting vehicle | actor_id: " + str(actor_id) + " | vid: " + str(vid))

            vehicle_manager.start_vehicle(actor_id, vid)

            self.world.tick()
            logger.debug("ticked world")

            vehicle_manager.v2x_manager.set_platoon(None)
            logger.debug("set platoon on vehicle manager")

            destination = carla.Location(x=cav_config['destination'][0],
                                         y=cav_config['destination'][1],
                                         z=cav_config['destination'][2])
            logger.debug("get location of destination")

            #vehicle_manager.update_info()

            # gRPC update_info
            sim_state_update = sim_state.SimulationState()
            sim_state_update.state = sim_state.State.ACTIVE
            sim_state_update.tick_id = ScenarioManager.tick_id
            sim_state_update.command = sim_state.Command.UPDATE_INFO
            sim_state_update.vehicle_index = i
            sim_state_update.message_id = str(hashlib.sha256(sim_state_update.SerializeToString()).hexdigest())
            self.message_queue.put(sim_state_update)
            ScenarioManager.pushed_message.set()
            # end gRPC update_info
            
            logger.debug(f"update info complete for vehicle_index {i}")
            
            # vehicle_manager.set_destination(
            #     vehicle_manager.vehicle.get_location(),
            #     destination,
            #     clean=True)

            # gRPC set_destination
            ScenarioManager.popped_message.wait(timeout=None)
            ScenarioManager.popped_message.clear()
            sim_state_update = sim_state.SimulationState()
            sim_state_update.state = sim_state.State.ACTIVE
            sim_state_update.tick_id = ScenarioManager.tick_id
            sim_state_update.vehicle_index = i

            start_location = vehicle_manager.vehicle.get_location()
            message = { 
                    "params": {
                    "start": {"x": start_location.x, "y": start_location.y, "z": start_location.z},
                    "end": {"x": destination.x, "y": destination.y, "z": destination.z},
                    "clean": True, "reset": True
                    }
            }
            sim_state_update.params_json = json.dumps(message).encode('utf-8')

            sim_state_update.command = sim_state.Command.SET_DESTINATION
            sim_state_update.message_id = str(hashlib.sha256(sim_state_update.SerializeToString()).hexdigest())
            self.message_queue.put(sim_state_update)
            ScenarioManager.pushed_message.set()
            # end gRPC set_destination

            logger.debug(f"set destination complete for vehicle_index {i}")

            single_cav_list.append(vehicle_manager)
            ScenarioManager.vehicle_managers[self.vehicle_index] = vehicle_manager
            self.vehicle_index = self.vehicle_index + 1


            ScenarioManager.popped_message.wait(timeout=None)
            ScenarioManager.popped_message.clear()

        logger.debug("finished creating vehicle managers and returning cav list")
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
                vehicle_manager = VehicleManagerProxy(
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

        # create edges
        for e, edge in enumerate(
                self.scenario_params['scenario']['edge_list']):
            edge_manager = EdgeManager(edge, self.cav_world, world_dt=world_dt, edge_dt=edge_dt, search_dt=search_dt)
            for i, cav in enumerate(edge['members']):

                logger.debug(f"Creating VehiceManagerProxy for vehicle {i}")

                # create vehicle manager for each cav
                vehicle_manager = VehicleManagerProxy(
                    self.vehicle_index, self.config_file, application,
                    self.carla_map, self.cav_world,
                    current_time=self.scenario_params['current_time'],
                    data_dumping=data_dump, carla_version=self.carla_version)
                logger.debug("finished creating VehiceManagerProxy")

                # vehicle_manager = VehicleManager(
                #     i, self.config_file, application,
                #     self.carla_map, self.cav_world,
                #     current_time=self.scenario_params['current_time'],
                #     data_dumping=data_dump)
                # logger.debug("finished creating VehiceManager")

                # send gRPC with START info
                ScenarioManager.application = application

                # update the vehicle manager
                # keep a tuple of actor_id and vid in a list based on vehicle_index
                actor_id = None
                vid = None

                while len(ScenarioManager.vehicles) < ScenarioManager.vehicle_count:
                    #time.sleep(1)
                    logger.info("waiting for Carla data")

                actor_id = ScenarioManager.vehicles[f"vehicle_{i}"][0]
                vid = ScenarioManager.vehicles[f"vehicle_{i}"][1]

                logger.debug(f"starting vehicle {i} | actor_id: {actor_id} | vid: {vid}")

                vehicle_manager.start_vehicle(actor_id, vid)

                self.world.tick()
                logger.debug("ticked world")

                vehicle_manager.v2x_manager.set_platoon(None)
                logger.debug("set platoon on vehicle manager")

                destination = carla.Location(x=edge['destination'][0],
                                             y=edge['destination'][1],
                                             z=edge['destination'][2])
                logger.debug("get location of destination")

                #vehicle_manager.update_info()

                # gRPC update_info
                sim_state_update = sim_state.SimulationState()
                sim_state_update.state = sim_state.State.ACTIVE
                sim_state_update.tick_id = ScenarioManager.tick_id
                sim_state_update.command = sim_state.Command.UPDATE_INFO
                sim_state_update.vehicle_index = i
                sim_state_update.message_id = str(hashlib.sha256(sim_state_update.SerializeToString()).hexdigest())
                self.message_queue.put(sim_state_update)
                ScenarioManager.pushed_message.set()
                # end gRPC update_info
                
                logger.debug(f"update info complete for vehicle_index {i}")
                
                # vehicle_manager.set_destination(
                #     vehicle_manager.vehicle.get_location(),
                #     destination,
                #     clean=True)

                # gRPC set_destination
                ScenarioManager.popped_message.wait(timeout=None)
                ScenarioManager.popped_message.clear()
                sim_state_update = sim_state.SimulationState()
                sim_state_update.state = sim_state.State.ACTIVE
                sim_state_update.tick_id = ScenarioManager.tick_id
                sim_state_update.vehicle_index = self.vehicle_index

                start_location = vehicle_manager.vehicle.get_location()
                message = { 
                        "params": {
                        "start": {"x": start_location.x, "y": start_location.y, "z": start_location.z},
                        "end": {"x": destination.x, "y": destination.y, "z": destination.z},
                        "clean": True, "reset": True
                        }
                }
                sim_state_update.params_json = json.dumps(message).encode('utf-8')
                logger.debug(f"location for vehicle_{i} - is - x: {start_location.x}, y: {start_location.y}")

                sim_state_update.command = sim_state.Command.SET_DESTINATION
                sim_state_update.message_id = str(hashlib.sha256(sim_state_update.SerializeToString()).hexdigest())
                self.message_queue.put(sim_state_update)
                ScenarioManager.pushed_message.set()
                # end gRPC set_destination

                logger.debug(f"set destination complete for vehicle_index {i}")

                # add the vehicle manager to platoon
                edge_manager.add_member(vehicle_manager)
                ScenarioManager.vehicle_managers[self.vehicle_index] = vehicle_manager
                self.vehicle_index = self.vehicle_index + 1

                ScenarioManager.popped_message.wait(timeout=None)
                ScenarioManager.popped_message.clear()         

            self.world.tick()
            destination = carla.Location(x=edge['destination'][0],
                                         y=edge['destination'][1],
                                         z=edge['destination'][2])

            edge_manager.set_destination(destination)
            
            edge_manager.start_edge()
            
            edge_list.append(edge_manager)

        return edge_list

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

    def tick(self):
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...

        returns bool 
        """
        self.world.tick()

    # eCLOUD BEGIN
    def tick_world(self):
        """
        Tick the server; just a pass-through to broadcast_tick to preserve backwards compatibility for now...

        returns bool 
        """
        pre_world_tick_time = time.time()
        self.world.tick()  
        post_world_tick_time = time.time()
        logger.debug("World tick completion time: %s" %(post_world_tick_time - pre_world_tick_time))
        self.debug_helper.update_world_tick((post_world_tick_time - pre_world_tick_time)*1000)
            

    def broadcast_tick(self):
        """
        Tick the server - broadcasts a message to all vehicles

        returns bool
        """

        #TODO change tick_id to msg_id
        pre_client_tick_time = time.time()
        ScenarioManager.tick_id = ScenarioManager.tick_id + 1
        with ScenarioManager.lock:
            ScenarioManager.sim_state_responses.append(ScenarioManager.tick_id)
            ScenarioManager.sim_state_responses[ScenarioManager.tick_id] = []

        sim_state_update = sim_state.SimulationState()
        sim_state_update.state = sim_state.State.ACTIVE
        sim_state_update.tick_id = ScenarioManager.tick_id
        sim_state_update.command = sim_state.Command.TICK
        for waypoint_buffer_proto in ScenarioManager.waypoint_buffer_overrides:
            #logger.debug(waypoint_buffer_proto.SerializeToString())
            sim_state_update.all_waypoint_buffers.extend([waypoint_buffer_proto])
        sim_state_update.message_id = str(hashlib.sha256(sim_state_update.SerializeToString()).hexdigest())
        self.message_queue.put(sim_state_update)
        ScenarioManager.pushed_message.set()

        logger.debug(f"queued tick {ScenarioManager.tick_id}")

        ScenarioManager.popped_message.wait(timeout=None)
        ScenarioManager.popped_message.clear()

        logger.debug(f"pushed tick {ScenarioManager.tick_id}")

        ScenarioManager.tick_complete.wait(timeout=None)
        ScenarioManager.tick_complete.clear()

        ScenarioManager.waypoint_buffer_overrides.clear()
        post_client_tick_time = time.time()
        self.debug_helper.update_client_tick((post_client_tick_time - pre_client_tick_time)*1000)
        if len(ScenarioManager.sim_state_completions) == ScenarioManager.vehicle_count:
            return False # TODO - make a better flag
        else:  
            return True

    def add_waypoint_buffer_to_tick(self, waypoint_buffer): #, vehicle_index=None, vid=None, actor_id=None):
        """
        adds a waypoint buffer for a specific vehicle to the current tick message

        currently assumes the scenario has constructed a WaypointBuffer protobuf with explicit vehicle_index UID

        returns bool
        """             
        assert( len(ScenarioManager.waypoint_buffer_overrides) == 0 )

        # TODO: clone?
        ScenarioManager.waypoint_buffer_overrides = waypoint_buffer

        return True       

    def end(self):
        """
        broadcast end to all vehicles
        """
        sim_state_update = sim_state.SimulationState()
        sim_state_update.state = sim_state.State.ENDED
        sim_state_update.command = sim_state.Command.END
        sim_state_update.message_id = str(hashlib.sha256(sim_state_update.SerializeToString()).hexdigest())
        self.message_queue.put(sim_state_update)
        ScenarioManager.pushed_message.set()

        logger.debug(f"queued END")

        ScenarioManager.popped_message.wait(timeout=None)
        ScenarioManager.popped_message.clear()

        logger.debug(f"pushed END")

        for i in range(0, 5):
            time.sleep(1)
            logger.debug("scenario ending in %d", 5 - i) 
    
    # eCLOUD END    

    def destroyActors(self):
        """
        Destroy all actors in the world.
        """
        if self.run_distributed:
            logger.info("waiting for container shutdown...")
            for i in range(0, 5):
                #time.sleep(1)
                logger.debug("destroying actors in %d", 5 - i) 

        actor_list = self.world.get_actors()
        for actor in actor_list:
            logger.debug(f"destroying actor %s", actor)
            actor.destroy()

    def close(self):
        """
        Simulation close.
        """

        if self.run_distributed:
            ScenarioManager.server_run = False
            ScenarioManager.server.stop(grace=5)
            logger.debug(f"telling server_thread to STOP")
            self.server_thread.join()
            logger.debug(f"server_thread joined")
        
        # restore to origin setting
        self.world.apply_settings(self.origin_settings)
        logger.debug(f"world state restored...")

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

            perform_txt = ''

            cumulative_stats_folder_path = './evaluation_outputs/cumulative_stats_no_perception'
            if self.perception:
                cumulative_stats_folder_path = './evaluation_outputs/cumulative_stats_with_perception'

            if not os.path.exists(cumulative_stats_folder_path):
                os.makedirs(cumulative_stats_folder_path)

                
            client_tick_time_list = self.debug_helper.client_tick_time_list
            client_tick_time_list_flat = np.concatenate(client_tick_time_list).ravel()

            client_step_time_df = pd.DataFrame(client_tick_time_list_flat, columns = ['client_step_time_ms'])
            client_step_time_df['num_cars'] = ScenarioManager.vehicle_count
            client_step_time_df['run_timestamp'] =  pd.Timestamp.today().strftime('%Y-%m-%d %X')
            client_step_time_df = client_step_time_df[['num_cars','client_step_time_ms', 'run_timestamp']]

            client_step_time_df_path = f'./{cumulative_stats_folder_path}/df_client_step_time'
            client_step_time_df_cumstats_path = f'./{cumulative_stats_folder_path}/df_client_step_time_cumstats'
            try:
                picklefile = open(client_step_time_df_path, 'rb+')
                current_client_step_time_df = pickle.load(picklefile)  #unpickle the dataframe
            except:
                picklefile = open(client_step_time_df_path, 'wb+')
                current_client_step_time_df = pd.DataFrame(columns=['num_cars', 'client_step_time_ms', 'run_timestamp'])

            picklefile = open(client_step_time_df_path, 'wb+')
            client_step_time_df = pd.concat([current_client_step_time_df, client_step_time_df], axis=0, ignore_index=True)

            # pickle the dataFrame
            pickle.dump(client_step_time_df, picklefile)
            print(client_step_time_df)
            #close file
            picklefile.close()

            # create new df with cumultaive stats (e.g. mean, std, median, min, max)
            client_step_time_cumstats_df = pd.DataFrame()
            client_step_time_cumstats_df = client_step_time_df.groupby('num_cars')['client_step_time_ms'].agg(['std', 'mean', 'median', 'min', 'max']).reset_index()
            picklefile = open(client_step_time_df_cumstats_path, 'wb+')
            pickle.dump(client_step_time_cumstats_df, picklefile)
            picklefile.close()
            print(client_step_time_cumstats_df)

 
            # ___________________________________World Step time________________________________________________
            world_tick_time_list = self.debug_helper.world_tick_time_list
            world_tick_time_list_flat = np.concatenate(world_tick_time_list).ravel()
    

            step_time_df = pd.DataFrame(world_tick_time_list_flat, columns = ['world_step_time_ms'])
            step_time_df['num_cars'] = ScenarioManager.vehicle_count
            step_time_df['run_timestamp'] =  pd.Timestamp.today().strftime('%Y-%m-%d %X')
            step_time_df = step_time_df[['num_cars','world_step_time_ms', 'run_timestamp']]

            step_time_df_path = f'./{cumulative_stats_folder_path}/df_world_step_time'
            step_time_df_cumstats_path = f'./{cumulative_stats_folder_path}/df_world_step_time_cumstats'
            try:
                picklefile = open(step_time_df_path, 'rb+')
                current_step_time_df = pickle.load(picklefile)  #unpickle the dataframe
            except:
                picklefile = open(step_time_df_path, 'wb+')
                current_step_time_df = pd.DataFrame(columns=['num_cars', 'world_step_time_ms', 'run_timestamp'])

            picklefile = open(step_time_df_path, 'wb+')
            step_time_df = pd.concat([current_step_time_df, step_time_df], axis=0, ignore_index=True)

            # pickle the dataFrame
            pickle.dump(step_time_df, picklefile)
            print(step_time_df)
            #close file
            picklefile.close()

            # create new df with cumultaive stats (e.g. mean, std, median, min, max)
            step_time_cumstats_df = pd.DataFrame()
            step_time_cumstats_df = step_time_df.groupby('num_cars')['world_step_time_ms'].agg(['std', 'mean', 'median', 'min', 'max']).reset_index()
            picklefile = open(step_time_df_cumstats_path, 'wb+')
            pickle.dump(step_time_cumstats_df, picklefile)
            picklefile.close()
            print(step_time_cumstats_df)


            # ________________________________________Total simulation time __________________________________________________
            sim_start_time = self.debug_helper.sim_start_timestamp
            sim_end_time = time.time()
            total_sim_time = (sim_end_time - sim_start_time) # total time in seconds
            perform_txt += f"Total Simulation Time: {total_sim_time}"

            sim_time_df_path = f'./{cumulative_stats_folder_path}/df_total_sim_time'
            sim_time_df_cumstats_path = f'./{cumulative_stats_folder_path}/df_total_sim_time_cumstats'

            try:
                picklefile = open(sim_time_df_path, 'rb+')
                sim_time_df = pickle.load(picklefile)  #unpickle the dataframe
            except:
                picklefile = open(sim_time_df_path, 'wb+')
                sim_time_df = pd.DataFrame(columns=['num_cars', 'time_s', 'run_timestamp'])

            picklefile = open(sim_time_df_path, 'wb+')
            sim_time_df = pd.concat([sim_time_df, pd.DataFrame.from_records \
                ([{"num_cars": ScenarioManager.vehicle_count, \
                    "time_s": total_sim_time, \
                    "run_timestamp": pd.Timestamp.today().strftime('%Y-%m-%d %X') }])], \
                    ignore_index=True)
            
            # pickle the dataFrame
            pickle.dump(sim_time_df, picklefile)
            print(sim_time_df)
            #close file
            picklefile.close()

            # create new df with cumultaive stats (e.g. mean, std, median, min, max)
            sim_time_cumstats_df = pd.DataFrame()
            sim_time_cumstats_df = sim_time_df.groupby('num_cars')['time_s'].agg(['std', 'mean', 'median', 'min', 'max']).reset_index()
            picklefile = open(sim_time_df_cumstats_path, 'wb+')
            pickle.dump(sim_time_cumstats_df, picklefile)
            picklefile.close()
            print(sim_time_cumstats_df)
        
            # plotting
            figure = plt.figure()

            plt.subplot(411)
            open_plt.draw_world_tick_time_profile_single_plot(world_tick_time_list)

            # plt.subplot(412)
            # open_plt.draw_algorithm_time_profile_single_plot(algorithm_time_list)

            return figure, perform_txt
