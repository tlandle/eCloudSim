# -*- coding: utf-8 -*-
"""
Script to run a simulated vehicle
"""

# Author: Aaron Drysdale <adrysdale3@gatech.edu>
import argparse
from atexit import register
from curses import A_DIM
import os
import sys
import json

import carla

from opencda.version import __version__
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.core.application.edge.transform_utils import *
from opencda.core.plan.local_planner_behavior import RoadOption
from opencda.core.plan.global_route_planner import GlobalRoutePlanner
from opencda.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO

# gRPC
from concurrent.futures import ThreadPoolExecutor
import coloredlogs, logging
import threading
import time
from typing import Iterator
from queue import Queue
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from google.protobuf.json_format import MessageToJson
import grpc

# sys.path.append('../../protos/')

import sim_api_pb2 as sim_state
import sim_api_pb2_grpc as rpc
#end gRPC

state = sim_state.State.UNDEFINED #do we need a global state?
vehicle_index = None
tick_id = 0
opencda_responded = threading.Event() # to signal from gRPC thread to main. redundant to below? Just set-clear-set?
opencda_started = threading.Event() # to signal from gRPC thread to main
carla_vehicle_created = threading.Event() # to signal from main to gRPC thread
sim_finished = threading.Event()
lock = threading.Lock()

pushed_message = threading.Event()
popped_message = threading.Event()

pushed_response = threading.Event()
popped_response = threading.Event()

# sim params
test_scenario = None #= message["params"]["scenario"]
application = None #= message["params"]["application"]
version = None #= message["params"]["version"]

# carla params
actor_id = None
vid = None

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]

class Client:

    def __init__(self, queue, channel: grpc.Channel) -> None:
        #self._executor = executor
        self._channel = channel
        self._stub = rpc.OpenCDAStub(self._channel)
        self._actor_id = None
        self._vid = None
        self._queue = queue

    
    def _sim_state_watcher(self) -> None:
        #try:
        global tick_id
        global vehicle_index
        for response in self._stub.SimulationStateStream(sim_state.Ping( vehicle_index = vehicle_index )):  # this line will wait for new messages from the server!
            #if response.tick_id != tick_id:
            logger.debug(f"M{response.message_id} - S{response.state} C{response.command} T{response.tick_id}")
            self._on_sim_state_update(response)
        #         else:
        #             raise RuntimeError(
        #                 "Received StreamCallResponse without state and tick_id"
        #             )
        # except Exception as e:
        #     global opencda_responded
        #     opencda_responded.set()
        #     raise

    def _on_sim_state_update(self, sim_state_update: sim_state.SimulationState) -> None:
        global sim_finished
        global opencda_responded
        global tick_id
        global vehicle_index
        global state
        global lock

        global pushed_message
        global popped_message

        # switch based on the command argument
        if sim_state_update.command == sim_state.Command.SET_DESTINATION and sim_state_update.vehicle_index == vehicle_index:
            logger.debug("received a set_destination command")

            self._queue.put(sim_state_update)
            pushed_message.set()
            popped_message.wait(timeout=None)
            popped_message.clear()

            logger.debug("processed a set_destination command")

            return

        elif sim_state_update.command == sim_state.Command.UPDATE_INFO and sim_state_update.vehicle_index == vehicle_index:
            logger.debug("received an update_info command")
                        
            self._queue.put(sim_state_update)
            pushed_message.set()
            popped_message.wait(timeout=None)
            popped_message.clear()

            logger.debug("processed an update_info command")

            return                      

        # did we get a new tick_id?
        if tick_id != sim_state_update.tick_id:
            logger.info(f"received new tick... {sim_state_update.tick_id}")
            tick_id = sim_state_update.tick_id
            #signal update

            self._queue.put(sim_state_update)
            pushed_message.set()
            popped_message.wait(timeout=None)
            popped_message.clear()

            pushed_response.wait()
            message = self._queue.get()
            pushed_response.clear()
            logger.info("responding to tick...")
            #logger.debug(message.SerializeToString())    
            self._stub.SendUpdate(message) 
            popped_response.set()

            return

        # check vehicle index to see if the message is for us
        if sim_state_update.state == sim_state.State.START and state != sim_state.State.START:
            logger.info("simulation state is now STARTED")
            global test_scenario
            global application
            global version

            test_scenario = sim_state_update.test_scenario
            test_scenario = test_scenario[ test_scenario.find("opencda/"): ] # strip off any pathing from host
            application = sim_state_update.application
            version = sim_state_update.version

            logger.debug(f"test_scenario: {test_scenario}")
            logger.debug(f"application: {application}")
            logger.debug(f"version: {version}")

            # signal main thread that we've got the start response
            global opencda_started
            opencda_started.set()

            logger.info("waiting for Carla data...")

            carla_vehicle_created.wait(timeout=None)

            self._actor_id = self._queue.get()
            self._vid = self._queue.get()

            logger.info("sending Carla data...")
            self.send_carla_data_to_opencda()

        if state != sim_state.State.ACTIVE and sim_state_update.state == sim_state.State.ACTIVE:
            logger.info("simulation state is now ACTIVE")
            with lock:
                state = sim_state_update.state
                logger.debug("eCloud debug: updated state to ACTIVE")
            #opencda_responded.set()

        if sim_state_update.state == sim_state.State.ENDED:
            logger.info("simulation state is now ENDED")
            sim_finished.set()

        logger.debug("exiting _on_sim_state_update")    

    def set_aid_and_vid(self, aid, vid) -> None:
        self._actor_id = aid
        self._vid = vid

    # def send_ping_to_opencda(self) -> None:
    #     global vehicle_index

    #     request = sim_state.VehicleUpdate()
    #     request.tick_id = tick_id
    #     request.vehicle_index = vehicle_index
    #     self._stub.SendUpdate(request)     

    # def send_update_info_ok(self) -> None:
    #     logger.debug("sending update info...")

    # def request_destination(self) -> None:
    #     logger.debug("sending update info...")    

    def send_registration_to_opencda(self) -> None:
        global vehicle_index

        request = sim_state.VehicleUpdate()
        request.vehicle_state = sim_state.VehicleState.REGISTERING
        response = self._stub.RegisterVehicle(request)

        vehicle_index = response.vehicle_index
        logger.debug(f"Vehicle ID {vehicle_index} received...")
        assert response.state == sim_state.State.NEW
        state = response.state

    def send_carla_data_to_opencda(self) -> None:
        global vehicle_index

        message = {"vehicle_index": vehicle_index, "actor_id": self._actor_id, "vid": self._vid}
        logger.debug(f"Vehicle: Sending Carla rpc {message}")

        # send actor ID and vid to API
        update = sim_state.VehicleUpdate()
        update.vehicle_state = sim_state.VehicleState.CARLA_UPDATE
        update.vehicle_index = vehicle_index
        update.vid = self._vid
        update.actor_id = self._actor_id
        
        response = None
        response = self._stub.RegisterVehicle(update)
        logger.debug("response...")

        # signal main thread?


def register_with_opencda(queue, channel: grpc.Channel) -> None:
    opencda_client = Client(queue, channel)
    opencda_client.send_registration_to_opencda()
    logger.info("Registration with OpenCDA finished!")
    opencda_client._sim_state_watcher() # listen on stream

def run(q):
    #executor = ThreadPoolExecutor()
    with grpc.insecure_channel(f"{CARLA_IP}:50051") as channel:
        #future = executor.submit(register_with_opencda, executor, channel)
        #future.result()
        register_with_opencda(q, channel)

#end gRPC

def arg_parse():
    parser = argparse.ArgumentParser(description="OpenCDA Vehicle Simulation.")
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-i', "--ipaddress", type=str, default=CARLA_IP,
                        help="Specifies the ip address of the server to connect to. [Default: localhost]")
    parser.add_argument('-p', "--port", type=int, default=5555,
                        help="Specifies the port to connect to. [Default: 5555]")
    parser.add_argument('-v', "--verbose", action="store_true",
                            help="Make more noise")
    parser.add_argument('-q', "--quiet", action="store_true",
                            help="Make no noise")

    opt = parser.parse_args()
    return opt

def main():
    # default params which can be over-written from the simulation controller
    application = ["single"]
    version = "0.9.12"

    opt = arg_parse()
    if opt.verbose:
        logger.setLevel(logging.DEBUG)
    elif opt.quiet:
        logger.setLevel(logging.WARNING)
    logger.debug(f"OpenCDA Version: {version}")

    # Use zmq for interprocess communication between OpenCDA and each vehicle
#    context = zmq.Context()
#    socket = context.socket(zmq.REP)
#    socket.bind(f"tcp://*:{opt.port}")

    # gRPC start

    logging.basicConfig()
    q = Queue()
    client_thread = threading.Thread(target=run, args=(q,), daemon=True)
    client_thread.start()

    #run()

    logger.info("waiting for opencda sim_api to start")
    opencda_started.wait(timeout=None) # send the start command below

    logger.info("opencda sim_api start")

    # gRPC end

    #_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #_socket.connect((opt.ipaddress, opt.port))


    # Wait for the START message from OpenCDA before going any further
    # TODO: change this to gRPC response 
    # print(f"Vehicle created. Waiting for start...", flush=True)
    # message = json.loads(_socket.recv(1024).decode('utf-8'))
    # print(f"Vehicle: received cmd {message}")
    # if message["cmd"] == "start":
    #     test_scenario = message["params"]["scenario"]
    #     vehicle_index = message["params"]["vehicle"]
    #     application = message["params"]["application"]
    #     version = message["params"]["version"]

    if not os.path.isfile(test_scenario):
        sys.exit("%s not found!" % test_scenario)

    logger.debug(f"main - test_scenario: {test_scenario}")
    logger.debug(f"main - application: {application[0]}")
    logger.debug(f"main - version: {version}")

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    logger.debug(f"eCloud debug: creating VehicleManager vehicle_index: {vehicle_index}")

    vehicle_manager = VehicleManager(vehicle_index, test_scenario, application, cav_world, version)

    scenario_yaml = load_yaml(test_scenario)
    target_speed = None
    if 'edge_list' in scenario_yaml['scenario']:
        # TODO: support multiple edges... 
        target_speed = scenario_yaml['scenario']['edge_list'][0]['target_speed']

    # send gRPC in response to start
    with lock:
        actor_id = vehicle_manager.vehicle.id
        vid = vehicle_manager.vid
        q.put(actor_id)
        q.put(vid)

    message = {"actor_id": actor_id, "vid": vid}
    logger.debug(f"Vehicle: Sending id {message}")

    carla_vehicle_created.set()

    #_socket.send(json.dumps(message).encode('utf-8'))


    # run scenario testing
    # TODO
    # - replace with event-based on streaming connection
    # - split the if/elif/elif into helper functions

    flag = True
    while flag:
        pushed_message.wait(timeout=None)    
        sim_state_update = q.get()
        #message = json.loads(_socket.recv(1024).decode('utf-8'))

        assert( type(sim_state_update) == type(sim_state.SimulationState()) )
        if type(sim_state_update) != type(sim_state.SimulationState()):
            logger.warning("eCloud debug: got bad message in queue, skipping...")
            continue

        if sim_state_update.command != sim_state.Command.TICK: # don't print tick message since there are too many
            logger.info(f"Vehicle: received cmd {sim_state_update.command}")

        # HANDLE UPDATE INFO
        if sim_state_update.command == sim_state.Command.UPDATE_INFO:

            vehicle_manager.update_info()
            #_socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))
            pushed_message.clear()    
            popped_message.set()
        
        # HANDLE SET DESTINATION
        elif sim_state_update.command == sim_state.Command.SET_DESTINATION:
            params_json = json.loads(sim_state_update.params_json)
            destination = params_json['params']
            logger.debug("JSON Params: " + sim_state_update.params_json)
            logger.debug(f"Vehicle: x=%s" % destination["start"]["x"])
            logger.debug(f"Vehicle: y=%s" % destination["start"]["y"])
            start_location = carla.Location(x=destination["start"]["x"], y=destination["start"]["y"], z=destination["start"]["z"])
            end_location = carla.Location(x=destination["end"]["x"], y=destination["end"]["y"], z=destination["end"]["z"])
            clean = bool(destination["clean"])
            end_reset = bool(destination["reset"])
            vehicle_manager.set_destination(start_location, end_location, clean, end_reset)
            #_socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))
            pushed_message.clear()    
            popped_message.set()
        
        # HANDLE TICK
        elif sim_state_update.command == sim_state.Command.TICK:
            # update info runs BEFORE waypoint injection
            vehicle_manager.update_info()

            # find waypoint buffer for our vehicle
            waypoint_proto = None
            for wpb in sim_state_update.all_waypoint_buffers:
                #logger.debug(wpb.SerializeToString())
                if wpb.vehicle_index == vehicle_index:
                    waypoint_proto = wpb
                    break
            
            if waypoint_proto != None:
                #override waypoints
                waypoint_buffer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
                # print(waypoint_buffer)
                # for waypoints in waypoint_buffer:
                #   print("Waypoints transform for Vehicle Before Clearing: " + str(i) + " : ", waypoints[0].transform)
                waypoint_buffer.clear() #EDIT MADE

                '''
                world = self.vehicle_manager_list[0].vehicle.get_world()
                self._dao = GlobalRoutePlannerDAO(world.get_map(), 2)
                location = self._dao.get_waypoint(carla.Location(x=car_array[0][i], y=car_array[1][i], z=0.0))
                '''
                world = vehicle_manager.vehicle.get_world()
                dao = GlobalRoutePlannerDAO(world.get_map(), 2)
                for swp in waypoint_proto.waypoint_buffer:
                    #logger.debug(swp.SerializeToString())
                    logger.debug(f"Override Waypoint x:{swp.transform.location.x}, y:{swp.transform.location.y}, z:{swp.transform.location.z}, rl:{swp.transform.rotation.roll}, pt:{swp.transform.rotation.pitch}, yw:{swp.transform.rotation.yaw}")
                    wp = deserialize_waypoint(swp, dao)
                    logger.debug(f"DAO Waypoint x:{wp.transform.location.x}, y:{wp.transform.location.y}, z:{wp.transform.location.z}, rl:{wp.transform.rotation.roll}, pt:{wp.transform.rotation.pitch}, yw:{wp.transform.rotation.yaw}")
                    waypoint_buffer.append((wp, RoadOption.STRAIGHT))

                cur_location = vehicle_manager.vehicle.get_location()
                logger.debug(f"location for vehicle_{vehicle_index} - is - x: {cur_location.x}, y: {cur_location.y}")

                waypoints_buffer_printer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
                for waypoints in waypoints_buffer_printer:
                    print("Waypoints transform for Vehicle: ", waypoints[0].transform)

            waypoints_buffer_printer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
            for waypoints in waypoints_buffer_printer:
                print("Waypoints transform for Vehicle: ", waypoints[0].transform)

            control = vehicle_manager.run_step(target_speed=target_speed)
            response = sim_state.VehicleUpdate()
            response.tick_id = tick_id
            response.vehicle_index = vehicle_index
            if control is None or vehicle_manager.is_close_to_scenario_destination():
                
                response.vehicle_state = sim_state.VehicleState.TICK_DONE

            else:

                vehicle_manager.apply_control(control)
                response.vehicle_state = sim_state.VehicleState.TICK_OK
                #_socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))

            cur_location = vehicle_manager.vehicle.get_location()
            logger.debug(f"send OK and location for vehicle_{vehicle_index} - is - x: {cur_location.x}, y: {cur_location.y}")

            pushed_message.clear()    
            popped_message.set()

            q.put(response)
            pushed_response.set()
            popped_response.wait()
            popped_response.clear()       

        # HANDLE END
        elif sim_state_update.command == sim_state.Command.END:
            pushed_message.clear()    
            popped_message.set()
            break
    
    # vehicle_manager.destroy() # let the scenario manager destroy...
    #_socket.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logger.info(' - Exited by user.')
