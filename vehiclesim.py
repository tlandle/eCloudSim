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
import zmq
import socket
import json

import carla

from opencda.version import __version__
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager

# gRPC
from concurrent.futures import ThreadPoolExecutor
import logging
import threading
import time
from typing import Iterator
from queue import Queue

from google.protobuf.json_format import MessageToJson
import grpc

# sys.path.append('../../protos/')

import sim_api_pb2 as sim_state
import sim_api_pb2_grpc as rpc
#end gRPC

state = sim_state.State.UNDEFINED #do we need a global state?
vehicle_index = None
tick_id = None
opencda_responded = threading.Event() # to signal from gRPC thread to main. redundant to below? Just set-clear-set?
opencda_started = threading.Event() # to signal from gRPC thread to main
carla_vehicle_created = threading.Event() # to signal from main to gRPC thread
sim_finished = threading.Event()
lock = threading.Lock()

# sim params
test_scenario = None #= message["params"]["scenario"]
application = None #= message["params"]["application"]
version = None #= message["params"]["version"]

# carla params
actor_id = None
vid = None

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
        for response in self._stub.SimulationStateStream(sim_state.Empty()):  # this line will wait for new messages from the server!
            if response.tick_id != tick_id:
                self._on_sim_state_update(response)
                print("R[{}] {}".format(response.state, response.tick_id))  # debugging statement
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

        # did we get a new tick_id?
        if tick_id != sim_state_update.tick_id:
            print("received new tick...")
            tick_id = sim_state_update.tick_id
            #signal update

        # check vehicle index to see if the message is for us
        if sim_state_update.state == sim_state.State.START:
            global test_scenario
            global application
            global version

            test_scenario = sim_state_update.test_scenario
            application = sim_state_update.application
            version = sim_state_update.version

            print("test_scenario: " + test_scenario)
            print("application: " + application)
            print("version: " + version)

            # signal main thread that we've got the start response
            global opencda_started
            opencda_started.set()

            print("waiting for Carla data...")

            carla_vehicle_created.wait(timeout=None)

            self._actor_id = self._queue.get()
            self._vid = self._queue.get()

            print("sending Carla data...")
            self.send_carla_data_to_opencda()

        if state != sim_state.State.ACTIVE and sim_state_update.state == sim_state.State.ACTIVE:
            print("simulation state is now ACTIVE")
            with lock:
                state = sim_state_update.state
            opencda_responded.set()

        if sim_state_update.state == sim_state.State.ENDED:
            print("simulation state is now ENDED")
            sim_finished.set()

    def set_aid_and_vid(self, aid, vid) -> None:
        self._actor_id = aid
        self._vid = vid

    def send_ping_to_opencda(self) -> None:
        global vehicle_index

        request = sim_state.VehicleUpdate()
        request.tick_id = tick_id
        request.vehicle_index = vehicle_index
        self._stub.SendUpdate(request)     

    def send_registration_to_opencda(self) -> None:
        global vehicle_index

        request = sim_state.VehicleUpdate()
        request.vehicle_state = sim_state.VehicleState.REGISTERING
        response = self._stub.RegisterVehicle(request)

        vehicle_index = response.vehicle_index
        print("Vehicle ID " + str(vehicle_index) + " received...")
        assert response.state == sim_state.State.NEW
        state = response.state

    def send_carla_data_to_opencda(self) -> None:
        global vehicle_index

        message = {"vehicle_index": vehicle_index, "actor_id": self._actor_id, "vid": self._vid}
        print(f"Vehicle: Sending Carla rpc {message}")

        # send actor ID and vid to API
        update = sim_state.VehicleUpdate()
        update.vehicle_state = sim_state.VehicleState.CARLA_UPDATE
        update.vehicle_index = vehicle_index
        update.vid = self._vid
        update.actor_id = self._actor_id
        
        response = None
        response = self._stub.RegisterVehicle(update)
        print("response...")

        # signal main thread?


def register_with_opencda(queue, channel: grpc.Channel) -> None:
    opencda_client = Client(queue, channel)
    opencda_client.send_registration_to_opencda()
    logging.info("Registration with OpenCDA finished!")
    opencda_client._sim_state_watcher() # listen on stream

def run(q):
    #executor = ThreadPoolExecutor()
    with grpc.insecure_channel("localhost:50051") as channel:
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
    parser.add_argument('-i', "--ipaddress", type=str, default="127.0.0.1",
                        help="Specifies the ip address of the server to connect to. [Default: 127.0.0.1]")
    parser.add_argument('-p', "--port", type=int, default=5555,
                        help="Specifies the port to connect to. [Default: 5555]")

    opt = parser.parse_args()
    return opt

def main():
    # default params which can be over-written from the simulation controller
    application = ["single"]
    version = "0.9.12"

    opt = arg_parse()
    print("OpenCDA Version: %s" % __version__)

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

    print("waiting for opencda sim_api to start")
    opencda_started.wait(timeout=None) # send the start command below

    print("opencda sim_api start")

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

    print("main - test_scenario: " + test_scenario)
    print("main - application: " + application[0])
    print("main - version: " + version)

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    print("eCloud debug: creating VehicleManager vehicle_index: " + str(vehicle_index))

    vehicle_manager = VehicleManager(vehicle_index, test_scenario, application, cav_world, version)

    # send gRPC in response to start
    with lock:
        actor_id = vehicle_manager.vehicle.id
        vid = vehicle_manager.vid
        q.put(actor_id)
        q.put(vid)

    message = {"actor_id": actor_id, "vid": vid}
    print(f"Vehicle: Sending id {message}")

    carla_vehicle_created.set()

    #_socket.send(json.dumps(message).encode('utf-8'))

    while True:
        time.sleep(10)
        print('ready to run scenario testing...')

    # run scenario testing --> replace with event-based on streaming connection
    while(True):
        message = json.loads(_socket.recv(1024).decode('utf-8'))
        cmd = message["cmd"]
        if cmd != "TICK": # don't print tick message since there are too many
            print(f"Vehicle: received cmd {cmd}")

        if cmd == "update_info":
            vehicle_manager.update_info()
            _socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))
        elif cmd == "set_destination":
            destination = message["params"]
            print(f"Vehicle: x=%s" % destination["start"]["x"])
            start_location = carla.Location(x=destination["start"]["x"], y=destination["start"]["y"], z=destination["start"]["z"])
            end_location = carla.Location(x=destination["end"]["x"], y=destination["end"]["y"], z=destination["end"]["z"])
            clean = bool(destination["clean"])
            end_reset = bool(destination["reset"])
            vehicle_manager.set_destination(start_location, end_location, clean, end_reset)
            _socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))
        elif cmd == "TICK":
            vehicle_manager.update_info()
            control = vehicle_manager.run_step()
            if control is None:
                _socket.send(json.dumps({"resp": "DONE"}).encode('utf-8'))
            else:
                vehicle_manager.apply_control(control)
                _socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))
        elif cmd == "END":
            break
    
    vehicle_manager.destroy()
    _socket.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
