# -*- coding: utf-8 -*-
"""
Script to run a simulated vehicle
"""

# Author: Aaron Drysdale <adrysdale3@gatech.edu>
import argparse
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

from google.protobuf.json_format import MessageToJson
import grpc

# sys.path.append('../../protos/')

import sim_api_pb2 as sim_state
import sim_api_pb2_grpc as rpc
#end gRPC

class Client:

    def __init__(self, executor: ThreadPoolExecutor, channel: grpc.Channel) -> None:
        self._executor = executor
        self._channel = channel
        self._stub = rpc.OpenCDAStub(self._channel)
        self._vehicle_id = None
        self._sim_state = None
        self._opencda_responded = threading.Event()
        self._sim_finished = threading.Event()
        self._consumer_future = None

    def _sim_state_watcher(
            self,
            response_iterator: Iterator[sim_state.SimulationState]) -> None:
        try:
            for response in response_iterator:
                # NOTE: All fields in Proto3 are optional. This is the recommended way
                # to check if a field is present or not, or to exam which one-of field is
                # fulfilled by this message.
                if response.HasField("call_info"):
                    self._on_call_info(response.call_info)
                elif response.HasField("call_state"):
                    self._on_call_state(response.call_state.state)
                else:
                    raise RuntimeError(
                        "Received StreamCallResponse without call_info and call_state"
                    )
        except Exception as e:
            self._opencda_responded.set()
            raise

    def _on_call_info(self, call_info: phone_pb2.CallInfo) -> None:
        print("received stream message...")
        self._session_id = call_info.session_id
        self._audio_session_link = call_info.media

    def _on_call_state(self, call_state: phone_pb2.CallState.State) -> None:
        logging.info("Call toward [%s] enters [%s] state", self._phone_number,
                     phone_pb2.CallState.State.Name(call_state))
        self._call_state = call_state
        if call_state == phone_pb2.CallState.State.ACTIVE:
            self._opencda_responded.set()
        if call_state == phone_pb2.CallState.State.ENDED:
            self._opencda_responded.set()
            #self._call_finished.set()

    def call(self) -> None:
        request = phone_pb2.StreamCallRequest()
        request.phone_number = self._phone_number
        response_iterator = self._stub.StreamCall(iter((request,)))
        # Instead of consuming the response on current thread, spawn a consumption thread.
        self._consumer_future = self._executor.submit(self._sim_state_watcher,
                                                      response_iterator)

    def wait_opencda(self) -> bool:
        logging.info("Waiting for OpenCDA to connect...")
        self._opencda_responded.wait(timeout=None)
        if self._consumer_future.done():
            # If the future raises, forwards the exception here
            self._consumer_future.result()
        return self._call_state == sim_state.SimulationState.State.ACTIVE

    def subscribe_to_sim_state(self) -> None:
        assert self._audio_session_link is not None
        logging.info("Consuming audio resource [%s]", self._audio_session_link)
        self._call_finished.wait(timeout=None)
        logging.info("Audio session finished [%s]", self._audio_session_link)


def register_with_opencda(executor: ThreadPoolExecutor, channel: grpc.Channel) -> None:
    opencda_client = Client(executor, channel)
    opencda_client.call() # register with opencda
    if opencda_client.wait_opencda():
        opencda_client.subscribe_to_sim_state()
        logging.info("Call finished!")
    else:
        logging.info("Call failed: peer didn't answer")


def run():
    executor = ThreadPoolExecutor()
    with grpc.insecure_channel("localhost:50051") as channel:
        future = executor.submit(register_with_opencda, executor, channel,
                                 "555-0100-XXXX")
        future.result()

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
    vehicle_index = 0
    application = ["single"]
    version = "0.9.11"

    opt = arg_parse()
    print("OpenCDA Version: %s" % __version__)

    # Use zmq for interprocess communication between OpenCDA and each vehicle
#    context = zmq.Context()
#    socket = context.socket(zmq.REP)
#    socket.bind(f"tcp://*:{opt.port}")

    # gRPC start

    logging.basicConfig()
    client_thread = threading.Thread(target=run, daemon=True)
    client_thread.start()

    # gRPC end

    _socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    _socket.connect((opt.ipaddress, opt.port))

    # Wait for the START message from OpenCDA before going any further
    print(f"Vehicle created. Waiting for start...", flush=True)
    message = json.loads(_socket.recv(1024).decode('utf-8'))
    print(f"Vehicle: received cmd {message}")
    if message["cmd"] == "start":
        test_scenario = message["params"]["scenario"]
        vehicle_index = message["params"]["vehicle"]
        application = message["params"]["application"]
        version = message["params"]["version"]

    if not os.path.isfile(test_scenario):
        sys.exit("%s not found!" % test_scenario)

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    vehicle_manager = VehicleManager(vehicle_index, test_scenario, application, cav_world, version)

    message = {"actor_id": vehicle_manager.vehicle.id, "vid": vehicle_manager.vid}
    print(f"Vehicle: Sending id {message}")
    _socket.send(json.dumps(message).encode('utf-8'))

    # run scenario testing
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
