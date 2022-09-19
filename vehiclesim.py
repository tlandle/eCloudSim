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
import asyncio
import logging

# sys.path.append('../../protos/')

import grpc
import helloworld_pb2
import helloworld_pb2_grpc

async def run() -> None:
    async with grpc.aio.insecure_channel('localhost:50051') as channel:
        stub = helloworld_pb2_grpc.GreeterStub(channel)
        response = await stub.SayHello(helloworld_pb2.HelloRequest(name='you'))
    print("Greeter client received: " + response.message)

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
    asyncio.run(run())

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
