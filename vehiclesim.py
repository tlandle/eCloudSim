# -*- coding: utf-8 -*-
"""
Script to run a simulated vehicle
"""

# Author: Aaron Drysdale <adrysdale3@gatech.edu>
import argparse
import os
import sys
import zmq

import carla

from opencda.version import __version__
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager


def arg_parse():
    parser = argparse.ArgumentParser(description="OpenCDA Vehicle Simulation.")
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-p', "--port", type=int, default=5555,
                        help="Specifies the port to listen on, default is 5555")

    opt = parser.parse_args()
    return opt

def main():
    vehicle_index = 0
    application = ["single"]
    version = "0.9.11"

    opt = arg_parse()
    print("OpenCDA Version: %s" % __version__)

    # Use zmq for interprocess communication between OpenCDA and each vehicle
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://*:{opt.port}")

    # Wait for the START message from OpenCDA before going any further
    print(f"Vehicle created. Waiting for start on port {opt.port}...", flush=True)
    message = socket.recv_json()
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
    socket.send_json(message)

    # run scenario testing
    while(True):
        message = socket.recv_json()
        cmd = message["cmd"]
        if cmd != "TICK": # don't print tick message since there are too many
            print(f"Vehicle: received cmd {cmd}")

        if cmd == "update_info":
            vehicle_manager.update_info()
            socket.send_json({"resp": "OK"})
        elif cmd == "set_destination":
            destination = message["params"]
            print(f"Vehicle: x=%s" % destination["start"]["x"])
            start_location = carla.Location(x=destination["start"]["x"], y=destination["start"]["y"], z=destination["start"]["z"])
            end_location = carla.Location(x=destination["end"]["x"], y=destination["end"]["y"], z=destination["end"]["z"])
            clean = bool(destination["clean"])
            end_reset = bool(destination["reset"])
            vehicle_manager.set_destination(start_location, end_location, clean, end_reset)
            socket.send_json({"resp": "OK"})
        elif cmd == "TICK":
            vehicle_manager.update_info()
            control = vehicle_manager.run_step()
            if control is None:
                socket.send_json({"resp": "DONE"})
            else:
                vehicle_manager.apply_control(control)
                socket.send_json({"resp": "OK"})
        elif cmd == "END":
            break
    
    vehicle_manager.destroy()
    socket.close()
    context.term()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
