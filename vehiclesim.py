# -*- coding: utf-8 -*-
"""
Script to run a simulated vehicle
"""

# Authors: Aaron Drysdale <adrysdale3@gatech.edu>
#        : Jordan Rapp <jrapp7@gatech.edu>


import argparse
from atexit import register
from curses import A_DIM
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

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc
#end gRPC

TIMEOUT_S = 60
TIMEOUT_MS = TIMEOUT_S * 1000

state = ecloud.State.UNDEFINED #do we need a global state?
vehicle_index = None
tick_id = 0

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

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

def serialize_debug_info(vehicle_update, vehicle_manager):
    planer_debug_helper = vehicle_manager.agent.debug_helper
    planer_debug_helper_msg = ecloud.PlanerDebugHelper()
    planer_debug_helper.serialize_debug_info(planer_debug_helper_msg)
    vehicle_update.planer_debug_helper.CopyFrom( planer_debug_helper_msg )

    loc_debug_helper = vehicle_manager.localizer.debug_helper
    loc_debug_helper_msg = ecloud.LocDebugHelper()
    loc_debug_helper.serialize_debug_info(loc_debug_helper_msg)
    vehicle_update.loc_debug_helper.CopyFrom( loc_debug_helper_msg )

    client_debug_helper = vehicle_manager.debug_helper
    #logger.debug(vehicle_manager.debug_helper.perception_time_list)
    client_debug_helper_msg = ecloud.ClientDebugHelper()
    client_debug_helper.serialize_debug_info(client_debug_helper_msg)
    vehicle_update.client_debug_helper.CopyFrom(client_debug_helper_msg)

def send_registration_to_ecloud_server(stub_):
    request = ecloud.VehicleUpdate()
    request.vehicle_state = ecloud.VehicleState.REGISTERING
    response = stub_.Client_RegisterVehicle(request)

    logger.debug(f"Vehicle ID {response.vehicle_index} received...")
    assert response.state == ecloud.State.NEW
    
    return response

def send_carla_data_to_opencda(stub_, vehicle_index, actor_id, vid):
    message = {"vehicle_index": vehicle_index, "actor_id": actor_id, "vid": vid}
    logger.debug(f"Vehicle: Sending Carla rpc {message}")

    # send actor ID and vid to API
    update = ecloud.VehicleUpdate()
    update.vehicle_state = ecloud.VehicleState.CARLA_UPDATE
    update.vehicle_index = vehicle_index
    update.vid = vid
    update.actor_id = actor_id
    
    response = stub_.Client_RegisterVehicle(update)

    return response

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

    logging.basicConfig()

    channel = grpc.insecure_channel(
        target=f"{CARLA_IP}:50051",
        options=[
            ("grpc.lb_policy_name", "pick_first"),
            ("grpc.enable_retries", 0),
            ("grpc.keepalive_timeout_ms", TIMEOUT_MS),
        ],
    )
    ecloud_server = ecloud_rpc.EcloudStub(channel)

    ecloud_update = send_registration_to_ecloud_server(ecloud_server)
    vehicle_index = ecloud_update.vehicle_index
    state = ecloud_update.state
    assert( vehicle_index != None )

    test_scenario = ecloud_update.test_scenario
    application = ecloud_update.application
    version = ecloud_update.version

    logger.debug(f"main - test_scenario: {test_scenario}")
    logger.debug(f"main - application: {application}")
    logger.debug(f"main - version: {version}")

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    logger.debug(f"eCloud debug: creating VehicleManager vehicle_index: {vehicle_index}")

    scenario_yaml = json.loads(test_scenario) #load_yaml(test_scenario)
    vehicle_manager = VehicleManager(vehicle_index=vehicle_index, config_yaml=scenario_yaml, application=application, cav_world=cav_world, carla_version=version)

    target_speed = None
    edge_sets_destination = False
    if 'edge_list' in scenario_yaml['scenario']:
        # TODO: support multiple edges... 
        target_speed = scenario_yaml['scenario']['edge_list'][0]['target_speed']
        edge_sets_destination = scenario_yaml['scenario']['edge_list'][0]['edge_sets_destination'] \
            if 'edge_sets_destination' in scenario_yaml['scenario']['edge_list'][0] else False

    actor_id = vehicle_manager.vehicle.id
    vid = vehicle_manager.vid

    ecloud_update = send_carla_data_to_opencda(ecloud_server, vehicle_index, actor_id, vid)

    if not edge_sets_destination:
        cav_config = scenario_yaml['scenario']['single_cav_list'][vehicle_index]
        destination = carla.Location(x=cav_config['destination'][0],
                                     y=cav_config['destination'][1],
                                     z=cav_config['destination'][2])
        vehicle_manager.update_info()
        vehicle_manager.set_destination(
                vehicle_manager.vehicle.get_location(),
                destination,
                clean=True)

    while state != ecloud.State.ENDED:   

        tick_id = ecloud_update.tick_id

        vehicle_update = ecloud.VehicleUpdate()
        if ecloud_update.command != ecloud.Command.TICK: # don't print tick message since there are too many
            logger.info(f"Vehicle: received cmd {ecloud_update.command}")
        
        # HANDLE DEBUG DATA REQUEST
        if ecloud_update.command == ecloud.Command.REQUEST_DEBUG_INFO:
            vehicle_update.tick_id = tick_id
            vehicle_update.vehicle_index = vehicle_index
            vehicle_update.vehicle_state = ecloud.VehicleState.DEBUG_INFO_UPDATE            
            serialize_debug_info(vehicle_update, vehicle_manager)
  
        # HANDLE TICK
        elif ecloud_update.command == ecloud.Command.TICK:
            # update info runs BEFORE waypoint injection
            vehicle_manager.update_info()

            # find waypoint buffer for our vehicle
            waypoint_proto = None
            for wpb in ecloud_update.all_waypoint_buffers:
                #logger.debug(wpb.SerializeToString())
                if wpb.vehicle_index == vehicle_index:
                    waypoint_proto = wpb
                    break
            
            is_wp_valid = False
            has_not_cleared_buffer = True
            if waypoint_proto != None:
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
                    is_wp_valid = vehicle_manager.agent.get_local_planner().is_waypoint_valid(waypoint=wp)
                    
                    if edge_sets_destination and is_wp_valid:
                        cur_location = vehicle_manager.vehicle.get_location()
                        start_location = carla.Location(x=cur_location.x, y=cur_location.y, z=cur_location.z)
                        end_location = carla.Location(x=wp.transform.location.x, y=wp.transform.location.y, z=wp.transform.location.z)
                        clean = True # bool(destination["clean"])
                        end_reset = True # bool(destination["reset"])
                        vehicle_manager.set_destination(start_location, end_location, clean, end_reset)

                    elif is_wp_valid:
                            if has_not_cleared_buffer:
                                # override waypoints
                                waypoint_buffer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
                                # print(waypoint_buffer)
                                # for waypoints in waypoint_buffer:
                                #   print("Waypoints transform for Vehicle Before Clearing: " + str(i) + " : ", waypoints[0].transform)
                                waypoint_buffer.clear() #EDIT MADE
                                has_not_cleared_buffer = False
                            waypoint_buffer.append((wp, RoadOption.STRAIGHT))

                cur_location = vehicle_manager.vehicle.get_location()
                logger.debug(f"location for vehicle_{vehicle_index} - is - x: {cur_location.x}, y: {cur_location.y}")

                waypoints_buffer_printer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
                for waypoints in waypoints_buffer_printer:
                    logger.warning("waypoint_proto: waypoints transform for Vehicle: %s", waypoints[0].transform)

            waypoints_buffer_printer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
            for waypoints in waypoints_buffer_printer:
                logger.warning("final: waypoints transform for Vehicle: %s", waypoints[0].transform)

            should_run_step = False
            if ( has_not_cleared_buffer and waypoint_proto == None ) or ( ( not has_not_cleared_buffer ) and waypoint_proto != None ):
                should_run_step = True

            if should_run_step:
                control = vehicle_manager.run_step(target_speed=target_speed)

            vehicle_update = ecloud.VehicleUpdate()
            vehicle_update.tick_id = tick_id
            vehicle_update.vehicle_index = vehicle_index
            
            if should_run_step:
                if control is None or vehicle_manager.is_close_to_scenario_destination():
                    
                    vehicle_update.vehicle_state = ecloud.VehicleState.TICK_DONE

                    serialize_debug_info(vehicle_update, vehicle_manager)

                else:

                    vehicle_manager.apply_control(control)
                    vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK
                    #_socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))
            
            else:
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK # TODO: make a WP error status

            cur_location = vehicle_manager.vehicle.get_location()
            logger.debug(f"send OK and location for vehicle_{vehicle_index} - is - x: {cur_location.x}, y: {cur_location.y}")   

        # HANDLE END
        elif ecloud_update.command == ecloud.Command.END:
            logger.info("END received")
            break
        
        # block waiting for a response
        ecloud_update = ecloud_server.Client_SendUpdate(vehicle_update)
        logger.debug(f"received tick: {ecloud_update.tick_id}")

    # end while    
    
    # vehicle_manager.destroy() # let the scenario manager destroy...
    #_socket.close()
    logger.info("scenario complete. exiting.")
    sys.exit(0)
    logger.info("this should not print...")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logger.info(' - Exited by user.')
