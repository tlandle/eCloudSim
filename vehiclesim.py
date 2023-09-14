# -*- coding: utf-8 -*-
"""
Script to run a simulated vehicle
"""

# Authors: Aaron Drysdale <adrysdale3@gatech.edu>
#        : Jordan Rapp <jrapp7@gatech.edu>


import argparse
import sys
import json
import asyncio
import os
import logging
import threading
import time
import queue

import carla
import numpy as np
import coloredlogs

from opencda.version import __version__
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.core.application.edge.transform_utils import *
from opencda.core.plan.local_planner_behavior import RoadOption
from opencda.core.plan.global_route_planner import GlobalRoutePlanner
from opencda.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO
from opencda.core.common.ecloud_config import EcloudConfig, eDoneBehavior
from opencda.scenario_testing.utils.yaml_utils import load_yaml

from opencda.ecloud_server.ecloud_comms import EcloudClient, EcloudPushServer, ecloud_run_push_server

import grpc
from google.protobuf.json_format import MessageToJson
from google.protobuf.timestamp_pb2 import Timestamp

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]
ECLOUD_IP = cloud_config["ecloud_server_public_ip"]
ECLOUD_PUSH_BASE_PORT = 50101 # TODO: config

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

async def send_registration_to_ecloud_server(stub_):
    request = ecloud.VehicleUpdate()
    request.vehicle_state = ecloud.VehicleState.REGISTERING
    try:
        request.container_name = os.environ["HOSTNAME"]
    except Exception as e:
        request.container_name = f"vehiclesim.py"    
    
    response = await stub_.Client_RegisterVehicle(request)

    logger.info(f"vehicle ID {response.vehicle_index} received...")
    assert response.state == ecloud.State.NEW
    
    return response

async def send_carla_data_to_opencda(stub_, vehicle_index, actor_id, vid):
    message = {"vehicle_index": vehicle_index, "actor_id": actor_id, "vid": vid}
    logger.info(f"Vehicle: Sending Carla rpc {message}")

    # send actor ID and vid to API
    update = ecloud.VehicleUpdate()
    update.vehicle_state = ecloud.VehicleState.CARLA_UPDATE
    update.vehicle_index = vehicle_index
    update.vid = vid
    update.actor_id = actor_id
    
    response = await stub_.Client_RegisterVehicle(update)

    logger.info(f"send_carla_data_to_opencda: response received")

    return response

async def send_vehicle_update(stub_, vehicle_update_):
    response = await stub_.Client_SendUpdate(vehicle_update_)

    #logger.info(f"send_vehicle_update: response received")

    return response

def arg_parse():
    parser = argparse.ArgumentParser(description="OpenCDA Vehicle Simulation.")
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-i', "--ipaddress", type=str, default=CARLA_IP,
                        help="Specifies the ip address of the server to connect to. [Default: localhost]")
    parser.add_argument('-p', "--port", type=int, default=50051,
                        help="Specifies the port to connect to. [Default: 50051]")
    parser.add_argument('-v', "--verbose", action="store_true",
                            help="Make more noise")
    parser.add_argument('-q', "--quiet", action="store_true",
                            help="Make no noise")

    opt = parser.parse_args()
    return opt

async def main():
    # default params which can be over-written from the simulation controller
    SPECTATOR_INDEX = 0

    application = ["single"]
    version = "0.9.12"
    tick_id = 0
    state = ecloud.State.UNDEFINED #do we need a global state?
    reported_done = False
    push_q = asyncio.Queue()

    opt = arg_parse()
    if opt.verbose:
        logger.setLevel(logging.DEBUG)
    elif opt.quiet:
        logger.setLevel(logging.WARNING)
    logger.info(f"OpenCDA Version: {version}")

    logging.basicConfig()

    channel = grpc.aio.insecure_channel(
        target=f"{ECLOUD_IP}:{opt.port}",
        options=[
            ("grpc.lb_policy_name", "pick_first"),
            ("grpc.enable_retries", 0),
            ("grpc.keepalive_timeout_ms", 10000),
        ],
    )
    ecloud_server = ecloud_rpc.EcloudStub(channel)
    ecloud_update = await send_registration_to_ecloud_server(ecloud_server)
    vehicle_index = ecloud_update.vehicle_index
    state = ecloud_update.state
    assert( vehicle_index != None )

    test_scenario = ecloud_update.test_scenario
    application = ecloud_update.application
    version = ecloud_update.version

    logger.debug(f"main - application: {application}")
    logger.debug(f"main - version: {version}")

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    logger.info(f"eCloud debug: creating VehicleManager vehicle_index: {vehicle_index}")

    scenario_yaml = json.loads(test_scenario) #load_yaml(test_scenario)
    if 'debug_scenario' in scenario_yaml:
        logger.debug(f"main - test_scenario: {test_scenario}") # VERY verbose

    # spawn push server
    push_port = ECLOUD_PUSH_BASE_PORT + vehicle_index
    push_server = asyncio.create_task(ecloud_run_push_server(push_port, push_q))

    await asyncio.sleep(1)

    ecloud_config = EcloudConfig(scenario_yaml, logger)
    SPAWN_SLEEP_TIME = ecloud_config.get_client_spawn_ping_time_s()
    TICK_SLEEP_TIME = ecloud_config.get_client_tick_ping_time_s()
    WORLD_TIME_SLEEP_FACTOR = ecloud_config.get_client_world_tick_factor()
    NUM_SERVERS = ecloud_config.get_num_servers()
    NUM_PORTS = ecloud_config.get_num_ports()

    location_type = ecloud_config.get_location_type()
    done_behavior = ecloud_config.get_done_behavior()

    target_speed = None
    edge_sets_destination = False
    is_edge = False # TODO: added this to the actual protobuf message
    if 'edge_list' in scenario_yaml['scenario']:
        is_edge = True
        # TODO: support multiple edges... 
        target_speed = scenario_yaml['scenario']['edge_list'][0]['target_speed']
        edge_sets_destination = scenario_yaml['scenario']['edge_list'][0]['edge_sets_destination'] \
            if 'edge_sets_destination' in scenario_yaml['scenario']['edge_list'][0] else False

    if opt.apply_ml:
        await asyncio.sleep(vehicle_index + 1)

    vehicle_manager = VehicleManager(vehicle_index=vehicle_index, config_yaml=scenario_yaml, application=application, cav_world=cav_world, \
                                     carla_version=version, location_type=location_type, run_distributed=True, is_edge=is_edge)

    actor_id = vehicle_manager.vehicle.id
    vid = vehicle_manager.vid

    ecloud_update = await send_carla_data_to_opencda(ecloud_server, vehicle_index, actor_id, vid)

    assert(push_q.empty())
    await push_q.get()
    push_q.task_done()

    vehicle_manager.update_info()
    vehicle_manager.set_destination(
                vehicle_manager.vehicle.get_location(),
                vehicle_manager.destination_location,
                clean=True)

    logger.info(f"vehicle {vehicle_index} beginning scenario tick flow")
    waypoint_proto = None
    while state != ecloud.State.ENDED:   
        
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
            client_start_timestamp = Timestamp()
            client_start_timestamp.GetCurrentTime()
            # update info runs BEFORE waypoint injection
            update_info_start_time = time.time()
            vehicle_manager.update_info()
            update_info_end_time = time.time()
            vehicle_manager.debug_helper.update_update_info_time((update_info_end_time-update_info_start_time)*1000)
            logger.debug("update_info complete")

            if is_edge:               
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

                    waypoint_proto = None

                cur_location = vehicle_manager.vehicle.get_location()
                logger.debug(f"location for vehicle_{vehicle_index} - is - x: {cur_location.x}, y: {cur_location.y}")

                waypoints_buffer_printer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
                for waypoints in waypoints_buffer_printer:
                    logger.debug("waypoint_proto: waypoints transform for Vehicle: %s", waypoints[0].transform)

            #waypoints_buffer_printer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
            #for waypoints in waypoints_buffer_printer:
            #    logger.warning("final: waypoints transform for Vehicle: %s", waypoints[0].transform)

            should_run_step = False
            if not is_edge or ( has_not_cleared_buffer and waypoint_proto == None ) or ( ( not has_not_cleared_buffer ) and waypoint_proto != None ):
                should_run_step = True

            if should_run_step:
                if reported_done:
                   target_speed = 0 
                control = vehicle_manager.run_step(target_speed=target_speed)
                logger.debug("run_step complete")

            vehicle_update = ecloud.VehicleUpdate()
            vehicle_update.tick_id = tick_id
            vehicle_update.vehicle_index = vehicle_index
            vehicle_update.client_start_tstamp.CopyFrom(client_start_timestamp)
            vehicle_update.sm_start_tstamp.CopyFrom(ecloud_update.sm_start_tstamp)
            
            if should_run_step:
                if control is None or vehicle_manager.is_close_to_scenario_destination():
                    vehicle_update.vehicle_state = ecloud.VehicleState.TICK_DONE
                    if not reported_done:
                        serialize_debug_info(vehicle_update, vehicle_manager)

                    if control is not None and done_behavior == eDoneBehavior.CONTROL:
                        vehicle_manager.apply_control(control)

                else:
                    vehicle_manager.apply_control(control)
                    logger.debug("apply_control complete")
                    vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK
                    vehicle_update.client_end_tstamp.GetCurrentTime()
                    #_socket.send(json.dumps({"resp": "OK"}).encode('utf-8'))

                if is_edge or vehicle_index == SPECTATOR_INDEX:
                    velocity = vehicle_manager.vehicle.get_velocity()
                    pv = ecloud.Velocity()
                    pv.x = velocity.x
                    pv.y = velocity.y
                    pv.z = velocity.z
                    vehicle_update.velocity.CopyFrom(pv)

                    transform = vehicle_manager.vehicle.get_transform()
                    pt = ecloud.Transform()
                    pt.location.x = transform.location.x
                    pt.location.y = transform.location.y
                    pt.location.z = transform.location.z
                    pt.rotation.roll = transform.rotation.roll
                    pt.rotation.yaw = transform.rotation.yaw
                    pt.rotation.pitch = transform.rotation.pitch
                    vehicle_update.transform.CopyFrom(pt)
            
            else:
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK # TODO: make a WP error status

            #cur_location = vehicle_manager.vehicle.get_location()
            #logger.debug(f"send OK and location for vehicle_{vehicle_index} - is - x: {cur_location.x}, y: {cur_location.y}")   

        # HANDLE END
        elif ecloud_update.command == ecloud.Command.END:
            logger.info("END received")
            break
        
        # block waiting for a response
        if not reported_done or done_behavior == eDoneBehavior.CONTROL:
            if not reported_done:
                last_command = ecloud_update.command
                ecloud_update = await send_vehicle_update(ecloud_server, vehicle_update)
                        
            assert(push_q.empty())
            pong = await push_q.get()
            push_q.task_done()
            assert( pong.tick_id != tick_id )
            tick_id = pong.tick_id

            if pong.command == ecloud.Command.REQUEST_DEBUG_INFO:
                ecloud_update.command = ecloud.Command.REQUEST_DEBUG_INFO

            elif pong.command == ecloud.Command.PULL_WAYPOINTS_AND_TICK:
                wp_request = ecloud.WaypointRequest()
                wp_request.vehicle_index = vehicle_index
                waypoint_proto = await ecloud_server.Client_GetWaypoints(wp_request)
                ecloud_update.command = ecloud.Command.TICK    

            if vehicle_update.vehicle_state == ecloud.VehicleState.TICK_DONE or vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE:
                if vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE and last_command == ecloud.Command.REQUEST_DEBUG_INFO:
                    # we were asked for debug data and provided it, so NOW we exit
                    # TODO: this is better handled by done
                    logger.info(f"pushed DEBUG_INFO_UPDATE")
                    break

                else:
                    reported_done = True
                    logger.info(f"reported_done")
                
        else: # done
            break

    # end while    
    vehicle_manager.destroy()
    push_server.cancel()
    await asyncio.gather(*push_server, return_exceptions=True)  
    logger.info("scenario complete. exiting.")
    sys.exit(0)

if __name__ == '__main__':
    try:
        asyncio.get_event_loop().run_until_complete(main())
    except KeyboardInterrupt:
        logger.info(' - Exited by user.')
