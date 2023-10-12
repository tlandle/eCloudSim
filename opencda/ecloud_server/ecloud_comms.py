from concurrent.futures import ThreadPoolExecutor
import coloredlogs, logging
import time
from typing import Iterator
import os
import sys
import json
import asyncio

import carla

from opencda.scenario_testing.utils.yaml_utils import load_yaml

import grpc
from google.protobuf.json_format import MessageToJson
from google.protobuf.timestamp_pb2 import Timestamp

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

cloud_config = load_yaml("cloud_config.yaml")
if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

ECLOUD_PUSH_BASE_PORT = 50101 # TODO: config

class EcloudClient:

    '''
    Wrapper Class around gRPC Vehicle Client Calls
    '''

    retry_opts = json.dumps({
                    "methodConfig": [
                    {
                        "name": [{"service": "ecloud.Ecloud"}],
                        "retryPolicy": {
                            "maxAttempts": 5,
                            "initialBackoff": "0.05s",
                            "maxBackoff": "0.5s",
                            "backoffMultiplier": 2,
                            "retryableStatusCodes": ["UNAVAILABLE"],
                        },
                    }]})

    def __init__(self, channel: grpc.Channel) -> None:
        self.channel = channel
        self.stub = ecloud_rpc.EcloudStub(self.channel)

    async def run(self) -> ecloud.Tick:
        count = 0
        pong = None
        async for ecloud_update in self.stub.SimulationStateStream(ecloud.Tick( tick_id = self.tick_id )):
            logger.debug("T%s:C%s", ecloud_update.tick_id, ecloud_update.command)
            assert(self.tick_id != ecloud_update.tick_id)
            self.tick_id = ecloud_update.tick_id
            count += 1
            pong = ecloud_update

        assert(pong != None)
        assert(count == 1)
        return pong

    async def register_vehicle(self, update: ecloud.VehicleUpdate) -> ecloud.SimulationInfo:
        sim_info = await self.stub.Client_RegisterVehicle(update)

        return sim_info

    async def send_vehicle_update(self, update: ecloud.VehicleUpdate) -> ecloud.Empty:
        empty = await self.stub.Client_SendUpdate(update)

        return empty

    async def get_waypoints(self, request: ecloud.WaypointRequest) -> ecloud.WaypointBuffer:
        buffer = await self.stub.Client_GetWaypoints(request)

        return buffer

class EcloudPushServer(ecloud_rpc.EcloudServicer):

    '''
    Lightweight gRPC Server Class for Receiving Push Messages from Ochestrator
    '''

    def __init__(self,
                 q: asyncio.Queue):

        logger.info("eCloud push server initialized")
        self.q = q
        self.last_tick = 0

    async def PushTick(self,
                       tick: ecloud.Tick,
                       context: grpc.aio.ServicerContext) -> ecloud.Empty:

        if tick.tick_id != ( self.last_tick + 1 ) and tick.tick_id > 0 and self.last_tick > 0 and tick.command == ecloud.Command.TICK:
            logger.error('received an out of sync tick. had %s | received %s', self.last_tick, tick.tick_id)
        elif tick.tick_id:
            self.last_tick = tick.tick_id

        logger.debug("PushTick(): tick - %s", tick)
        #assert(self.q.empty())
        if not self.q.empty():
            t = self.q.get_nowait()
            logger.error('received tick %s while %s was already present', tick, t)
        else:
            self.q.put_nowait(tick)

        return ecloud.Empty()

async def ecloud_run_push_server(port,
                                 que: asyncio.Queue) -> None:
    '''
    runs a simple listen server that accepts event-based message from the central ecloud gRPC server
    '''

    logger.info("spinning up eCloud push server")
    server = grpc.aio.server()
    ecloud_rpc.add_EcloudServicer_to_server(EcloudPushServer(que), server)
    try:
        listen_addr = f"0.0.0.0:{port}"
        server.add_insecure_port(listen_addr)
    except Exception as port_exception: # pylint: disable=broad-exception-caught
        logger.error("failed - %s: %s - to start push server on port %s - incrementing port & retrying",
                     type(port_exception), port_exception, port)
        port += 1

    logger.critical("starting eCloud push server on port %s", port)

    if port >= ECLOUD_PUSH_BASE_PORT:
        que.put_nowait(port)

    await server.start()
    await server.wait_for_termination()
