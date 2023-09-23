
import logging
import json
import asyncio

import grpc

from ecloud.scenario_testing.utils.yaml_utils import load_yaml
import ecloud.globals as ecloud_globals

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger("ecloud")

class EcloudComms:
    '''
    static class containing comms definitions
    '''
    TIMEOUT_S = 10
    TIMEOUT_MS = TIMEOUT_S * 1000

    RETRY_OPTS = json.dumps({
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

    GRPC_OPTIONS = [("grpc.lb_policy_name", "pick_first"),
                    ("grpc.enable_retries", 1),
                    ("grpc.keepalive_timeout_ms", TIMEOUT_MS),
                    ("grpc.service_config", RETRY_OPTS)]

class EcloudClient:

    '''
    Wrapper Class around gRPC Vehicle Client Calls
    '''

    def __init__(self, channel: grpc.Channel) -> None:
        self.channel = channel
        self.stub = ecloud_rpc.EcloudStub(self.channel)     

    async def stream_updates(self) -> ecloud.Tick:
        count = 0
        pong = None
        async for ecloud_update in self.stub.SimulationStateStream(ecloud.Tick( tick_id = self.tick_id )):
            logger.debug(f"T{ecloud_update.tick_id}:C{ecloud_update.command}")
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
        self.port_no = 0

    async def PushTick(self, 
                       tick: ecloud.Tick, 
                       context: grpc.aio.ServicerContext) -> ecloud.Empty:

        if tick.tick_id != ( self.last_tick + 1 ) and tick.tick_id > 0 and self.last_tick > 0 and tick.command == ecloud.Command.TICK:
            logger.warning(f'received an out of sync tick: had {self.last_tick} | received {tick.tick_id}')
        elif tick.tick_id:
            self.last_tick = tick.tick_id

        logger.debug(f"PushTick(): tick - {tick}")
        #assert(self.q.empty())
        if not self.q.empty():
            t = self.q.get_nowait()
            if tick.tick_id == t.tick_id:
                logger.warning(f'received duplicate tick {tick} with same tick_id as tick in queue - discarding.')
            else:
                logger.error(f'received new tick {tick} but {t} was already in queue - discarding.')
        else:
            self.q.put_nowait(tick)

        return ecloud.Empty()     

async def ecloud_run_push_server(port, 
                                 q: asyncio.Queue) -> None:
    
    logger.info("spinning up eCloud push server")
    server = grpc.aio.server()
    ecloud_rpc.add_EcloudServicer_to_server(EcloudPushServer(q), server)
    try:    
        listen_addr = f"0.0.0.0:{port}"
        server.add_insecure_port(listen_addr)
    except:
        logger.error("failed to start push server on port %s - incrementing port & retying", port)
        port += 1

    print(f"starting eCloud push server on port {port}")
    
    if port >= ecloud_globals.__push_base_port__:
        q.put_nowait(port)
    
    await server.start()
    await server.wait_for_termination()
