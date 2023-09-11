# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane
freeway sorely with carla Warning: You have to load the 2lanefreecomplete
map into your ue4 editor before running this
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
#       : Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import logging
import time

import carla

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
import opencda.logging_ecloud

from opencda.core.common.cav_world import CavWorld
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.common.ecloud_config import EcloudConfig

import ecloud_pb2 as ecloud

def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        cav_world = CavWorld(opt.apply_ml)
        ecloud_config = EcloudConfig(scenario_params)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town='Town06',
                                                   cav_world=cav_world,
                                                   config_file=config_yaml,
                                                   distributed=True)

        if opt.record:
            scenario_manager.client. \
                start_recorder("ecloud_edge_16_car.log", True)

        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']
        assert( edge_dt % world_dt == 0 ) # we need edge time to be an exact multiple of world time because we send waypoints every Nth tick

        # create single cavs
        edge_list = \
            scenario_manager.create_edge_manager(application=['edge'], edge_dt=edge_dt, world_dt=world_dt)

        # create background traffic in carla
        #traffic_manager, bg_veh_list = \
            #scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='ecloud_edge_scenario',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = edge_list[0].vehicle_manager_list[0].vehicle

        # run steps
        step = 0
        flag = True
        waypoint_buffer = []
        world_time = 0
        while flag:
            print("Step: %d" %step)
            scenario_manager.tick_world()

            world_time += world_dt

            if world_time % edge_dt == 0:
                world_time = 0

                waypoint_buffer.clear()
                for edge in edge_list:
                    edge.update_information()
                    waypoint_buffer = edge.run_step()

                scenario_manager.push_waypoint_buffer(waypoint_buffer)

            flag = scenario_manager.broadcast_tick()
            
            step = step + 1
            if step > ecloud_config.get_step_count():
                flag = scenario_manager.broadcast_message(ecloud.Command.REQUEST_DEBUG_INFO)
                break

            transform = spectator_vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=80),
                    carla.Rotation(
                        pitch=-
                        90)))

    finally:

        scenario_manager.end()

        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()       

        scenario_manager.close()
        for edge in edge_list:
            edge.destroy()
