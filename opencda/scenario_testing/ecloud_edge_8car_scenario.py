# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane
freeway sorely with carla Warning: You have to load the 2lanefreecomplete
map into your ue4 editor before running this
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


import carla

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
import opencda.logging_ecloud
import logging
import time

from opencda.core.common.cav_world import CavWorld


import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town='Town06',
                                                   cav_world=cav_world,
                                                   config_file=config_yaml)

        if opt.record:
            scenario_manager.client. \
                start_recorder("ecloud_4lane_8car.log", True)

        # create single cavs
        edge_list = \
            scenario_manager.create_edge_manager(application=['edge'],)

        # create background traffic in carla
        #traffic_manager, bg_veh_list = \
            #scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='ecloud_4lane_scenario',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = edge_list[0].vehicle_manager_list[0].vehicle

        # run steps

        eval_time = 0
        flag = True
        waypoint_buffer = []
        while flag:
            eval_time += 1
            # print("Stepping, ", eval_time*0.2)
            pre_tick  = time.time()

            scenario_manager.tick_world()

            waypoint_buffer.clear()
            for edge in edge_list:
              edge.update_information()
              waypoint_buffer = edge.run_step()

            scenario_manager.add_waypoint_buffer_to_tick(waypoint_buffer)

            flag = scenario_manager.broadcast_tick()
            
            post_tick = time.time()
            logging.debug("Scenario Manager Tick Time: %s"%(post_tick - pre_tick))
          
            pre_tick  = time.time()
            transform = spectator_vehicle.get_transform()
            post_tick  = time.time()
            logging.debug("Camera Getting Transform Time: %s"%(post_tick - pre_tick))   
            pre_tick = time.time()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=80),
                    carla.Rotation(
                        pitch=-
                        90)))
            post_tick = time.time()
            logging.debug("Camera Setting Transform Time: %s"%(post_tick - pre_tick))

            # for edge in edge_list:
            #   pre_tick = time.time()
            #   edge.update_information()
            #   post_tick = time.time()
            #   logging.debug("Edge update Information Time: %s"%(post_tick - pre_tick))
            #   pre_tick = time.time()

            #   waypoint_buffer = edge.run_step()
              
            #   post_tick = time.time()
            #   logging.debug("Edge Total Step Time: %s"%(post_tick - pre_tick))

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()
