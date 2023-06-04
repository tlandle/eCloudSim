# -*- coding: utf-8 -*-
"""
Scenario testing: two vehicle driving in the customized 2 lane highway map.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os

import carla
from opencda.core.common.vehicle_manager_proxy import VehicleManagerProxy

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api

from opencda.core.common.cav_world import CavWorld
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

        print("scenario manager created...", flush=True)                                           

        if opt.record:
            scenario_manager.client. \
                start_recorder("ecloud_4lane.log", True)

        single_cav_list = \
            scenario_manager.create_distributed_vehicle_manager(application=['single'],
                                                    map_helper=map_api.
                                                    spawn_helper_2lanefree)

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='ecloud_4lane_dist_2_car',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        # run steps
        flag = True
        while flag:
            scenario_manager.tick_world()

            flag = scenario_manager.broadcast_tick()

            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=120),
                carla.Rotation(
                    pitch=-
                    90)))

            for _, single_cav in enumerate(single_cav_list):
                single_cav.update_info()

    finally:
        
        scenario_manager.end()

        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            print("destroying single CAV")
            try:
                v.destroy()
            except:
                print("failed to destroy single CAV")    
        for v in bg_veh_list:
            print("destroying background vehicle")
            try:
                v.destroy()
            except:
                print("failed to destroy background vehicle")  