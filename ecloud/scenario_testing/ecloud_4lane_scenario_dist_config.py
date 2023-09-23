# -*- coding: utf-8 -*-
"""
eCloudSim
---------
Scenario testing: *TEMPLATE*
Use for OpenCDA vs eCloudSim DIST-ONLY comparisons

*NOT for Edge*

Town 06 Scenarios *ONLY*

DO NOT USE for 2-Lane Free
"""
# Author: Jordan Rapp, Dean Blank, Tyler Landle <Georgia Tech>
# License: TDG-Attribution-NonCommercial-NoDistrib

# Core
import os
import time
import asyncio
import logging
import sys

# 3rd Party
import carla

# OpenCDA Utils
import ecloud.scenario_testing.utils.sim_api as sim_api
from ecloud.scenario_testing.utils.yaml_utils import load_yaml
from ecloud.core.common.cav_world import CavWorld
from ecloud.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
# ONLY *required* for 2 Lane highway scenarios
# import ecloud.scenario_testing.utils.customized_map_api as map_api

from ecloud.core.common.ecloud_config import EcloudConfig

import ecloud_pb2 as ecloud

# Consts
LOG_NAME = f"{os.path.basename(__file__)}.log"
SCENARIO_NAME = f"{os.path.basename(__file__)}"
TOWN = 'Town06'

logger = logging.getLogger("ecloud")

def run_scenario(opt, config_yaml):
    '''
    scenario runner for default eCloud scenario on Carla's Town 06
    '''
    step_count = 0
    step = 0
    run_distributed = opt.distributed

    try:
        scenario_params = load_yaml(config_yaml)

        if opt.num_cars != 0:
            assert 'ecloud' in scenario_params['scenario']
            scenario_params['scenario']['ecloud']['num_cars'] = opt.num_cars
            scenario_params['scenario']['ecloud']['location_type'] = 'random'

        ecloud_config = EcloudConfig(scenario_params)
        ecloud_config.set_fatal_errors(opt.fatal_errors)
        step_count = ecloud_config.get_step_count() if opt.steps == 0 else opt.steps
        # sanity checks...
        assert 'edge_list' not in scenario_params['scenario'] # do NOT use this template for edge scenarios
        assert 'sync_mode' in scenario_params['world'] and scenario_params['world']['sync_mode'] is True
        assert scenario_params['world']['fixed_delta_seconds'] == 0.03 \
               or scenario_params['world']['fixed_delta_seconds'] == 0.05

        # spectator configs
        world_x = scenario_params['world']['x_pos'] if 'x_pos' in scenario_params['world'] else 0
        world_y = scenario_params['world']['y_pos'] if 'y_pos' in scenario_params['world'] else 0
        world_z = scenario_params['world']['z_pos'] if 'z_pos' in scenario_params['world'] else 256
        world_roll = scenario_params['world']['roll'] if 'roll' in scenario_params['world'] else 0
        world_pitch = scenario_params['world']['pitch'] if 'pitch' in scenario_params['world'] else -90
        world_yaw = scenario_params['world']['yaw'] if 'yaw' in scenario_params['world'] else 0

        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=TOWN,
                                                   cav_world=cav_world,
                                                   distributed=run_distributed,
                                                   log_level=opt.log_level,
                                                   ecloud_config=ecloud_config,
                                                   run_carla=opt.run_carla)

        if opt.record:
            scenario_manager.client. \
                start_recorder(LOG_NAME, True)

        # create single cavs
        if run_distributed:
            asyncio.get_event_loop().run_until_complete(scenario_manager.run_comms())
            single_cav_list = \
                scenario_manager.create_distributed_vehicle_manager(application=['single'])

        else:
            single_cav_list = \
                scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        _, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name=SCENARIO_NAME,
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()

        flag = True
        while flag:
            print(f"ticking - step: {step}")
            scenario_manager.tick_world()
            if run_distributed:
                flag = scenario_manager.broadcast_tick()

            else:
                # non-dist will break automatically; don't need to set flag
                pre_client_tick_time = time.time()
                for _, single_cav in enumerate(single_cav_list):
                    single_cav.update_info()
                    control = single_cav.run_step()
                    single_cav.apply_control(control)
                post_client_tick_time = time.time()

                logger.info("client tick completion time: %s", (post_client_tick_time - pre_client_tick_time))
                if step > 0: # discard the first tick as startup is a major outlier
                    scenario_manager.debug_helper.update_client_tick((post_client_tick_time - pre_client_tick_time)*1000)

            # same for dist / non-dist - only required for specate
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    x=world_x,
                    y=world_y,
                    z=world_z),
                carla.Rotation(
                    yaw=world_yaw,
                    roll=world_roll,
                    pitch=world_pitch)))

            step = step + 1
            if step > step_count:
                if run_distributed:
                    flag = scenario_manager.broadcast_message(ecloud.Command.REQUEST_DEBUG_INFO)
                break

    except Exception as scenario_error:
        if isinstance(scenario_error, SystemExit):
            logger.info('system exit: %s', scenario_error)
            sys.exit(scenario_error)

        else:
            logger.critical("exception hit during scenario execution - %s", type(scenario_error))
            if opt.fatal_errors:
                raise

    else:
        if run_distributed:
            scenario_manager.end() # only dist requires explicit scenario end call

        if step > step_count:
            eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        if not run_distributed:
            for veh in single_cav_list:
                try:
                    veh.destroy()
                except Exception as destroy_error:
                    logger.error('%s: failed to destroy single CAV', type(destroy_error))

        for veh in bg_veh_list:
            logger.warning("destroying background vehicle")
            try:
                veh.destroy()
            except Exception as destroy_error:
                logger.warning("%s: failed to destroy background vehicle", type(destroy_error))

    #finally:
