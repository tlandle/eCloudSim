# -*- coding: utf-8 -*-
"""
Script to run different scenarios.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
#       : Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import argparse
import importlib
import os
import sys
import subprocess
import logging

from opencda.version import __version__

DEFAULT_SCENARIO="ecloud_4lane_scenario_dist_config"
logger = logging.getLogger("ecloud")

def arg_parse():
    parser = argparse.ArgumentParser(description="eCloudSim scenario runner.")
    parser.add_argument('-t', "--test_scenario", type=str, default=DEFAULT_SCENARIO,
                        help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'opencda/scenario_testing/ folder'
                             f' as well as the corresponding yaml file in opencda/scenario_testing/config_yaml. [Default: {DEFAULT_SCENARIO}]')
    parser.add_argument("--record", action='store_true', help='whether to record and save the simulation process to'
                                                              '.log file')
    parser.add_argument('-n', "--num_cars", type=int, default=0,
                        help="number of vehicles to run."
                             "forces RANDOM spawning behavior")
    parser.add_argument('-d', "--distributed", type=int, default=1,
                        help="run a distributed scenario.")
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.'
                             'NOT compatible with distributed scenarios: containers must be started at runtime with perception enabled.')
    parser.add_argument('-l', "--log_level", type=int, default=0,
                            help="0: DEBUG | 1: INFO | WARNING: 2 | ERROR: 3")
    parser.add_argument('-b', "--build", action="store_true",
                            help="Rebuild gRPC proto files")
    parser.add_argument('-s', "--steps", type=int, default=0,
                            help="Number of scenario ticks to execute before exiting; if set, overrides scenario config")
    parser.add_argument('-e', "--environment", type=str, default="local",
                            help="Environment to run in: 'local' or 'azure'. [Default: 'local']")
    parser.add_argument('-c', "--run_carla", type=str, nargs='?', default=False, const=" ",
                            help="Run Carla with optional args: 'offscreen' --> -RenderOffscreen")
    parser.add_argument("--version", type=str, default="0.9.12") # only support version
    opt = parser.parse_args()
    return opt

def main():
    opt = arg_parse()
    print (opt)
    print(f"eCloudSim Version: {__version__}")

    try:
        testing_scenario = importlib.import_module("opencda.scenario_testing.%s" % opt.test_scenario)
    except ModuleNotFoundError:
        sys.exit("ERROR: %s.py not found under opencda/scenario_testing" % opt.test_scenario)

    config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'opencda/scenario_testing/config_yaml/%s.yaml' % opt.test_scenario)
    if not os.path.isfile(config_yaml):
        sys.exit("opencda/scenario_testing/config_yaml/%s.yaml not found!" % opt.test_scenario)

    # eCLoud
    if opt.build:
        subprocess.run(['python','-m','grpc_tools.protoc','-I./opencda/protos','--python_out=.','--grpc_python_out=.','./opencda//protos/ecloud.proto'])
    # ------

    scenario_runner = getattr(testing_scenario, 'run_scenario')
    # run scenario testing
    scenario_runner(opt, config_yaml)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logger.info(' - Exited by user.')
