# -*- coding: utf-8 -*-
"""
Script to run different scenarios.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import argparse
import importlib
import os
import sys
import subprocess

from opencda.version import __version__
import coloredlogs, logging

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO', logger=logger)

def arg_parse():
    parser = argparse.ArgumentParser(description="OpenCDA scenario runner.")
    parser.add_argument('-t', "--test_scenario", required=True, type=str,
                        help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'opencda/scenario_testing/ folder'
                             ' as well as the corresponding yaml file in opencda/scenario_testing/config_yaml.')
    parser.add_argument("--record", action='store_true', help='whether to record and save the simulation process to'
                                                              '.log file')
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-v', "--version", type=str, default='0.9.11',
                        help='Specify the CARLA simulator version, default'
                             'is 0.9.11, 0.9.12 is also supported.')
    parser.add_argument("--verbose", action="store_true",
                            help="Make more noise")
    parser.add_argument('-q', "--quiet", action="store_true",
                            help="Make no noise")
    parser.add_argument('-b', "--build", action="store_true",
                            help="Rebuild gRPC proto files")
    opt = parser.parse_args()
    return opt


def main():
    opt = arg_parse()
    print("OpenCDA Version: %s" % __version__)

    try:
        testing_scenario = importlib.import_module(f"opencda.scenario_testing.{opt.test_scenario}")
    except ModuleNotFoundError:
        logger.exception('%s.py not found under opencda/scenario_testing', opt.test_scenario)
        sys.exit(1) 

    config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               f'opencda/scenario_testing/config_yaml/{opt.test_scenario}.yaml')
    if not os.path.isfile(config_yaml):
        logger.error("opencda/scenario_testing/config_yaml/%s.yaml not found!", opt.test_scenario)
        sys.exit(1)

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
         logger.info('exited by user.')
    except SystemExit as exit:
         logger.info('system exit - %s', exit)
    except:
        logger.exception('unhandled exception')
