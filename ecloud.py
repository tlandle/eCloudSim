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
import re
import traceback

from ecloud.globals import __version__, __ecloud__, __default_scenario__, EnvironmentConfig
import ecloud.globals as ecloud_globals

logger = logging.getLogger(__ecloud__)

import_module = re.compile(r'import ([\.A-Za-z0-9_-]+) ')
import_class = re.compile(r'from ([\.A-Za-z0-9_-]+) import')

FATAL_ERRORS = False

def arg_parse():
    '''
    fetch the command line args & returns an args object
    '''
    parser = argparse.ArgumentParser(description="eCloudSim scenario runner.")
    parser.add_argument('-t', "--test_scenario", type=str, default=__default_scenario__,
                        help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'ecloud/scenario_testing/ folder'
                             ' as well as the corresponding yaml file in ecloud/scenario_testing/config_yaml.'
                             f'[Default: {__default_scenario__}]')

    # CONFIGURATION ARGS
    parser.add_argument('-n', "--num_cars", type=int, default=0,
                            help="number of vehicles to run - forces RANDOM spawning behavior")
    parser.add_argument('-d', "--distributed", type=int, default=1,
                            help="run a distributed scenario.")
    parser.add_argument('-l', "--log_level", type=int, default=0,
                            help="0: DEBUG | 1: INFO | WARNING: 2 | ERROR: 3")
    parser.add_argument('-b', "--build", action="store_true",
                            help="Rebuild gRPC proto files")
    parser.add_argument('-s', "--steps", type=int, default=0,
                            help="Number of scenario ticks to execute before exiting; if set, overrides scenario config")
    parser.add_argument('-e', "--environment", type=str, default=ecloud_globals.__local__,
                            help=f"Environment to run in: 'local' or 'azure'. [Default: '{ecloud_globals.__local__}']")
    parser.add_argument('-r', "--run_carla", type=str, nargs='?', default=False, const=" ",
                            help="Run Carla with optional args; use = --run_carla='-RenderOffscreen'")
    parser.add_argument('-f', "--fatal_errors", action='store_true',
                        help="will raise exceptions when set to allow for easier debugging")

    # SEQUENTIAL ONLY
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.'
                             'NOT compatible with distributed scenarios:'
                             'containers must be started at runtime with perception enabled.')

    # DEPRECATED
    parser.add_argument("--record", action='store_true', help='whether to record and save the simulation process to'
                                                              '.log file')
    parser.add_argument("--version", type=str, default="0.9.12",
                            help="Carla version. [default: 0.9.12]") # only support version 0.9.12
    opt = parser.parse_args()
    return opt

def check_imports():
    '''
    debug helper function to scan for missing imports
    '''
    missing_imports = {}
    for (root,_,files) in os.walk(__ecloud__, topdown=True):
        for file in files:
            if file.endswith('.py'):
                # print(f"{file}")
                with open(os.path.join(root, file), 'r', encoding='utf-8') as f:
                    s = f.read()
                    for l in s.splitlines():
                        x = re.search(import_module, l)
                        if x:
                            module = x.group(1)
                            try:
                                importlib.import_module(module)
                            except Exception as me:
                                if module not in missing_imports and f"{me}" not in missing_imports.values():
                                    missing_imports[module] = f"{me}"
                                    logger.error("failed importing %s - %s", module, me)
                                continue
                            else:
                                logger.debug("module %s imported OK", module)

                        x = re.search(import_class, l)
                        if x:
                            try:
                                module = x.group(1)
                                importlib.import_module(module)
                            except Exception as ce:
                                if module not in missing_imports and f"{ce}" not in missing_imports.values():
                                    missing_imports[module] = f"{ce}"
                                    logger.error("failed importing %s - %s", module, ce)
                                continue
                            else:
                                logger.debug("module %s imported OK", module)

def get_scenario(opt):
    '''
    fetch the desired scenario module & associatd YAML
    '''
    testing_scenario = None
    config_yaml = None
    error = None
    try:
        testing_scenario = importlib.import_module(f"ecloud.scenario_testing.{opt.test_scenario}")
    except ModuleNotFoundError:
        error = f"{opt.test_scenario}.py not found under ecloud/scenario_testing"

    if error is not None:
        try:
            testing_scenario = importlib.import_module(f"ecloud.scenario_testing.archived.{opt.test_scenario}")
        except ModuleNotFoundError:
            error = f"{opt.test_scenario}.py not found under ecloud/scenario_testing[/archived]"

    config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               f'ecloud/scenario_testing/config_yaml/{opt.test_scenario}.yaml')
    if not os.path.isfile(config_yaml):
        config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               f'ecloud/scenario_testing/config_yaml/archived/{opt.test_scenario}.yaml')
        if not os.path.isfile(config_yaml):
            error = f"ecloud/scenario_testing/config_yaml/[archived/]{opt.test_scenario}.yaml not found!"

    return testing_scenario, config_yaml, error

def main():
    '''
    fetches the specific scenario module (or default) and calls its 'run_scenario' method
    '''
    opt = arg_parse()
    assert ( opt.apply_ml is True and opt.distributed == 0 ) or opt.apply_ml is False
    logger.debug(opt)

    global FATAL_ERRORS
    FATAL_ERRORS = opt.fatal_errors

    print(f"eCloudSim Version: {__version__}")

    testing_scenario, config_yaml, error = get_scenario(opt)
    if error is not None:
        check_imports()
        sys.exit(error)

    if opt.build:
        subprocess.run(['python','-m','grpc_tools.protoc',
                        '-I./ecloud/protos','--python_out=.',
                        '--grpc_python_out=.','./ecloud//protos/ecloud.proto'],
                        check=True)

    EnvironmentConfig.set_environment(opt.environment)
    scenario_runner = getattr(testing_scenario, 'run_scenario')
    scenario_runner(opt, config_yaml)


if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        logger.info('exited by user.')
        sys.exit(0)

    except SystemExit:
        logger.info('scenario complete - exiting.')
        sys.exit(0)

    except Exception as e:
        logger.critical(e)
        traceback.print_exc(file=sys.stdout)
        if FATAL_ERRORS:
            raise
        sys.exit(1)
