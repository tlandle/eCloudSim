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

from ecloud.globals import __version__, __ecloud__, __default_scenario__

logger = logging.getLogger(__ecloud__)

import_module = re.compile(r'import ([\.A-Za-z0-9_-]+) ')
import_class = re.compile(r'from ([\.A-Za-z0-9_-]+) import')

def arg_parse():
    '''
    fetch the command line args & returns an args object
    '''
    parser = argparse.ArgumentParser(description="eCloudSim scenario runner.")
    parser.add_argument('-t', "--test_scenario", type=str, default=__default_scenario__,
                        help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'ecloud/scenario_testing/ folder'
                             f' as well as the corresponding yaml file in ecloud/scenario_testing/config_yaml. [Default: {__default_scenario__}]')
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
                            help="Run Carla with optional args; use = --run_carla='-RenderOffscreen'")
    parser.add_argument("--version", type=str, default="0.9.12",
                        "Carla version. [default: 0.9.12]") # only support version
    opt = parser.parse_args()
    return opt

def check_imports():
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
                                except Exception as e:
                                    if module not in missing_imports.keys() and f"{e}" not in missing_imports.values():
                                        missing_imports[module] = f"{e}"
                                        logger.error(f"failed importing {module} - {e}")
                                    continue
                                else:
                                    logger.debug(f"module {x.group(1)} imported OK")
                            
                            x = re.search(import_class, l)
                            if x:
                                try:
                                    module = x.group(1)
                                    importlib.import_module(module)
                                except Exception as e:
                                    if module not in missing_imports.keys() and f"{e}" not in missing_imports.values():
                                        missing_imports[module] = f"{e}"
                                        logger.error(f"failed importing {module} - {e}")
                                    continue
                                else:
                                    logger.debug(f"module {module} imported OK")

def get_scenario(opt):
    testing_scenario = None
    config_yaml = None
    error = None
    try:
        testing_scenario = importlib.import_module("ecloud.scenario_testing.%s" % opt.test_scenario)
    except ModuleNotFoundError:
        error = format("%s.py not found under ecloud/scenario_testing" % opt.test_scenario)

    if error is not None:
        try:
            testing_scenario = importlib.import_module("ecloud.scenario_testing.archived.%s" % opt.test_scenario)
        except ModuleNotFoundError:
            error = format("%s.py not found under ecloud/scenario_testing[/archived]" % opt.test_scenario)

    config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'ecloud/scenario_testing/config_yaml/%s.yaml' % opt.test_scenario)
    if not os.path.isfile(config_yaml):
        config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'ecloud/scenario_testing/config_yaml/archived/%s.yaml' % opt.test_scenario)
        if not os.path.isfile(config_yaml):
            error = format("ecloud/scenario_testing/config_yaml/[archived/]%s.yaml not found!" % opt.test_scenario)

    return testing_scenario, config_yaml, error

def main():
    opt = arg_parse()
    logger.debug(opt)
    print(f"eCloudSim Version: {__version__}")

    testing_scenario, config_yaml, error = get_scenario(opt)
    if error is not None:
        check_imports()
        sys.exit(error)

    if opt.build:
        subprocess.run(['python','-m','grpc_tools.protoc','-I./ecloud/protos','--python_out=.','--grpc_python_out=.','./ecloud//protos/ecloud.proto'])

    scenario_runner = getattr(testing_scenario, 'run_scenario')
    scenario_runner(opt, config_yaml)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        if type(e) == KeyboardInterrupt:
            logger.info('exited by user.')
        elif type(e) == SystemExit:
            logger.info(f'system exit: {e}')
        else:
            logger.critical(e)
            raise