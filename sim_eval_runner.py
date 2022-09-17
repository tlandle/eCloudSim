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

from opencda.version import __version__
from opencda import opencda

def arg_parse():
    parser = argparse.ArgumentParser(description="OpenCDA scenario runneri eval runner.")
    parser.add_argument('-t', "--test_scenario_prefix", required=True, type=str,
                        help='Define the prefix of the scenario')
    parser.add_argument('-v', "--version", type=str, default='0.9.11',
                        help='Specify the CARLA simulator version, default'
                             'is 0.9.11, 0.9.12 is also supported.')

    opt = parser.parse_args()
    return opt


def main():
    opt = arg_parse()
    subprocess.call([])

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
