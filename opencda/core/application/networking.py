# -*- coding: utf-8 -*-
"""
Extensible modular class for Network modeling. Calculates latency between individual Vehicle client and a networked control plane such as an Edge or Cloud
"""
# Author: Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import importlib

import numpy as np
import carla

from opencda.core.common.misc import compute_distance

class NetworkModel(object):

    def __init__(self) -> None:
        pass

    def set_latency_factor(self, latency_factor):
        '''
        should be something like time-per-meter
        '''
        pass

    def get_latency_between_nodes(self, node_1, node_2):
        '''
        the two nodes in this case can be two vehicles or a vehicle and an edge. maybe should be less generic...
        '''

        pass