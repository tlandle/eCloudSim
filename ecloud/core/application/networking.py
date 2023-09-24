# -*- coding: utf-8 -*-
"""
Extensible modular class for Network modeling. Calculates latency between individual Vehicle client and a networked control plane such as an Edge or Cloud

- track world time and increment it each time it ticks
- use world time and latency to evaluate SET_DESTINATION commands in queue
- Sim API *should* manage this and not send SET_DESTINATION to a specific vehicle until it's ready, but it's also fine to just broadcast them all and for individual vehicles to manage the queue
- it's possibly a bit easier if Sim API keeps queue since it can manage latencies centrally; otherwise, we need a way for vehicles to get latencies - which is fine? - but it does require support for a PING command
"""
# Author: Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import importlib

import numpy as np
import carla

from ecloud.core.common.misc import compute_distance

class NetworkModel:

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
