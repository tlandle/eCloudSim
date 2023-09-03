# -*- coding: utf-8 -*-
"""
Analysis + visualization functions for platooning
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from opencda.core.plan.planer_debug_helper \
    import PlanDebugHelper


class SimDebugHelper(PlanDebugHelper):
    """This class aims to save statistics for platoon behaviour

    Parameters
    ----------
    actor_id : int
        The actor ID of the selected vehcile.

    Attributes
    ----------
    time_gap_list : list
        The list containing intra-time-gap(s) of all time-steps.

    dist_gap_list : list
        The list containing distance gap(s) of all time-steps.
    """

    def __init__(self, actor_id):
        super(SimDebugHelper, self).__init__(actor_id)

        self.world_tick_time_list = [[]]
        self.client_tick_time_list = [[]]
        self.sim_start_timestamp = None
        self.network_time_dict = {}
        self.client_tick_time_dict = {}       

    def update_world_tick(self, tick_time_step=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.world_tick_time_list[0].append(tick_time_step)
    

    def update_client_tick(self, tick_time_step=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.client_tick_time_list[0].append(tick_time_step)

    def update_individual_client_step_time(self, vehicle_index, tick_time_step=None):
        if vehicle_index not in self.client_tick_time_dict:
          self.client_tick_time_dict[vehicle_index] = []
        self.client_tick_time_dict[vehicle_index].append(tick_time_step)

    def update_sim_start_timestamp(self, timestamp=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.sim_start_timestamp = timestamp

    def update_network_time_timestamp(self, vehicle_index, network_time_step=None):
        if vehicle_index not in self.network_time_dict:
          self.network_time_dict[vehicle_index] = []
        self.network_time_dict[vehicle_index].append(network_time_step)
