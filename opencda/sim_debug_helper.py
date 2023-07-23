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
        

    def update_world_tick(self, tick_time_step=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.world_tick_time_list[0].append(tick_time_step)
    

    def update_world_tick(self, tick_time_step=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.client_tick_time_list[0].append(tick_time_step)


    def update_sim_start_timestamp(self, timestamp=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.sim_start_timestamp = timestamp
