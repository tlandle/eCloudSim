# -*- coding: utf-8 -*-
"""
Analysis + visualization functions for platooning
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from opencda.core.plan.planer_debug_helper \
    import PlanDebugHelper


class ClientDebugHelper(PlanDebugHelper):
    """This class aims to save statistics for client time

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
        super(ClientDebugHelper, self).__init__(actor_id)

        self.perception_time_list = []
        self.localization_time_list = []
        self.update_info_time_list = []
        self.agent_update_info_time_list = []
        self.controller_update_info_time_list = []
        self.agent_step_time_list = []
        self.controller_step_time_list = []
        self.vehicle_step_time_list = []

        

    def update_perception_time(self, tick_time_step=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.perception_time_list.append(tick_time_step)
    

    def update_localization_time(self, tick_time_step=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.localization_time_list.append(tick_time_step)


    def update_update_info_time(self, time=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.update_info_time_list.append(time)

    def update_agent_update_info_time(self, time=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.agent_update_info_time_list.append(time)


    def update_controller_update_info_time(self, time=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.controller_update_info_time_list.append(time)

    def update_agent_step_time(self, time=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.agent_step_time_list.append(time)

    def update_vehicle_step_time(self, time=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.vehicle_step_time_list.append(time)

    def update_controller_step_time(self, time=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.controller_step_time_list.append(time)


    def serialize_debug_info(self, proto_debug_helper):
        # TODO: extend instead of append? or [:] = ?

        for obj in self.perception_time_list:
            proto_debug_helper.perception_time_list.append(obj)

        for obj in self.localization__time_list:
            proto_debug_helper.localization_time_list.append(obj)

        for obj in self.update_info_time_list:
            proto_debug_helper.update_info_time_list.append(obj)
        
        for obj in self.agent_update_info_time_list:
            proto_debug_helper.agent_update_info_time_list.append(obj)

        for obj in self.controller_update_info_time_list:
            proto_debug_helper.controller_update_info_time_list.append(obj)
        
        for obj in self.agent_step_time_list:
            proto_debug_helper.agent_step_time_list.append(obj)
        
        for obj in self.vehicle_step_time_list:
            proto_debug_helper.vehicle_step_time_list.append(obj)
        
        for obj in self.controller_step_time_list:
            proto_debug_helper.controller_step_time_list.append(obj)


    def deserialize_debug_info(self, proto_debug_helper):
        # call from Sim API to populate locally

        self.perception_time_list.clear()
        for obj in proto_debug_helper.perception_time_list:
            self.perception_time_list.append(obj)

        self.localization_time_list.clear()
        for obj in proto_debug_helper.localization_time_list:
            self.localization_time_list.append(obj)

        self.update_info_time_list.clear()
        for obj in proto_debug_helper.update_info_time_list:
            self.update_info_time_list.append(obj)
 
        self.agent_update_info_time_list.clear()
        for obj in proto_debug_helper.agent_update_info_time_list:
            self.agent_update_info_time_list.append(obj)
 
        self.controller_update_info_time_list.clear()
        for obj in proto_debug_helper.controller_update_info_time_list:
            self.controller_update_info_time_list.append(obj)
 
        self.agent_step_time_list.clear()
        for obj in proto_debug_helper.agent_step_time_list:
            self.agent_step_time_list.append(obj)
 
        self.vehicle_step_time_list.clear()
        for obj in proto_debug_helper.vehicle_step_time_list:
            self.vehicle_step_time_list.append(obj)
 
        self.controller_step_time_list.clear()
        for obj in proto_debug_helper.controller_step_time_list:
            self.controller_step_time_list.append(obj)
  
