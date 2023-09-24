# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
#       : Aaron Drysdale <adrysdale3@gatech.edu>
#       : Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import uuid

import carla

from ecloud.core.actuation.control_manager \
    import ControlManager
from ecloud.core.application.platooning.platoon_behavior_agent\
    import PlatooningBehaviorAgent
from ecloud.core.common.v2x_manager \
    import V2XManager
from ecloud.core.sensing.localization.localization_manager \
    import LocalizationManager
from ecloud.core.sensing.perception.perception_manager \
    import PerceptionManager
from ecloud.core.plan.behavior_agent \
    import BehaviorAgent
from ecloud.core.common.data_dumper import DataDumper
from ecloud.client_debug_helper import ClientDebugHelper
from ecloud.core.common.ecloud_config import eLocationType

RESULT_SUCCESS = 0 # Step ran ok
RESULT_ERROR = 1 # Step resulted in an error
RESULT_END = 2 # Step resulted in the vehicle simulation ending

CARLA_IP = 'localhost' # TODO Environment Config

# once we have all methods, we no longer need any reference to the actual actor in ecloud
class ActorProxy(object):
    '''
    lightweight proxy class used as a replace for an actual Carla actor in distributed scenarios
    '''
    def __init__(self):
        self.transform = carla.Transform(
                carla.Location(
                    x=0,
                    y=0,
                    z=0),
                carla.Rotation(
                    yaw=0,
                    roll=0,
                    pitch=0))
        self.velocity = carla.Vector3D(x=0, y=0, z=0)
        self.id = uuid.uuid1()

    def is_proxy(self):
        '''
        used by getattr to indicate that this is a proxy, not called directly
        '''
        return True

    def get_transform(self):
        '''
        override of equivalent Carla actor method
        '''
        return self.transform

    def set_transform(self, transform):
        '''
        override of equivalent Carla actor method
        '''
        self.transform = transform

    def get_location(self):
        '''
        override of equivalent Carla actor method
        '''
        # need to return Carla.location
        return self.transform.location

    def set_location(self, location):
        '''
        override of equivalent Carla actor method
        '''
        self.transform.location = location

    def get_velocity(self):
        '''
        override of equivalent Carla actor method
        '''
        # need to return carla.Vector3D
        return self.velocity

    def set_velocity(self, velocity):
        '''
        override of equivalent Carla actor method
        '''
        self.velocity = velocity

class VehicleManagerProxy(object):
    """
    TODO: update
    """

    def __init__(self,
                vehicle_index,
                #conn,
                config_yaml,
                application,
                carla_map,
                cav_world,
                current_time='',
                data_dumping=False,
                location_type=eLocationType.EXPLICIT):

        # default initializers
        self.agent = None
        self.controller = None
        self.data_dumper = None
        self.perception_manager = None
        self.vehicle = None
        self.v2x_manager = None
        self.localizer = None

        self.is_edge = False
        if 'single_cav_list' in config_yaml['scenario']:
            self.cav_config = config_yaml['scenario']['single_cav_list'][vehicle_index] \
                                if location_type == eLocationType.EXPLICIT \
                                else config_yaml['scenario']['single_cav_list'][0]

        elif 'edge_list' in config_yaml['scenario']:
            # TODO: support multiple edges...
            self.is_edge = True
            self.cav_config = config_yaml['scenario']['edge_list'][0]['members'][vehicle_index]

        else:
            assert False, "no known vehicle indexing format found"

        self.cav_world = cav_world
        self.data_dumping = data_dumping
        self.application = application
        self.current_time = current_time
        self.vehicle_index = vehicle_index
        self.vid = vehicle_index # needed for compatibility

        self.carla_map = carla_map

        # Use sockets for interprocess communication between OpenCDA and each vehicle
        #self._socket = conn
        self.debug_helper = ClientDebugHelper(0)

    def start_vehicle(self):
        '''
        override of equivalent VehicleManager method
        '''
        # print("eCloud debug | actor_id: " + str(actor_id))
        self.vehicle = ActorProxy()

        # retrieve the configure for different modules
        sensing_config = self.cav_config['sensing']
        behavior_config = self.cav_config['behavior']
        control_config = self.cav_config['controller']
        v2x_config = self.cav_config['v2x']
        # v2x module
        self.v2x_manager = V2XManager(self.cav_world, v2x_config, self.vehicle_index)
        # localization module
        self.localizer = LocalizationManager(
            self.vehicle, sensing_config['localization'], self.carla_map)
        # perception module - proxy should never use perception
        sensing_config['perception']['activate'] = False
        sensing_config['perception']['camera_visualize'] = False
        sensing_config['perception']['lidar_visualize'] = False
        self.perception_manager = PerceptionManager(
            self.vehicle, sensing_config['perception'], self.cav_world,
            self.data_dumping)

        # behavior agent
        self.agent = None
        if 'platooning' in self.application:
            platoon_config = self.cav_config['platoon']
            self.agent = PlatooningBehaviorAgent(
                self.vehicle, # TODO: fix
                self,
                self.v2x_manager,
                behavior_config,
                platoon_config,
                self.carla_map)
        else:
            self.agent = BehaviorAgent(self.vehicle, self.carla_map, behavior_config)

        # Control module
        self.controller = ControlManager(control_config)

        if self.data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          self.vehicle.id,
                                          save_time=self.current_time)
        else:
            self.data_dumper = None

        self.cav_world.update_vehicle_manager(self)
