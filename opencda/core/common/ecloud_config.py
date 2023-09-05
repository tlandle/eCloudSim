from enum import Enum

class eLocationType(Enum):
    RANDOM = 0
    EXPLICT = 1
    COUNT = 2

class eDoneBehavior(Enum):
    DESTROY = 0
    CONTROL = 1
    COUNT = 2 

class EcloudConfig(object):    
    def __init__(self):
        
        self.ecloud_base = {
            "perception_active" : False,
            "distributed" : True, # set to false or comment out to run "standard" OpenCDA sequential sims
            "num_servers": 2, # % num_cars to choose which port to connect to. 2nd - nth server port : p = 50053 + ( n - 1 )
            "server_ping_time_s" : 0.005, # 5ms
            "client_ping_base_s" : 0.01, # multiplied by num vehicles
            "client_ping_multiply_by_num_cars" : True, # 0.01 * num_cars = initial sleep before ping in s
            "client_ping_spawn_max_s" : 0.5, # how long at most to wait before pinging after spawn
            "client_ping_spawn_min_s" : 0.01, # minimum sleep to wait between pings after spawn
            "client_ping_spawn_backoff" : 0.5, # how much to drop sleep each successive ping
            "client_ping_tick_max_s" : 0.1, # how long at most to wait before pinging after tick complete
            "client_ping_tick_min_s" : 0.01, # minimum sleep to wait between pings after spawn
            "client_ping_tick_backoff" : 0.5, # how much to drop sleep each successive ping
        }

        '''
        scenario:
          ecloud: 
            num_cars: 128
            spawn_type: random # random || explicit
            destination_type: explicit # random || explicit
            done_behavior: destroy # destroy || control
        '''

        self.RANDOM = "random"
        self.EXPLICIT = "explicit"
        self.DESTROY = "destroy"
        self.CONTROL = "control"

        self.location_types = { self.RANDOM : eLocationType.RANDOM, 
                                self.EXPLICIT : eLocationType.EXPLICT }
        self.done_behavior_types = { self.DESTROY : eDoneBehavior.DESTROY,
                                     self.CONTROL : eDoneBehavior.CONTROL }

        self.ecloud_scenario = {
           "num_cars" : 0,
           "spawn_type" : self.RANDOM, 
           "destination_type" : self.EXPLICIT,
           "done_behavior" : self.DESTROY,
        }

        '''
        # make getter? 
        ecloud_config = EcloudConfig()
        for k, _ in ecloud_config.ecloud_scenario.items():
            if k in YAML:
                ecloud_config.ecloud_scenario[k] = YAML[k]
        '''