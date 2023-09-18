from enum import Enum
import logging
import sys

import coloredlogs

class eLocationType(Enum):
    RANDOM = 0
    EXPLICIT = 1
    COUNT = 2

class eDoneBehavior(Enum):
    DESTROY = 0
    CONTROL = 1
    DRIVE = 2
    COUNT = 3 

class EcloudConfig(object):  

    RANDOM = "random"
    EXPLICIT = "explicit"
    DESTROY = "destroy"
    CONTROL = "control"
    DRIVE = "drive"

    FORMAT_STRING = "%(asctime)s %(filename)s %(funcName)s %(lineno)d %(message)s"

    log_level = logging.INFO
    logger = None
    
    location_types = { RANDOM : eLocationType.RANDOM, 
                       EXPLICIT : eLocationType.EXPLICIT }
    
    done_behavior_types = { DESTROY : eDoneBehavior.DESTROY,
                            CONTROL : eDoneBehavior.CONTROL,
                            DRIVE   : eDoneBehavior.DRIVE }


    # TODO: move actual inits to @staticmethod
    def __init__(self, config_json):

        # logger.debug(f"main - test_scenario: {config_json}") # VERY verbose
        EcloudConfig.init_logging()
        
        self.config_json = config_json
        self.ecloud_base = {}

        self.ecloud_scenario = {
           "num_cars" : 0,
           "location_type" : self.EXPLICIT,
           "done_behavior" : self.CONTROL,
           "step_count" : 250, # number of steps to take before breaking
           "log_level" : EcloudConfig.log_level,
        }

        if 'ecloud' in config_json:
            self.logger.debug("'ecloud' found in config_base")
            for k in self.ecloud_base.keys():
                if k in config_json['ecloud']:
                    self.logger.info(f"overriding base_config {k} with {config_json['ecloud'][k]}")
                    self.ecloud_base[k] = config_json['ecloud'][k]
        else:
            self.logger.warning("'ecloud' not found in config_base")

        if 'ecloud' in config_json['scenario']:
            self.logger.debug("'ecloud' found in config_scenario")
            for k in self.ecloud_scenario.keys():
                if k in config_json['scenario']['ecloud']:
                    self.logger.info(f"overriding scenario_config {k} with {config_json['scenario']['ecloud'][k]}")
                    self.ecloud_scenario[k] = config_json['scenario']['ecloud'][k] 
        else:
            self.logger.warning("'ecloud' not found in config_scenario")

        EcloudConfig.set_log_level(self.ecloud_scenario['log_level'])            
   
    def get_num_cars(self):
        self.logger.debug(f"num_cars: {self.ecloud_scenario['num_cars'] if self.ecloud_scenario['num_cars'] != 0 else len(self.config_json['scenario']['single_cav_list'])}")
        return self.ecloud_scenario['num_cars'] if self.ecloud_scenario['num_cars'] != 0 else \
                len(self.config_json['scenario']['single_cav_list'])
    
    def get_location_type(self):
        self.logger.debug(f"location_type: {self.ecloud_scenario['location_type']}")
        return EcloudConfig.location_types[self.ecloud_scenario['location_type']]
    
    def get_done_behavior(self):
        self.logger.debug(f"done_behavior: {self.ecloud_scenario['done_behavior']}")
        return EcloudConfig.done_behavior_types[self.ecloud_scenario['done_behavior']]
    
    def get_step_count(self):
        self.logger.debug(f"step_count: {self.ecloud_scenario['step_count']}")
        return self.ecloud_scenario['step_count']
    
    @staticmethod
    def get_log_level():
        return EcloudConfig.log_level
        
    @staticmethod    
    def set_log_level(log_level: int, debug=False):
        # TODO: make these global consts
        if log_level == 3:
            EcloudConfig.log_level = logging.ERROR
        elif log_level == 2:
            EcloudConfig.log_level = logging.WARNING
        elif log_level == 1:
            EcloudConfig.log_level = logging.INFO
        elif log_level == 0:
            EcloudConfig.log_level = logging.DEBUG
        else:
            EcloudConfig.log_level = log_level

        EcloudConfig.logger = logging.getLogger("ecloud")
        coloredlogs.install(level=EcloudConfig.log_level, 
                            logger=EcloudConfig.logger,
                            miliseconds=True,
                            fmt=EcloudConfig.FORMAT_STRING,
                            datefmt='%H:%M:%S',
                            field_styles={'asctime': {'color': 'green'}, 
                                          'filename': {'bold': True, 'color': 'blue'}, 
                                          'funcName': {'color': 'cyan'},
                                          'lineno': {'color' : 'cyan'},
                                          'level': coloredlogs.DEFAULT_LEVEL_STYLES},
                            level_styles=coloredlogs.DEFAULT_LEVEL_STYLES,
                            reconfigure=True,
                            )
        if debug:
            EcloudConfig.logger.error("ERROR")
            EcloudConfig.logger.warning("WARNING")
            EcloudConfig.logger.info("INFO")
            EcloudConfig.logger.debug("DEBUG")

    @staticmethod
    def get_logger():
        return EcloudConfig.logger
    
    @staticmethod
    def init_logging():
        EcloudConfig.logger = logging.getLogger("ecloud")
        coloredlogs.install(level=EcloudConfig.log_level, 
                            logger=EcloudConfig.logger,
                            miliseconds=True,
                            fmt=EcloudConfig.FORMAT_STRING,
                            datefmt='%H:%M:%S',
                            field_styles={'asctime': {'color': 'green'}, 
                                          'filename': {'bold': True, 'color': 'blue'}, 
                                          'funcName': {'color': 'cyan'}},
                            level_styles=coloredlogs.DEFAULT_LEVEL_STYLES,
                            reconfigure=True,
                            )