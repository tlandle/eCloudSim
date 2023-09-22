from ecloud.scenario_testing.utils.yaml_utils import load_yaml

__version__ = "0.0.3" # 3: CPP server
__ecloud__ = "ecloud"

# CONSTS
__local__ = "local"
__localhost__ = "localhost"
__carla_version__ = "0.9.12"
__default_scenario__ = "ecloud_4lane_scenario_dist_config"

# FILE PATHS
__environment_config__ = "environment_config.yaml" # in eCloud root folder
# local Carla path
# eCloud gRPC server path
# evaluation outputs path(s)

class EnvironmentConfig():
    '''
    static class containing accessor methods for the environment_config.yaml

    carla_server_public_ip
    ecloud_server_public_ip
    clients:
        client_machine:
            machine_name: ndm
            vehicle_client_public_ip: '20.172.248.156'
            vehicle_client_dns: ''
    '''

    # keys: server IPs 
    CARLA_IP = 'carla_server_public_ip'
    ECLOUD_IP = 'ecloud_server_public_ip'

    # keys: clients
    CLIENTS = 'clients'
    CLIENT_MACHINE = 'client_machine'
    MACHINE_NAME = 'machine_name' # self-identifier in ecloud_client to fetch environment vars
    CLIENT_IP = 'vehicle_client_public_ip'
    CLIENT_DNS = 'vehicle_client_dns'

    config = load_yaml(__environment_config__)
    environment = __local__

    @staticmethod
    def set_environment(environment: str) -> None:
        '''
        sets the working environment
        '''
        assert environment in EnvironmentConfig.config
        EnvironmentConfig.environment = environment

    @staticmethod
    def get_environment_params() -> dict:
        '''
        returns params for a given environment
        '''
        return EnvironmentConfig.config[EnvironmentConfig.environment]
    
    @staticmethod
    def get_carla_ip() -> str:
        '''
        gets the IP of the Carla server
        '''
        return EnvironmentConfig.config[EnvironmentConfig.environment][EnvironmentConfig.CARLA_IP]
    
    @staticmethod
    def get_ecloud_ip() -> str:
        '''
        gets the IP of gRPC eCloud server
        '''
        return EnvironmentConfig.config[EnvironmentConfig.environment][EnvironmentConfig.ECLOUD_IP]
    
    @staticmethod
    def get_client_ip_by_name(client_name) -> dict:
        '''
        gets the parameters for a given client name - e.g. 'ndm' - so that the client can access its IP config
        '''
        for client in EnvironmentConfig.config[EnvironmentConfig.environment][EnvironmentConfig.CLIENTS]:
            if client[EnvironmentConfig.CLIENT_MACHINE][EnvironmentConfig.MACHINE_NAME] == client_name:
                return client[EnvironmentConfig.CLIENT_MACHINE]
            
        assert False, f'invalid client name: {client_name}'

    @staticmethod
    def get_client_ip_list(client_name) -> dict:
        '''
        gets the parameters for a given client name - e.g. 'ndm' - so that the client can access its IP config
        '''
        for client in EnvironmentConfig.config[EnvironmentConfig.environment][EnvironmentConfig.CLIENTS]:
            if client[EnvironmentConfig.CLIENT_MACHINE][EnvironmentConfig.MACHINE_NAME] == client_name:
                return client[EnvironmentConfig.CLIENT_MACHINE]
            
        assert False, f'invalid client name: {client_name}'