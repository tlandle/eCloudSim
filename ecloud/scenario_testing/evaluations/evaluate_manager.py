# -*- coding: utf-8 -*-
"""
Evaluation manager.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
#       : Tyler Landle <tlandle3@gatech.edu>
#       : Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import pickle
import logging
import time

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import ecloud.core.plan.drive_profile_plotting as open_plt

from ecloud.scenario_testing.evaluations.utils import lprint
from ecloud.client_debug_helper import ClientDebugHelper
from ecloud.globals import __ecloud__

logger = logging.getLogger(__ecloud__)

class EvaluationManager:
    """
    Evaluation manager to manage the analysis of the
    results for different modules.

    Parameters
    ----------
    cav_world : opencda object
        The CavWorld object that contains all CAVs' information.

    script_name : str
        The current scenario testing name. E.g, single_town06_carla

    current_time : str
        Current timestamp, used to name the output folder.

    Attributes
    ----------
    eval_save_path : str
        The final output folder name.

    """

    def __init__(self, cav_world, script_name, current_time):
        self.cav_world = cav_world

        current_path = os.path.dirname(os.path.realpath(__file__))

        self.eval_save_path = os.path.join(
            current_path, '../../../evaluation_outputs',
            script_name + '_' + current_time)
        if not os.path.exists(self.eval_save_path):
            os.makedirs(self.eval_save_path)

        # base inits
        self.debug_helper = None
        self.scenario_manager = None

        #print(f"self.eval_save_path: {self.eval_save_path}")

    def evaluate(self):
        """
        Evaluate performance of all modules by plotting and writing the
        statistics into the log file.
        """
        log_file = os.path.join(self.eval_save_path, 'log.txt')

        self.localization_eval(log_file)
        print('Localization Evaluation Done.')

        self.kinematics_eval(log_file)
        print('Kinematics Evaluation Done.')

        self.platooning_eval(log_file)
        print('Platooning Evaluation Done.')

        self.edge_eval(log_file)
        print('Edge Evaluation Done.')

        self.simulation_eval(log_file)
        print('Simulation Evaluation Done.')

    def kinematics_eval(self, log_file):
        """
        vehicle kinematics related evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Kinematics Module***********")
        for _, v_m in self.cav_world.get_vehicle_managers().items():
            actor_id = v_m.vehicle.id
            lprint(log_file, f'Actor ID: {actor_id}')

            loc_debug_helper = v_m.agent.debug_helper
            figure, perform_txt = loc_debug_helper.evaluate()

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                f'{actor_id}_kinematics_plotting.png')
            figure.savefig(figure_save_path, dpi=100)

            lprint(log_file, perform_txt)

    def localization_eval(self, log_file):
        """
        Localization module evaluation.

        Args:
            -log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Localization Module***********")
        for _, v_m in self.cav_world.get_vehicle_managers().items():
            actor_id = v_m.vehicle.id
            #print("actor_id = v_m.vehicle.id")
            lprint(log_file, f'Actor ID: {actor_id}')

            loc_debug_helper = v_m.localizer.debug_helper
            #print("loc_debug_helper = v_m.localizer.debug_helper")
            figure, perform_txt = loc_debug_helper.evaluate()
            #print("figure, perform_txt = loc_debug_helper.evaluate()")

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                f'{actor_id}_localization_plotting.png')
            figure.savefig(figure_save_path, dpi=100)

            # save log txt
            lprint(log_file, perform_txt)

    def platooning_eval(self, log_file):
        """
        Platooning evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Platooning Analysis***********")

        for pm_id, p_m in self.cav_world.get_platoon_dict().items():
            lprint(log_file, f'Platoon ID: {pm_id}')
            figure, perform_txt = p_m.evaluate()

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                f'{pm_id}_platoon_plotting.png')
            figure.savefig(figure_save_path, dpi=100)

            # save log txt
            lprint(log_file, perform_txt)

    def edge_eval(self, log_file):
        """
        Edge evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Edge Analysis***********")

        for pm_id, p_m in self.cav_world.get_edge_dict().items():
            lprint(log_file, 'Edge ID: {pm_id}')
            figure, perform_txt = p_m.evaluate()

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                f'{pm_id}_edge_plotting.png')
            figure.savefig(figure_save_path, dpi=100)

            # save log txt
            lprint(log_file, perform_txt)


    def simulation_eval(self, log_file):
        """
        Simulation evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Simulation Analysis***********")

        self.scenario_manager = self.cav_world.get_scenario_manager()
        self.debug_helper = self.scenario_manager.debug_helper
        figure, perform_txt = self.evaluate_scenario()

        # save plotting
        figure_save_path = os.path.join(
            self.eval_save_path,
            'simulation_plotting.png')
        figure.savefig(figure_save_path, dpi=100)

        # save log txt
        lprint(log_file, perform_txt)

    def do_pickling(self, column_key, flat_list, file_path):
        '''
        pickle a dataframe for combined analysis in grpah generation
        '''
        logger.info("run stats for %s:\nmean %s: %s \nmedian %s: %s \n95th percentile %s %s",
                    column_key,
                    column_key,
                    np.mean(flat_list),
                    column_key,
                    np.median(flat_list),
                    column_key,
                    np.percentile(flat_list, 95))

        data_df = pd.DataFrame(flat_list, columns = [f'{column_key}_ms'])
        data_df['num_cars'] = self.scenario_manager.vehicle_count
        data_df['run_timestamp'] = pd.Timestamp.today().strftime('%Y-%m-%d %X')
        data_df = data_df[['num_cars', f'{column_key}_ms', 'run_timestamp']]

        data_df_path = f'./{file_path}/df_{column_key}'
        try:
            picklefile = open(data_df_path, 'rb+')
            current_data_df = pickle.load(picklefile)  #unpickle the dataframe
        except Exception as pickle_error:
            logger.warning('%s: failed to find pickle file - creating', type(pickle_error))
            picklefile = open(data_df_path, 'wb+')
            current_data_df = pd.DataFrame(columns=['num_cars', f'{column_key}_ms', 'run_timestamp'])

        picklefile = open(data_df_path, 'wb+')
        data_df = pd.concat([current_data_df, data_df], axis=0, ignore_index=True)

        pickle.dump(data_df, picklefile)
        logger.debug(data_df)
        picklefile.close()

    def evaluate_agent_data(self, cumulative_stats_folder_path):
        '''
        evaluate the data on the 12 individual agent steps
        '''
        PLANER_AGENT_STEPS = 12 # TODO: global
        all_agent_data_lists = [[] for _ in range(PLANER_AGENT_STEPS)]
        for _, v_m in self.scenario_manager.vehicle_managers.items():
            agent_data_list = v_m.agent.debug_helper.get_agent_step_list()
            for idx, sub_list in enumerate(agent_data_list):
                all_agent_data_lists[idx].append(sub_list)

        #logger.debug(all_agent_data_lists)

        for idx, all_agent_sub_list in enumerate(all_agent_data_lists):
            all_client_data_list_flat = np.array(all_agent_sub_list)
            if all_client_data_list_flat.any():
                all_client_data_list_flat = np.hstack(all_client_data_list_flat)
            else:
                all_client_data_list_flat = all_client_data_list_flat.flatten()
            data_key = f"agent_step_list_{idx}"
            self.do_pickling(data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_network_data(self, cumulative_stats_folder_path):
        '''
        evaluate networking overhead data
        '''
        all_network_data_list = sum(self.debug_helper.network_time_dict_per_client.values(), [])

        all_network_data_list_flat = np.array(all_network_data_list)
        if all_network_data_list_flat.any():
            all_network_data_list_flat = np.hstack(all_network_data_list_flat)
        else:
            all_network_data_list_flat = all_network_data_list_flat.flatten()

        data_key = "network_overhead"
        self.do_pickling(data_key, all_network_data_list_flat, cumulative_stats_folder_path)

    def evaluate_barrier_data(self, cumulative_stats_folder_path):
        '''
        evaluate time spent waiting at the barrier
        '''
        all_barrier_data_lists = sum(self.debug_helper.barrier_overhead_time_dict.values(), [])

        all_barrier_data_lists_flat = np.array(all_barrier_data_lists)
        if all_barrier_data_lists_flat.any():
            all_barrier_data_lists_flat = np.hstack(all_barrier_data_lists_flat)
        else:
            all_barrier_data_lists_flat = all_barrier_data_lists_flat.flatten()
        data_key = "barrier_overhead"
        self.do_pickling(data_key, all_barrier_data_lists_flat, cumulative_stats_folder_path)

    def evaluate_client_process_data(self, cumulative_stats_folder_path):
        '''
        evaluate how long each individual client spent actual running processing
        '''
        all_client_process_data_lists = sum(self.debug_helper.client_process_time_dict.values(), [])

        all_client_process_data_list_flat = np.array(all_client_process_data_lists)
        if all_client_process_data_list_flat.any():
            all_client_process_data_list_flat = np.hstack(all_client_process_data_list_flat)
        else:
            all_client_process_data_list_flat = all_client_process_data_list_flat.flatten()
        data_key = "client_process"
        self.do_pickling(data_key, all_client_process_data_list_flat, cumulative_stats_folder_path)

    def evaluate_individual_client_data(self, cumulative_stats_folder_path):
        '''
        evaluate individual client data

        TODO: redundant to process time above
        '''
        all_client_data_lists = sum(self.debug_helper.client_tick_time_dict_per_client.values(), [])

        all_client_data_list_flat = np.array(all_client_data_lists)
        if all_client_data_list_flat.any():
            all_client_data_list_flat = np.hstack(all_client_data_list_flat)
        else:
            all_client_data_list_flat = all_client_data_list_flat.flatten()

        data_key = "client_individual_step_time"
        self.do_pickling(data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_client_data(self, client_data_key, cumulative_stats_folder_path):
        '''
        helper function for client data by specific key
        '''
        all_client_data_list = []
        for _, v_m in self.scenario_manager.vehicle_managers.items():
            client_data_list = v_m.debug_helper.get_debug_data()[client_data_key]
            all_client_data_list.append(client_data_list)

        #logger.debug(all_client_data_list)

        all_client_data_list_flat = np.array(all_client_data_list)
        if all_client_data_list_flat.any():
            all_client_data_list_flat = np.hstack(all_client_data_list_flat)
        else:
            all_client_data_list_flat = all_client_data_list_flat.flatten()
        self.do_pickling(client_data_key, all_client_data_list_flat, cumulative_stats_folder_path)

    def evaluate_scenario(self, excludes_list=None):
        """
        Used to save all members' statistics.

        Returns
        -------
        figure : matplotlib.figure
            The figure drawing performance curve passed back to save to
            the disk.

        perform_txt : str
            The string that contains all evaluation results to print out.
        """

        perform_txt = ''

        node_count = 1
        if self.scenario_manager.run_distributed:
            node_count = self.scenario_manager.client_node_count
            cumulative_stats_folder_path = \
                f'./evaluation_outputs/cumulative_stats_dist_no_perception_{node_count}'
            if self.scenario_manager.perception:
                cumulative_stats_folder_path = \
                    f'./evaluation_outputs/cumulative_stats_dist_with_perception_{node_count}'
        else:
            cumulative_stats_folder_path = \
                    f'./evaluation_outputs/cumulative_stats_seq_no_perception_{node_count}'
            if self.scenario_manager.perception:
                cumulative_stats_folder_path = \
                    f'./evaluation_outputs/cumulative_stats_seq_with_perception_{node_count}'

        if not os.path.exists(cumulative_stats_folder_path):
            os.makedirs(cumulative_stats_folder_path)

        self.evaluate_agent_data(cumulative_stats_folder_path)
        if self.scenario_manager.run_distributed:
            self.evaluate_network_data(cumulative_stats_folder_path)
            self.evaluate_barrier_data(cumulative_stats_folder_path)
            self.evaluate_client_process_data(cumulative_stats_folder_path)
            self.evaluate_individual_client_data(cumulative_stats_folder_path)

        client_helper = ClientDebugHelper(0)
        debug_data_lists = client_helper.get_debug_data().keys()
        for list_name in debug_data_lists:
            if excludes_list is not None and list_name in excludes_list:
                continue
            self.evaluate_client_data(list_name, cumulative_stats_folder_path)

        # Client Step time
        client_tick_time_list = self.debug_helper.client_tick_time_list
        client_tick_time_list_flat = np.concatenate(client_tick_time_list)
        if client_tick_time_list_flat.any():
            client_tick_time_list_flat = np.hstack(client_tick_time_list_flat)
        else:
            client_tick_time_list_flat = client_tick_time_list_flat.flatten()
        client_step_time_key = 'client_step_time'
        self.do_pickling(client_step_time_key, client_tick_time_list_flat, cumulative_stats_folder_path)

        # World Step time
        world_tick_time_list = self.debug_helper.world_tick_time_list
        world_tick_time_list_flat = np.concatenate(world_tick_time_list)
        if world_tick_time_list_flat.any():
            world_tick_time_list_flat = np.hstack(world_tick_time_list_flat)
        else:
            world_tick_time_list_flat = world_tick_time_list_flat.flatten()
        world_step_time_key = 'world_step_time'
        self.do_pickling(world_step_time_key, world_tick_time_list_flat, cumulative_stats_folder_path)

        # Total simulation time
        sim_start_time = self.debug_helper.sim_start_timestamp
        sim_end_time = time.time()
        total_sim_time = sim_end_time - sim_start_time # total time in milliseconds
        perform_txt += f"Total Simulation Time: {total_sim_time}"
        perform_txt += f"\n\t Registration Time: {self.debug_helper.startup_time_ms}ms"
        perform_txt += f"\n\t Shutdown Time: {self.debug_helper.shutdown_time_ms}ms"

        sim_time_df_path = f'./{cumulative_stats_folder_path}/df_total_sim_time'
        try:
            picklefile = open(sim_time_df_path, 'rb+')
            sim_time_df = pickle.load(picklefile)
        except Exception as pickle_error:
            logger.warning('%s: failed to find pickle file - creating', type(pickle_error))
            picklefile = open(sim_time_df_path, 'wb+')
            sim_time_df = pd.DataFrame(columns=['num_cars', 'time_s', 'startup_time_ms', 'shutdown_time_ms', 'run_timestamp'])

        picklefile = open(sim_time_df_path, 'wb+')
        sim_time_df = pd.concat([sim_time_df, pd.DataFrame.from_records \
            ([{"num_cars": self.scenario_manager.vehicle_count, \
                "time_s": total_sim_time, \
                "startup_time_ms": self.debug_helper.startup_time_ms, \
                "shutdown_time_ms": self.debug_helper.shutdown_time_ms, \
                "run_timestamp": pd.Timestamp.today().strftime('%Y-%m-%d %X') }])], \
                ignore_index=True)

        pickle.dump(sim_time_df, picklefile)
        print(sim_time_df)
        picklefile.close()

        figure = plt.figure()
        plt.subplot(411)
        open_plt.draw_world_tick_time_profile_single_plot(world_tick_time_list)

        return figure, perform_txt
