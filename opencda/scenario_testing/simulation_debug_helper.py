# -*- coding: utf-8 -*-
"""
Analysis + Visualization functions for planning
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License:  TDG-Attribution-NonCommercial-NoDistrib
import warnings

import numpy as np
import matplotlib.pyplot as plt

import simulation_plotting as open_plt


class SimDebugHelper(object):
    """
    This class aims to save statistics for simulator 
    """

    def __init__(self):
        self.step_runtime_list = [[]]
        self.count = 0
        perform_txt = ''

    def update(self, step_time):
        """
        Update the step time.
        Args:
            -ego_speed (float): Ego speed in km/h.
            -ttc (flot): Time to collision in seconds.

        """
        self.count += 1
        self.step_runtime_list[0].append(step_time)

    def evaluate(self):
        """
        Evaluate the target vehicle and visulize the plot.
        Returns:
            -figure (matplotlib.pyplot.figure): The target vehicle's planning
             profile (velocity, acceleration, and ttc).
            -perform_txt (txt file): The target vehicle's planning profile
            as text files.

        """
        warnings.filterwarnings('ignore')
        # draw speed, acc and ttc plotting
        figure = plt.figure()
        open_plt.draw_step_time_plot(self.speed_list)
        open_plt.save_step_time_info(self.speed_list)

        figure.suptitle('Simulation Time')

        # calculate the statistics
        step_avg = np.mean(np.array(self.step_runtime_list))
        step_std = np.std(np.array(self.step_runtime_list))

        perform_txt = 'Step average: %f (ms), ' \
                      'Step std: %f (ms) \n' % (step_avg, step_std)

        perform_txt += 'Acceleration average: %f (m/s), ' \
                       'Acceleration std: %f (m/s) \n' % (acc_avg, acc_std)

        perform_txt += 'TTC average: %f (m/s), ' \
                       'TTC std: %f (m/s) \n' % (ttc_avg, ttc_std)

        return figure, perform_txt
