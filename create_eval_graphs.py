import matplotlib.pyplot as plt
import seaborn as sns
import pickle
import pandas as pd
import numpy as np

cumulative_stats_folder_path = './evaluation_outputs/cumulative_stats'


def get_stats_df(file_path=""):
    # Read simulation stats file
    try:
        picklefile = open(file_path, 'rb+')
        stats_df = pickle.load(picklefile)  # unpickle the dataframe
        picklefile.close()
    except:
        print(f"cannot find file {file_path}")

    return stats_df

def create_box_plot(data, x, y, labels):
    ax = sns.boxplot(data=data, x = x, y = y)
    ax.set(xlabel=labels['xlabel'],
    ylabel=labels['ylabel'],
    title=labels['title'])
    ax.figure.savefig(f'./{cumulative_stats_folder_path}/total_sim_time_boxplot.png')
    plt.clf()

def create_scatter_plot(data, x, y, labels):
    ax = sns.scatterplot(data=data, x = x, y = y)
    ax.set(xlabel=labels['xlabel'],
    ylabel=labels['ylabel'],
    title=labels['title'])
    ax.figure.savefig(f'./{cumulative_stats_folder_path}/total_sim_time_scatterplot.png')
    plt.clf()

if __name__ == "__main__":

    # DF Paths
    sim_time_df_path = f'./{cumulative_stats_folder_path}/df_total_sim_time'
    sim_time_df_cumstats_path = f'./{cumulative_stats_folder_path}/df_total_sim_time_cumstats'

    # Plotting simulation total run time stats
    sim_stats_df = get_stats_df(sim_time_df_path)
    sim_cumstats_df = get_stats_df(sim_time_df_cumstats_path)

    print(sim_stats_df)
    print(sim_cumstats_df)

    labels = {"xlabel": 'Number of Cars', 
    'ylabel':'Total Runtime (s)',
    'title': 'Total Runtime per Number of Cars (4lane_edge sim)'}
    create_box_plot(data=sim_stats_df, x = 'num_cars', y = 'time_s',labels=labels)
    create_scatter_plot(data=sim_stats_df, x = 'num_cars', y = 'time_s',labels=labels)
