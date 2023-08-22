#!/usr/bin/env python
# coding: utf-8

# In[1]:


import os
import matplotlib.pyplot as plt
import seaborn as sns
import pickle
import pandas as pd


# In[2]:


# Constants
GRAPH_PERCEPTION = False
GRAPH_SINGLE_NODE = False
PERCEPTION_TITLE = "with Perception" if GRAPH_PERCEPTION else "without Perception"
NODE_TITLE = "single node" if GRAPH_SINGLE_NODE else "multi node"
CUMULATIVE_STATS_FOLDER_PATH = './evaluation_outputs/cumulative_stats_with_perception' if GRAPH_PERCEPTION else './evaluation_outputs/cumulative_stats_no_perception'

SHOULD_SHOW = False

# In[3]:


def get_stats_df(file_path=""):
    """
    Load a DataFrame from a pickle file.

    Args:
    file_path (str): The path of the pickle file.

    Returns:
    pd.DataFrame: The DataFrame loaded from the pickle file.
    """
    if not os.path.exists(file_path):
        print(f"Cannot find file {file_path}")
        return None

    try:
        with open(file_path, 'rb') as picklefile:
            stats_df = pickle.load(picklefile)
        return stats_df
    except Exception as e:
        print(f"Error loading file {file_path}: {e}")


# In[4]:


def create_box_plot(data, x, y, labels):
    """
    Create a box plot using seaborn.

    Args:
    data (pd.DataFrame): The DataFrame containing the data to be plotted.
    x (str): The column name for the x-axis variable.
    y (str): The column name for the y-axis variable.
    labels (dict): A dictionary containing the labels for the plot (xlabel, ylabel, title).

    Returns:
    Axes: The axis object containing the box plot.
    """
    ax = sns.boxplot(data=data, x=x, y=y)
    ax.set(xlabel=labels['xlabel'],
           ylabel=labels['ylabel'],
           title=labels['title'])
    return ax


# In[5]:
def create_bar_plot(data, x, y, labels):
    """
    Create a bar plot using seaborn.

    Args:
    data (pd.DataFrame): The DataFrame containing the data to be plotted.
    x (str): The column name for the x-axis variable.
    y (str): The column name for the y-axis variable.
    labels (dict): A dictionary containing the labels for the plot (xlabel, ylabel, title).

    Returns:
    Axes: The axis object containing the box plot.
    """
    ax = sns.barplot(data=data, x=x, y=y)
    ax.set(xlabel=labels['xlabel'],
           ylabel=labels['ylabel'],
           title=labels['title'])
    return ax


def create_stacked_bar_chart(data,  y, labels):
    """
    Create a scatter plot using seaborn.

    Args:
    data (pd.DataFrame): The DataFrame containing the data to be plotted.
    x (str): The column name for the x-axis variable.
    y (str): The column name for the y-axis variable.
    labels (dict): A dictionary containing the labels for the plot (xlabel, ylabel, title).

    Returns:
    Axes: The axis object containing the scatter plot.
    """

    ax = data.plot(y=y, kind = 'bar', stacked=True, color=['red', 'blue', 'green', 'purple', 'yellow', 'black'])
    plt.xlabel(labels['xlabel'])
    plt.ylabel(labels['ylabel'])
    plt.title(labels['title'])
    return ax


# In[6]:


def save_ax(ax=None, file_path=""):
    """
    Save the plot to a file.

    Args:
    ax (Axes): The axis object containing the plot.
    file_path (str): The path where the plot should be saved.
    """
    if not file_path or not ax:
        print("File path or axis object not provided.")
        return

    ax.figure.savefig(file_path)
    print(f"Saved file: {file_path}")


# In[273]:


def plot_simulation_time():
    sim_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_total_sim_time'
    sim_stats_df = get_stats_df(sim_time_df_path)

    labels = {"xlabel": 'Number of Cars',
              "ylabel": 'Total Runtime (s)',
              "title": f'eCloudSim: Total Runtime \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}

    # sns.set(font_scale=1.2)
    sns.set_style('whitegrid')  

    # Box plot
    plt.figure(figsize=(10, 6))
    ax = create_box_plot(data=sim_stats_df, x='num_cars', y='time_s', labels=labels)
    save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/total_sim_time_boxplot.png'
    save_ax(ax, save_file_path)
    if SHOULD_SHOW:
        plt.show()
    plt.clf()

    # Scatter plot
    # ax = create_scatter_plot(data=sim_stats_df, x='num_cars', y='time_s', labels=labels)
    # save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/total_sim_time_scatterplot.png'
    # save_ax(ax, save_file_path)
    # if SHOULD_SHOW:
    #    plt.show()
    # plt.clf()


# In[274]:


sim_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_total_sim_time'
# sim_stats_df = get_stats_df(f'{CUMULATIVE_STATS_FOLDER_PATH}/df_total_sim_time_cumstats')
sim_stats_df = get_stats_df(sim_time_df_path)
sim_stats_df


# In[275]:


def plot_world_step_time():
    step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_world_step_time'
    sim_stats_df = get_stats_df(step_time_df_path)

    labels = {"xlabel": 'Number of Cars',
              "ylabel": 'Simulation Step Time (ms)',
              "title": f'eCloudSim: Simulation Step Time \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}

    # Box plot
    plt.figure(figsize=(10, 6))
    ax = create_box_plot(data=sim_stats_df, x='num_cars', y='world_step_time_ms', labels=labels)
    save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/world_step_time_boxplot.png'
    save_ax(ax, save_file_path)
    if SHOULD_SHOW:
        plt.show()
    plt.clf()

    # Scatter plot
    # ax = create_scatter_plot(data=sim_stats_df, x='num_cars', y='step_time_ms', labels=labels)
    # save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/step_time_scatterplot.png'
    # save_ax(ax, save_file_path)
    # if SHOULD_SHOW:
    #    plt.show()
    # plt.clf()

def plot_client_step_time():
    step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_client_step_time'
    sim_stats_df = get_stats_df(step_time_df_path)

    labels = {"xlabel": 'Number of Cars',
              "ylabel": 'Simulation Step Time (ms)',
              "title": f'eCloudSim: Simulation Step Time \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}

    # Box plot
    plt.figure(figsize=(10, 6))
    ax = create_box_plot(data=sim_stats_df, x='num_cars', y='client_step_time_ms', labels=labels)
    save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/client_step_time_boxplot.png'
    save_ax(ax, save_file_path)
    if SHOULD_SHOW:
        plt.show()
    plt.clf()

    # Scatter plot
    # ax = create_scatter_plot(data=sim_stats_df, x='num_cars', y='step_time_ms', labels=labels)
    # save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/step_time_scatterplot.png'
    # save_ax(ax, save_file_path)
    # if SHOULD_SHOW:
    #    plt.show()
    # plt.clf()

def plot_client_perception_time():
    step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_client_perception_time'
    sim_stats_df = get_stats_df(step_time_df_path)

    labels = {"xlabel": 'Number of Cars',
              "ylabel": 'Client Perception Time (ms)',
              "title": f'eCloudSim: Client Perception Time \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}

    # Box plot
    plt.figure(figsize=(10, 6))
    ax = create_box_plot(data=sim_stats_df, x='num_cars', y='client_perception_time_ms', labels=labels)
    save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/client_perception_time_boxplot.png'
    save_ax(ax, save_file_path)
    if SHOULD_SHOW:
        plt.show()
    plt.clf()

def plot_client_localization_time():
    step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_client_localization_time'
    sim_stats_df = get_stats_df(step_time_df_path)

    labels = {"xlabel": 'Number of Cars',
              "ylabel": 'Client Localization Time (ms)',
              "title": f'eCloudSim: Client Localization Time \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}

    # Box plot
    plt.figure(figsize=(10, 6))
    ax = create_box_plot(data=sim_stats_df, x='num_cars', y='client_localization_time_ms', labels=labels)
    save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/client_localization_time_boxplot.png'
    save_ax(ax, save_file_path)
    if SHOULD_SHOW:
        plt.show()
    plt.clf()

def plot_client_control_time():
    step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_client_control_time'
    sim_stats_df = get_stats_df(step_time_df_path)

    labels = {"xlabel": 'Number of Cars',
              "ylabel": 'Client Control Time (ms)',
              "title": f'eCloudSim: Client Control Time \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}

    # Box plot
    plt.figure(figsize=(10, 6))
    ax = create_box_plot(data=sim_stats_df, x='num_cars', y='client_control_time_ms', labels=labels)
    save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/client_control_time_boxplot.png'
    save_ax(ax, save_file_path)
    if SHOULD_SHOW:
        plt.show()
    plt.clf()

def plot_agent_step_times():
    TOTAL_AGENT_STEPS = 12
    AGENT_STEPS = {
            0: "sim end",
            1: "lights",
            2: "temp route",
            3: "path generation",
            4: "lane change",
            5: "collision",
            6: "no lane change composite",
            7: "push",
            8: "blocking",
            9: "overtake",
            10: "following",
            11: "normal",
    }

    for i in range(TOTAL_AGENT_STEPS):
        step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_agent_step_list_{i}'
        sim_stats_df = get_stats_df(step_time_df_path)

        labels = {"xlabel": 'Number of Cars',
              "ylabel": f'Agent Step {i} Time (ms)',
              "title": f'eCloudSim: Agent Step {i} Time - {AGENT_STEPS[i].title()} \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}

        # Box plot
        plt.figure(figsize=(10, 6))
        ax = create_box_plot(data=sim_stats_df, x='num_cars', y=f'agent_step_list_{i}_ms', labels=labels)
        save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/agent_step_{i}_time_boxplot.png'
        save_ax(ax, save_file_path)
        if SHOULD_SHOW:
            plt.show()
        plt.clf()


def plot_client_stacked_barchart():
    TOTAL_AGENT_STEPS = 7
    AGENT_STEPS = {
            0: "sim end",
            1: "lights",
            2: "temp route",
            3: "path generation",
            4: "lane change",
            5: "collision",
            6: "no lane change composite",
            7: "push",
            8: "blocking",
            9: "overtake",
            10: "following",
            11: "normal",
    }

    step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_agent_step_list_0'
    agent_df = get_stats_df(step_time_df_path)
    y_columns = ['agent_step_list_0_ms']

    labels = {"xlabel": 'Number of Cars',
              "ylabel": f'Agent Step Time',
              "title": f'eCloudSim: Agent Step Time - Composite \n per Number of Vehicles ({PERCEPTION_TITLE}) - {NODE_TITLE}'}


    for i in range(1, TOTAL_AGENT_STEPS):
      step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_agent_step_list_{i}'
      sim_stats_df = get_stats_df(step_time_df_path)
      #print(sim_stats_df[f'agent_step_list_{i}_ms'])
      y_columns.append(f'agent_step_list_{i}_ms')
      agent_df = agent_df.join(sim_stats_df[f'agent_step_list_{i}_ms'])
      #print(agent_df)


    new_df = agent_df.groupby(by='num_cars').mean()

    print(new_df)
    print(y_columns)

    ax = create_stacked_bar_chart(y=y_columns, data=new_df , labels=labels)

    save_file_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/agent_step_time_boxplot.png'
    save_ax(ax, save_file_path)
    if SHOULD_SHOW:
      plt.show()
    plt.clf()


# In[276]:


step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_world_step_time'
client_step_time_df_path = f'{CUMULATIVE_STATS_FOLDER_PATH}/df_client_step_time'
client_perception_time_df_path =  f'{CUMULATIVE_STATS_FOLDER_PATH}/df_client_perception_time' 

# In[277]:


def plot_comparison_chart_time(data):
    # Plotting
    sns.set_style("whitegrid")
    sns.set_palette("deep")
    sns.set_context("talk")

    plt.figure(figsize=(10, 6))

    # Convert the DataFrame to a long format for easy plotting
    data_long = data.melt(id_vars=['num_vehicles'], value_vars=['eCloudSim', 'openCDA'], var_name='Simulation', value_name='Value')

    sns.lineplot(x='num_vehicles', y='Value', hue='Simulation', data=data_long, marker='o')

    plt.xlabel('Number of Vehicles')
    plt.xticks(data['num_vehicles'].unique())
    plt.ylabel('Total Simulation Time (s)')
    plt.title('Simulation Time with Perception on a Single Node\n(Multi2lane Scenario)')

    plt.legend(['eCloudSim', 'openCDA'])

    if SHOULD_SHOW:
        plt.show()


# In[278]:


def plot_comparison_chart_cpu(data):
    # Plotting
    sns.set_style("whitegrid")
    sns.set_palette("deep")
    sns.set_context("talk")

    plt.figure(figsize=(10, 6))

    sns.lineplot(x='num_vehicles', y='cpu_util', hue='Simulation', data=data, marker='o')

    plt.xlabel('Number of Vehicles')
    plt.xticks(data['num_vehicles'].unique())
    plt.ylabel('CPU Utilization (%)')
    plt.title('CPU Utilization without Perception on a Single Node\n(Multi2lane Scenario)')

    plt.legend(['eCloudSim', 'openCDA'])

    plt.ylim([40, 80])

    if SHOULD_SHOW:
        plt.show()


# In[279]:


if __name__ == '__main__':
    # Plotting simulation total run time stats
    plot_simulation_time()

    # Plotting simulation step time stats
    plot_world_step_time()

    # Plotting simulation step time stats
    plot_client_step_time()

    # Plotting simulation step time stats
    plot_client_perception_time()

    # Plotting simulation step time stats
    plot_client_localization_time()

    # Plotting simulation step time stats
    plot_client_control_time()

    plot_agent_step_times()

    plot_client_stacked_barchart()

   # Example DataFrame for comparison chart
    comparison_data = pd.DataFrame({
        'num_vehicles': [4, 8, 16],
        'eCloudSim': [72.3, 94.159503, 156.235791],
        'openCDA': [54.68, 90, 170]
    })

    # Plotting comparison chart
    plot_comparison_chart_time(comparison_data)

    data = {
        "Simulation": ["eCloudSim", "eCloudSim", "eCloudSim", "openCDA", "openCDA", "openCDA"],
        "num_vehicles": [4, 8, 16, 4, 8, 16],
        "cpu_util": [54.68, 73.70, 72.21, 55.00, 73.00, 74.00],
    }

    df = pd.DataFrame(data)
    plot_comparison_chart_cpu(df)


# In[ ]:




