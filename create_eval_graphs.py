import matplotlib.pyplot as plt
import pickle

sim_stats_file = 'df_total_sim_time'

try:
    picklefile = open(sim_stats_file, 'rb+')
    sim_time_df = pickle.load(picklefile)  # unpickle the dataframe
except:
    print(f"cannot find sim stats file {sim_stats_file}")

ax1 = sim_time_df.plot.scatter(x='num_cars', y='time_s', c='DarkBlue')

plt.savefig('total_sim_time.png')

picklefile.close()
