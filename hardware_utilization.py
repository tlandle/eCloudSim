import threading
import time
import psutil
import pickle
import logging
import argparse
import subprocess

import pandas as pd
import numpy as np
import coloredlogs

def arg_parse():
    parser = argparse.ArgumentParser(description="hwardware utilization collector.")
    parser.add_argument('-n', "--num_cars", type=int, default=0,
                        help='number of cars running on this node for this scenario. DEFAULT: 0')
    parser.add_argument('-t', "--time_interval", type=float, default=5.0,
                        help='how often to run analysis *in seconds.* DEFAULT: 5.0s')
    parser.add_argument('-s', "--sequential", action='store_true',
                        help='running sequential? DEFAULT: false - assume distributed scenarios by default')
    parser.add_argument('-p', "--perception", action='store_true',
                        help='was perception active? *enables GPU measurement* DEFAULT: false - assume non-perception by default')
    parser.add_argument('-v', "--verbose", action='store_true',
                        help="enables DEBUG level logging; otherwise defaults to INFO")
    opt = parser.parse_args()
    return opt

def do_pickling(column_key, flat_list, file_path, num_cars, perception=False, sequential=False ):
    logger.info("run stats for %s:\nmean %s: %s \nmedian %s: %s \n95th percentile %s %s",
                column_key, column_key, np.mean(flat_list),
                column_key, np.median(flat_list),
                column_key, np.percentile(flat_list, 95))

    data_df = pd.DataFrame(flat_list, columns = [f'{column_key}'])
    data_df['num_cars'] = num_cars
    data_df['run_timestamp'] = pd.Timestamp.today().strftime('%Y-%m-%d %X')
    data_df = data_df[['num_cars', f'{column_key}', 'run_timestamp']]

    suffix = "perception" if perception else "no_perception"
    suffix = suffix + ( "_sequential" if sequential else "_distributed" )

    data_df_path = f'./{file_path}/df_{column_key}_{suffix}'
    try:
        picklefile = open(data_df_path, 'rb+')
        current_data_df = pickle.load(picklefile)  #unpickle the dataframe
    except:
        picklefile = open(data_df_path, 'wb+')
        current_data_df = pd.DataFrame(columns=['num_cars', f'{column_key}', 'run_timestamp'])

    picklefile = open(data_df_path, 'wb+')
    data_df = pd.concat([current_data_df, data_df], axis=0, ignore_index=True)

    pickle.dump(data_df, picklefile)
    logger.debug(data_df)
    picklefile.close()

def collect_gpu_utilization(gpu_percentages, interval, stop_event):
    while not stop_event.is_set():
        try:
            result = subprocess.check_output(["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"]).decode("utf-8").strip()
            gpu_percent = float(result)
            gpu_percentages.append(gpu_percent)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            break
        time.sleep(interval)

def collect_cpu_utilization(cpu_percentages, interval, stop_event):
    while not stop_event.is_set():
        cpu_percent = psutil.cpu_percent(interval=interval)
        cpu_percentages.append(cpu_percent)
        time.sleep(interval)

if __name__ == "__main__":
    opt = arg_parse()

    global logger
    logger = logging.getLogger(__name__)
    level = 'DEBUG' if opt.verbose else 'INFO'
    coloredlogs.install(level=level, logger=logger)
    
    gpu_percentages = []
    cpu_percentages = []

    collection_interval = opt.time_interval

    stop_event = threading.Event()

    cpu_collection_thread = threading.Thread(target=collect_cpu_utilization, args=(cpu_percentages, collection_interval, stop_event))
    cpu_collection_thread.start()

    gpu_collection_thread = None
    if opt.perception:
        gpu_collection_thread = threading.Thread(target=collect_gpu_utilization, args=(gpu_percentages, collection_interval, stop_event))
        gpu_collection_thread.start()

    print("Press Ctrl+C to stop the script")

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping the script...")

    stop_event.set()
    cpu_collection_thread.join()
    if gpu_collection_thread is not None:
        gpu_collection_thread.join()

    file_path = './evaluation_outputs'
    if opt.perception:
        do_pickling(column_key='gpu_usage',
                    flat_list=gpu_percentages,
                    file_path=file_path,
                    num_cars=opt.num_cars,
                    perception=opt.perception,
                    sequential=opt.sequential)
    
    do_pickling(column_key='cpu_usage',
                flat_list=cpu_percentages,
                file_path=file_path,
                num_cars=opt.num_cars,
                perception=opt.perception,
                sequential=opt.sequential)
