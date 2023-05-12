import threading
import time
import psutil

# CPU utilization collection function
def collect_cpu_utilization(cpu_percentages, interval, stop_event):
    while not stop_event.is_set():
        cpu_percent = psutil.cpu_percent(interval=interval)
        cpu_percentages.append(cpu_percent)
        time.sleep(interval)

# Function to calculate average CPU utilization
def calculate_average_cpu_utilization(cpu_percentages):
    return sum(cpu_percentages) / len(cpu_percentages)

if __name__ == "__main__":
    cpu_percentages = []
    collection_interval = 5 # Time interval between CPU utilization measurements in seconds

    # Create an event to signal when the script should stop
    stop_event = threading.Event()

    # Start the CPU utilization collection thread
    cpu_collection_thread = threading.Thread(target=collect_cpu_utilization, args=(cpu_percentages, collection_interval, stop_event))
    cpu_collection_thread.start()

    print("Press Ctrl+C to stop the script")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping the script...")

    # Signal the CPU utilization collection thread to stop and wait for it to finish
    stop_event.set()
    cpu_collection_thread.join()

    # Calculate and print the average CPU utilization
    avg_cpu_utilization = calculate_average_cpu_utilization(cpu_percentages)
    print(f"Average CPU utilization during the simulation: {avg_cpu_utilization:.2f}%")
