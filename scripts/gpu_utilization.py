import threading
import time
import subprocess

# GPU utilization collection function
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

# Function to calculate average GPU utilization
def calculate_average_gpu_utilization(gpu_percentages):
    return sum(gpu_percentages) / len(gpu_percentages)

if __name__ == "__main__":
    gpu_percentages = []
    collection_interval = 5 # Time interval between GPU utilization measurements in seconds

    # Create an event to signal when the script should stop
    stop_event = threading.Event()

    # Start the GPU utilization collection thread
    gpu_collection_thread = threading.Thread(target=collect_gpu_utilization, args=(gpu_percentages, collection_interval, stop_event))
    gpu_collection_thread.start()

    print("Press Ctrl+C to stop the script")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping the script...")

    # Signal the GPU utilization collection thread to stop and wait for it to finish
    stop_event.set()
    gpu_collection_thread.join()

    # Calculate and print the average GPU utilization
    avg_gpu_utilization = calculate_average_gpu_utilization(gpu_percentages)
    print(f"Average GPU utilization during the simulation: {avg_gpu_utilization:.2f}%")
