import os
import time
import psutil
import logging
from performance import calculate_path_length

#a helper function 
def measure_and_log_metrics(algorithm_name, algorithm, path, visited, plot_function, random_env):
    process = psutil.Process(os.getpid())
    start_memory = process.memory_info().rss
    start_time = time.time()

    # Execute the algorithm
    # path, visited = algorithm_execution_function()
    path, visited = algorithm.searching()

    end_time = time.time()
    end_memory = process.memory_info().rss

    # Calculate the metrics
    path_length = calculate_path_length(path)
    nodes_expanded = len(visited)
    execution_time = end_time - start_time
    memory_usage = end_memory - start_memory
    cpu_percent = psutil.cpu_percent(interval=None)


    # Log the metrics
    logging.info(f"Environment Size: {random_env.x_range}x{random_env.y_range}")
    logging.info(f"Obstacle Density: {random_env.obs_density}")
    logging.info(f"{algorithm_name} Metrics:")
    logging.info(f"Path Length: {path_length}")
    logging.info(f"Nodes Expanded: {nodes_expanded}")
    logging.info(f"Execution Time: {execution_time} seconds")
    logging.info(f"Memory Usage: {memory_usage} bytes")
    logging.info(f"CPU Usage: {cpu_percent}%")
    logging.info("\n")

     # Plot the algorithm animation if a plot_function is provided
    if plot_function:
        plot_function(path, visited, algorithm_name)
