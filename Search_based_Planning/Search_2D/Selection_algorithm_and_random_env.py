
import random
from random_env_for_validation import load_environments, plot_environment
from vertical_walls_env import load_vertical_walls_environments, plot_vertical_walls_environments 
from change_wall_size import load_vertical_wall_size_environments, plot_environments
import os
import sys
from itertools import chain
# from Astar import start_astar
# from LRTAstar import lrta_star_alg
from RTAAStar import rtaa_algo
from ARAstar import ara_star
import io
import contextlib
# from D_star import dstar_algo
from D_star_Lite import dstar_lite_algo
# from LPAstar import lpa_algo

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")
import csv
import gc


# selection algorithm 
def select_algorithm(priority, env_):
    if priority == 'Memory' or priority == 'PathCost':
        return dstar_lite_algo
    elif  priority == 'ExecutionTime':
        if env_.euclidean_distance >= 140:
            return rtaa_algo
        else:
            return ara_star
   
    

def capture_prints(func, *args, **kwargs):
    # Create a new io.StringIO object that will be used to catch prints
    captured_output = io.StringIO()
    # Redirect stdout to the StringIO object
    with contextlib.redirect_stdout(captured_output):
        # Call the function, all print statements will now be redirected to captured_output
        func(*args, **kwargs)
    # Get the output from the StringIO object
    output = captured_output.getvalue()
    # Return the captured output
    return output

# Store environment loading and plotting functions in lists, along with a description for each
environments = [
    {"name": "random environment", "load": load_environments, "plot": plot_environment},
    {"name": "Vertical Walls Environments", "load": load_vertical_walls_environments, "plot": plot_vertical_walls_environments}, 
    {"name": "Vertical Wall Size Environments", "load": load_vertical_wall_size_environments, "plot": plot_environments}
]

# Create a mapping between algorithm names and integer values
algorithm_mapping = {
    "LRTA*": 1,
    "RTAA*": 2,
    "ARA*": 3,
    "D* Lite": 4,
    "D*": 5,
    "LPA*": 6,
    # "A*": 7
}



# Store algorithm names and functions in a list 
algorithms = [
    # {"name": "A*", "func": start_astar},
    # {"name": "LRTA*", "func": lrta_star_alg},
    {"name": "RTAA*", "func": rtaa_algo},
    {"name": "ARA*", "func": ara_star},
    {"name": "D* Lite", "func": dstar_lite_algo}
    # {"name": "D*", "func": dstar_algo},
    # {"name": "LPA*", "func": lpa_algo},
]
# Choose the priorities to be tested
priorities = ['PathCost', 'ExecutionTime', 'Memory']
environments = load_environments()
print(f"Total number of grids/environments loaded: {len(environments)}")

if not environments:
    print("No environments to select from.")
    exit()

grid_number = 1
filename = "comparison_result_random_environment2.csv"  # Only one CSV file for all results
file_path = os.path.join("random_comparison_result", filename)

# Check if the file is empty and write the header if it is
if not os.path.exists(file_path) or os.stat(file_path).st_size == 0:
    with open(file_path, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Algorithm", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead", "Path cost", "Number of expanded nodes", "Number of searches", "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])

for env_ in environments:
    # Print and plot the environment
    print("###################################### \n ######################################")
    print(f"Selected Environment: Start={env_.start}, Goal={env_.goal}, Grid Size=({env_.x_range}, {env_.y_range}), obs_density={env_.obs_density:.2f}, s/g distance={env_.euclidean_distance} ")
    plot_environment(env_)  # Plot the selected grid

    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        try:
            # Run the algorithm and capture its prints
            for algo in algorithms:
                exp = algorithm_mapping[algo["name"]]
                algo_prints = capture_prints(algo["func"], env_, writer, exp, grid_number)
                print(f"Algorithm: {algo['name']}")
                print(f"Algorithm Prints: {algo_prints}")
        except Exception as e:
            print(f"Error running {algo['name']} algorithm: {e}")

        # Call garbage collector to free up memory
        gc.collect()

        print(f"All algorithms have been applied to the current environment.")

    grid_number += 1  # Increment grid_number for the next environment

print("Finished processing all environments.")