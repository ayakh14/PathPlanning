
import random
# from changing_grid_size import grid_size_env_load_environments, grid_size_env_plot_environment
# from changing_obstacle_density import obstacle_density_load_environments, obstacle_density_plot_environment
# from changing_start_goal_distance import start_goal_distance_load_environments, start_goal_distance_plot_environment
from vertical_walls_env import load_vertical_walls_environments, plot_vertical_walls_environments 
from change_wall_size import load_vertical_wall_size_environments, plot_environments
import os
import sys
from itertools import chain
from Astar import start_astar
from LRTAstar import lrta_star_alg
from RTAAStar import rtaa_algo
from ARAstar import ara_star
import io
import contextlib
from D_star import dstar_algo
from D_star_Lite import dstar_lite_algo
from LPAstar import lpa_algo

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")
import csv
import gc




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
    # {"name": "Grid Size Environments", "load": grid_size_env_load_environments, "plot": grid_size_env_plot_environment}, 
    # {"name": "Obstacle Density Environments", "load": obstacle_density_load_environments, "plot": obstacle_density_plot_environment}, 
    # {"name": "Start to Goal Distance Environments", "load": start_goal_distance_load_environments, "plot": start_goal_distance_plot_environment}, 
    {"name": "Vertical Walls Environments", "load": load_vertical_walls_environments, "plot": plot_vertical_walls_environments}, 
    {"name": "Vertical Wall Size Environments", "load": load_vertical_wall_size_environments, "plot": plot_environments}
]

# Create a mapping between algorithm names and integer values
algorithm_mapping = {
    # "LRTA*": 1,
    "RTAA*": 2,
    "ARA*": 3,
    "D* Lite": 4,
    # "D*": 5,
    # "LPA*": 6,
    # "A*": 7
}



# Store algorithm names and functions in a list 
algorithms = [
    # {"name": "A*", "func": start_astar},
    # {"name": "LRTA*", "func": lrta_star_alg},
    {"name": "RTAA*", "func": rtaa_algo},
    {"name": "ARA*", "func": ara_star},
    {"name": "D* Lite", "func": dstar_lite_algo},
    # {"name": "D*", "func": dstar_algo},
    # {"name": "LPA*", "func": lpa_algo},
]
# Choose the priorities to be tested
priorities = ['PathCost', 'ExecutionTime', 'Memory']
grid_number = 1

# # Iterate over each environment set
# for selected_env in environments:
#     # Load environments from the specific environment set
#     envs = selected_env["load"]()
#     # Check if there are any environments to select from
#     if not envs:
#         print("No environments to select from.")
#     else:
#         # Randomly select one grid parameter
#         if selected_env["name"] in ["Vertical Walls Environments", "Vertical Wall Size Environments"]:
            
#             selected_envs = [random.choice(envs)]

#         else:
#             # If it's not those special cases, select 10 instances of the same grid parameter
#             start_index = random.choice(range(0, len(envs), 10))
#             selected_envs = envs[start_index:start_index + 10]

#         # Iterate over the selected environments (10 instances of the same grid parameter)
#     for env_ in selected_envs:
#         grid_number += 1
#         # Print the selected environment
#         print("###################################### \n ######################################")
#         print(f"Selected Environment from {selected_env['name']}: Start={env_.start}, Goal={env_.goal}, Grid Size=({env_.x_range}, {env_.y_range}), obs_density={env_.obs_density:.2f}, s/g distance={env_.euclidean_distance} ")

#         # Add "big_obstacles" attribute to environments coming from "change_wall_size" and "vertical_walls_env"
#         if selected_env["name"] in ["Vertical Wall Size Environments", "Vertical Walls Environments"]:
#             big_obstacles = True
#         else:
#             big_obstacles = False

#         # Plot the selected grid
#         plot_func = selected_env["plot"]
#         plot_func(env_)

#         # Iterate over all priorities
#         for priority in priorities:
#             # Select the best algorithm based on priority and environment
#             selected_algo = select_algorithm(priority, env_)
#             print(f"Selected Algorithm: {selected_algo.__name__}, Priority: {priority}")

#             # Define the filename based on the selected algorithm and priority
#             filename = f"comparison_result_{selected_env['name']}_{priority}_{selected_algo.__name__}.csv"
#             file_path = os.path.join("comparison_result", filename)

#             with open(file_path, 'a', newline='') as file:
#                 writer = csv.writer(file)

#                 # Check if the file is empty and write the header if it is
#                 if os.stat(file_path).st_size == 0:
#                     writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])

#                 # if os.stat(file_path).st_size == 0:


#                             # Apply each algorithm to all selected environments
#                 for exp in range(1, 2):
#                     for algo in algorithms:
#                         try:
#                             # Get the corresponding integer value for the algorithm
#                             exp = algorithm_mapping[algo["name"]]
#                             # Pass this integer value as `exp` to `capture_prints`
#                             algo_prints = capture_prints(algo["func"], env_, writer, exp, grid_number) 
#                         except Exception as e:
#                             print(f"Error running {algo['name']}: {e}")                    
#                         print(f"Algorithm: {algo['name']}, Priority: {priority}")
#                         print(f"Algorithm Prints: {algo_prints}")

#                 # Increment the grid number after processing each grid
#                 grid_number += 1

#                     # Call garbage collector to free up memory
#                 gc.collect()

#                 print(f"All selected environments have been processed with {algo['name']}.")

#                 print("All algorithms have been applied to all selected environments.")
#                 print("All environments in this set have been processed.")

# Iterate over each environment set
for selected_env in environments:
    # Load environments from the specific environment set
    envs = selected_env["load"]()
    # Check if there are any environments to select from
    if not envs:
        print("No environments to select from.")
    else:
        # # Randomly select one grid parameter
        # selected_envs = [random.choice(envs)]
        # Randomly select one grid parameter
        if selected_env["name"] in ["Vertical Walls Environments", "Vertical Wall Size Environments"]:
            
            selected_envs = [random.choice(envs)]

        else:
            # If it's not those special cases, select 10 instances of the same grid parameter
            start_index = random.choice(range(0, len(envs), 10))
            selected_envs = envs[start_index:start_index + 10]
        # First loop through algorithms
        for algo in algorithms:
            
            # Iterate over the selected environments (single instance)
            for env_ in selected_envs:
                # Print the selected environment
                print("###################################### \n ######################################")
                print(f"Selected Environment from {selected_env['name']}: Start={env_.start}, Goal={env_.goal}, Grid Size=({env_.x_range}, {env_.y_range}), wall length={env_.wall_length:.2f}, number of walls={env_.num_walls:.2f}, s/g distance={env_.euclidean_distance} ")

                # Plot the selected grid
                plot_func = selected_env["plot"]
                plot_func(env_)

                # Define the filename based on the selected algorithm and priority
                filename = f"comparison_result_{selected_env['name']}.csv"
                file_path = os.path.join("comparison_result", filename)

                with open(file_path, 'a', newline='') as file:
                    writer = csv.writer(file)

                    # Check if the file is empty and write the header if it is
                    if os.stat(file_path).st_size == 0:
                        writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])

                    try:
                        # Get the corresponding integer value for the algorithm
                        exp = algorithm_mapping[algo["name"]]
                        # Pass this integer value as `exp` to `capture_prints`
                        algo_prints = capture_prints(algo["func"], env_, writer, exp, grid_number) 
                    except Exception as e:
                        print(f"Error running {algo['name']}: {e}")                    
                    print(f"Algorithm: {algo['name']},")
                    print(f"Algorithm Prints: {algo_prints}")

                # Call garbage collector to free up memory
                gc.collect()

                print(f"All selected environments have been processed with {algo['name']}.")

                print("All algorithms have been applied to all selected environments.")
                print("All environments in this set have been processed.")


# import random
# from changing_grid_size import grid_size_env_load_environments, grid_size_env_plot_environment
# from changing_obstacle_density import obstacle_density_load_environments, obstacle_density_plot_environment
# from changing_start_goal_distance import start_goal_distance_load_environments, start_goal_distance_plot_environment
# from vertical_walls_env import load_vertical_walls_environments, plot_vertical_walls_environments 
# from change_wall_size import load_vertical_wall_size_environments, plot_environments
# import os
# import sys
# from itertools import chain
# from Astar import start_astar
# from LRTAstar import lrta_star_alg
# from RTAAStar import rtaa_algo
# from ARAstar import ara_star
# import io
# import contextlib
# from D_star import dstar_algo
# from D_star_Lite import dstar_lite_algo
# from LPAstar import lpa_algo

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../Search_based_Planning/")
# import csv
# import gc



# def select_algorithm(priority, env_):
#     if priority == 'Memory':
#         if big_obstacles:
#             return lpa_algo
#         else:
#             return dstar_lite_algo
#     elif priority == 'ExecutionTime':
#         if not big_obstacles:
#             if env_.euclidean_distance < 140:
#                 return ara_star
#             else:
#                 return rtaa_algo
#         else:
#             if env_.x_range * env_.y_range <= 100:
#                 return rtaa_algo
#             else:
#                 return ara_star
#     elif priority == 'PathCost':
#         return dstar_lite_algo

# def capture_prints(func, *args, **kwargs):
#     # Create a new io.StringIO object that will be used to catch prints
#     captured_output = io.StringIO()
#     # Redirect stdout to the StringIO object
#     with contextlib.redirect_stdout(captured_output):
#         # Call the function, all print statements will now be redirected to captured_output
#         func(*args, **kwargs)
#     # Get the output from the StringIO object
#     output = captured_output.getvalue()
#     # Return the captured output
#     return output

# # Store environment loading and plotting functions in lists, along with a description for each
# environments = [
#     {"name": "Grid Size Environments", "load": grid_size_env_load_environments, "plot": grid_size_env_plot_environment}, 
#     {"name": "Obstacle Density Environments", "load": obstacle_density_load_environments, "plot": obstacle_density_plot_environment}, 
#     {"name": "Start/Goal Distance Environments", "load": start_goal_distance_load_environments, "plot": start_goal_distance_plot_environment}, 
#     {"name": "Vertical Walls Environments", "load": load_vertical_walls_environments, "plot": plot_vertical_walls_environments}, 
#     {"name": "Vertical Wall Size Environments", "load": load_vertical_wall_size_environments, "plot": plot_environments}
# ]

# # Choose the priorities to be tested
# priorities = ['PathCost', 'ExecutionTime', 'Memory']

# # Iterate over each environment set
# for selected_env in environments:
#     # Load environments from the specific environment set
#     envs = selected_env["load"]()
#     # Check if there are any environments to select from
#     if not envs:
#         print("No environments to select from.")
#     else:
#         # Select one environment randomly
#         env_ = random.choice(envs)
#         # Print the selected environment
#         print("###################################### \n ######################################")

#         print(f"Selected Environment from {selected_env['name']}: Start={env_.start}, Goal={env_.goal}, Grid Size=({env_.x_range}, {env_.y_range}), obs_dancity {env_.obs_density:.2f}, s/g distance {env_.euclidean_distance} ")

#         # Add "big_obstacles" attribute to environments coming from "change_wall_size" and "vertical_walls_env"
#         if selected_env["name"] in ["Vertical Wall Size Environments", "Vertical Walls Environments"]:
#             big_obstacles = True
#         else:
#             big_obstacles = False

#         # Plot the selected grid
#         plot_func = selected_env["plot"]
#         plot_func(env_)


#     # Create the directory if it doesn't exist
#     if not os.path.exists("comparison_result"):
#         os.makedirs("comparison_result")

#     with open("comparison_result/comparison_result.csv", 'w', newline='') as file:
#         writer = csv.writer(file)
        
#         # Write the header of the CSV file
#         writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])
                        
#         # Store algorithm names and functions in a list 
#         algorithms = [
#             {"name": "A*", "func": start_astar},
#             {"name": "LRTA*", "func": lrta_star_alg},
#             {"name": "RTAA*", "func": rtaa_algo},
#             {"name": "ARA*", "func": ara_star},
#             {"name": "D* Lite", "func": dstar_lite_algo},
#             {"name": "D*", "func": dstar_algo},
#             {"name": "LPA*", "func": lpa_algo},


#         ]
#         # Apply each algorithm to the selected environment for each priority
#         for priority in priorities:
#             # Select the best algorithm based on priority and environment
#             selected_algo = select_algorithm(priority, env_)
#             print(f"Selected Algorithm: {selected_algo.__name__}, Priority: {priority}")

#             # Apply each algorithm to the selected environment
#             for exp in range(1, 2):
#                 for i, algo in enumerate(algorithms):
#                     try:
#                         algo_prints = capture_prints(algo["func"], env_, writer, exp, 0)  # Grid number is set to 0 as there's only one grid being processed
#                     except Exception as e:
#                         print(f"Error running {algo['name']}: {e}")                    
#                     print(f"Algorithm: {algo['name']}, Priority: {priority}")
#                     print(f"Algorithm Prints: {algo_prints}")

#             # Call garbage collector to free up memory
#             gc.collect()
                
#         print("The selected environment has been processed.")






# import random
# # from changing_grid_size import grid_size_env_load_environments, grid_size_env_plot_environment
# # from changing_obstacle_density import obstacle_density_load_environments, obstacle_density_plot_environment
# # from changing_start_goal_distance import start_goal_distance_load_environments, start_goal_distance_plot_environment
# from vertical_walls_env import load_vertical_walls_environments, plot_vertical_walls_environments 
# from change_wall_size import load_vertical_wall_size_environments, plot_environments
# import os
# import sys
# from itertools import chain
# from Astar import start_astar
# from LRTAstar import lrta_star_alg
# from RTAAStar import rtaa_algo
# from ARAstar import ara_star
# import io
# import contextlib
# from D_star import dstar_algo
# from D_star_Lite import dstar_lite_algo
# from LPAstar import lpa_algo

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../Search_based_Planning/")
# import csv
# import gc



# def select_algorithm(priority, env_):
#     if priority == 'Memory' or priority == 'PathCost':
#         return dstar_lite_algo
#     elif  priority == 'ExecutionTime':
#         if env_.euclidean_distance >= 140:
#             return rtaa_algo
#         else:
#             return ara_star
   
# def sanitize_filename(name):
#     return name.replace('*', '')    

# def capture_prints(func, *args, **kwargs):
#     # Create a new io.StringIO object that will be used to catch prints
#     captured_output = io.StringIO()
#     # Redirect stdout to the StringIO object
#     with contextlib.redirect_stdout(captured_output):
#         # Call the function, all print statements will now be redirected to captured_output
#         func(*args, **kwargs)
#     # Get the output from the StringIO object
#     output = captured_output.getvalue()
#     # Return the captured output
#     return output

# # Store environment loading and plotting functions in lists, along with a description for each
# environments = [
#     # {"name": "Grid Size Environments", "load": grid_size_env_load_environments, "plot": grid_size_env_plot_environment}, 
#     # {"name": "Obstacle Density Environments", "load": obstacle_density_load_environments, "plot": obstacle_density_plot_environment}, 
#     # {"name": "Start to Goal Distance Environments", "load": start_goal_distance_load_environments, "plot": start_goal_distance_plot_environment}, 
#     {"name": "Vertical Walls Environments", "load": load_vertical_walls_environments, "plot": plot_vertical_walls_environments}, 
#     {"name": "Vertical Wall Size Environments", "load": load_vertical_wall_size_environments, "plot": plot_environments}
# ]

# # Create a mapping between algorithm names and integer values
# algorithm_mapping = {
#     # "LRTA*": 1,
#     "RTAA*": 2,
#     "ARA*": 3,
#     "D* Lite": 4,
#     # "D*": 5,
#     # "LPA*": 6,
#     # "A*": 7
# }
# filename = "comparison_result.csv"
# file_path = os.path.join("comparison_result", filename)


# # Store algorithm names and functions in a list 
# algorithms = [
#     # {"name": "A*", "func": start_astar},
#     # {"name": "LRTA*", "func": lrta_star_alg},
#     {"name": "RTAA*", "func": rtaa_algo},
#     {"name": "ARA*", "func": ara_star},
#     {"name": "D* Lite", "func": dstar_lite_algo},
#     # {"name": "D*", "func": dstar_algo},
#     # {"name": "LPA*", "func": lpa_algo},
# ]
# # Choose the priorities to be tested
# priorities = ['PathCost', 'ExecutionTime', 'Memory']
# grid_number = 1

# with open(file_path, 'a', newline='') as file:
#     writer = csv.writer(file)
#     if os.stat(file_path).st_size == 0:
#         writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead", "Path cost", "Number of expanded nodes", "Number of searches", "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])

#     for selected_env in environments:
#         envs = selected_env["load"]()
#         if not envs:
#             print("No environments to select from.")
#             continue
        
#         # Choose a random environment
#         selected_env_ = random.choice(envs)
        
#         # Plot the chosen environment
#         plot_func = selected_env["plot"]
#         plot_func(selected_env_)
        
#         # Select algorithm to run on this environment
#         algo_func = select_algorithm(selected_env_)
#         algo_prints = capture_prints(algo_func, selected_env_, writer, "Selected Algorithm", 1)