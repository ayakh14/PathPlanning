

"""
    This script generates randomly a set of environments with varying obstacle densities (10 instances of 
    the same obstacle density) and calculates the Euclidean distance from a given start point to a goal 
    point for each environment. The positions of both start and goal are fixed. 
"""

import os
import pickle
import json
import random
from grid_env import GridEnv
import matplotlib.pyplot as plt
import numpy as np
import math
# from Astar import AStar

def obstacle_density_generate_and_save_environments(x_range, y_range, start_state, goal_state):
    directory = "obstacle_density_env"
    if not os.path.exists(directory):
        os.makedirs(directory)
        
    envs = []
    envs_dicts = []
    densities = np.arange(0.1, 0.45, 0.05)  # Range of densities to be used  
    
    for obs_density in densities:
        for _ in range(10):  # Generate 10 instances for each obstacle density
            while True:
                env = GridEnv(x_range, y_range, obs_density, start_state, goal_state)
                euclidean_distance = math.sqrt((start_state[0] - goal_state[0])**2 + 
                                                (start_state[1] - goal_state[1])**2)  # Calculate Euclidean distance
                
                # Import AStar here, inside the function where it's used
                from Astar import AStar
                
                # Run A* on the grid to check if a path exists
                astar = AStar(env.start, env.goal, "euclidean", env)
                path, _ = astar.searching()
                if path is not None: # A* found a path
                    envs.append(env)
                    env_dict = {
                        "start": env.start,
                        "goal": env.goal,
                        "obs": list(env.obs),
                        "obs_density": obs_density,
                        "euclidean_distance": euclidean_distance
                    }
                    envs_dicts.append(env_dict)
                    break  # If a valid path was found, break the loop and move to the next density
    
    try:
        with open(os.path.join(directory, 'obstacle_density_env.pkl'), 'wb') as f:
            pickle.dump(envs, f)

        with open(os.path.join(directory, 'obstacle_density_env.json'), 'w') as f:
            json.dump(envs_dicts, f)
        print(f'Successfully generated and saved environments with varying densities.')
    except Exception as e:
        print(f'Failed to save environments. Error: {e}')

def obstacle_density_load_environments():
    try:
        with open('obstacle_density_env/obstacle_density_env.pkl', 'rb') as f:
            envs = pickle.load(f)
        return envs
    except FileNotFoundError:
        print("Environments file not found.")
        return None

def obstacle_density_plot_environment(envs):
    if envs is None:
        print("No environments to plot.")
        return

    if isinstance(envs, list):
        for i, env in enumerate(envs):
            plt.figure(figsize=(5, 5))
            obs_x = [x[0] for x in env.obs]
            obs_y = [x[1] for x in env.obs]
            plt.scatter(obs_x, obs_y, s=1, color='black')
            plt.plot(env.start[0], env.start[1], "bs", label='Start')
            plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
            plt.title(f"Density: {env.obs_density:.2f}, Distance: {env.euclidean_distance:.2f}")
            plt.legend()
            # plt.show()
    else:
        env = envs
        plt.figure(figsize=(5, 5))
        obs_x = [x[0] for x in env.obs]
        obs_y = [x[1] for x in env.obs]
        plt.scatter(obs_x, obs_y, s=1, color='black')
        plt.plot(env.start[0], env.start[1], "bs", label='Start')
        plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
        plt.title(f"Density: {env.obs_density:.2f}, Distance: {env.euclidean_distance:.2f}")
        plt.legend()
        # plt.show()

# Set the parameters
x_range = 301
y_range = 301
start_state = (10, 10)
goal_state = (200, 200)

# # Generate and save the environments
# obstacle_density_generate_and_save_environments(x_range, y_range, start_state, goal_state)

# Load environments and plot them
# envs = obstacle_density_load_environments()
# # obstacle_density_plot_environment(envs)

# for env in envs:
#     print(f"Start: {env.start}, Goal: {env.goal}, S/G Distance: {env.euclidean_distance:.2f}, Obstacle Density: {env.obs_density:.2f}")
