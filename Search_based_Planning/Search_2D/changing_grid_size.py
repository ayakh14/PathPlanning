"""
    This script generates grids with the same obstacle density and same start and goal 
    states positions. However, each time the grid size changes and it starts from size
    (51x51) till (301x301) with a scale of 50 in both length and hight.

    We generate 10 instances from the same grid size, for fair comparision.  

"""
import os
import pickle
import json
import random
from grid_env import GridEnv
import matplotlib.pyplot as plt
import numpy as np
import math

def grid_size_env_generate_and_save_environments(grid_sizes, start_state, goal_state):
    directory = "grid_size_env"
    if not os.path.exists(directory):
        os.makedirs(directory)

    envs = []
    envs_dicts = []
    obs_density = 0.25  # Define the obstacle density 

    for grid_size in grid_sizes:
        for _ in range(10):  # Generate 10 instances for each grid size
            while True:
                env = GridEnv(grid_size, grid_size, obs_density, start_state, goal_state)
                euclidean_distance = math.sqrt((start_state[0] - goal_state[0])**2 + 
                                                (start_state[1] - goal_state[1])**2)  # Calculate Euclidean distance

                # Store the grid_size, obs_density, and euclidean_distance in the GridEnv object
                env.grid_size = grid_size
                env.obs_density = obs_density
                env.euclidean_distance = euclidean_distance

                # Import AStar here, inside the function where it's used
                from Astar import AStar

                # Run A* on the grid to check if a path exists
                astar = AStar(env.start, env.goal, "euclidean", env)
                path, _ = astar.searching()
                if path is not None:  # A* found a path
                    envs.append(env)
                    env_dict = {
                        "grid_size": grid_size,
                        "start": env.start,
                        "goal": env.goal,
                        "obs": list(env.obs),
                        "obs_density": obs_density,
                        "euclidean_distance": euclidean_distance
                    }
                    envs_dicts.append(env_dict)
                    break

    try:
        with open(os.path.join(directory, 'grid_size_env.pkl'), 'wb') as f:
            pickle.dump(envs, f)

        with open(os.path.join(directory, 'grid_size_env.json'), 'w') as f:
            json.dump(envs_dicts, f)
        print(f'Successfully generated and saved environments with varying grid sizes.')
    except Exception as e:
        print(f'Failed to save environments. Error: {e}')


def grid_size_env_load_environments():
    try:
        with open('grid_size_env/grid_size_env.pkl', 'rb') as f:
            envs = pickle.load(f)
        
        return envs
    except FileNotFoundError:
        print("Environments file not found.")
        return None


def grid_size_env_plot_environment(envs):
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
            plt.title(f"Grid Size: {env.grid_size}, Density: {env.obs_density}, Distance: {env.euclidean_distance:.2f}")
            plt.legend()
            # plt.show()
    else:
        print("ffffffffffffffffffffffff")

        env = envs
        plt.figure(figsize=(5, 5))
        obs_x = [x[0] for x in env.obs]
        obs_y = [x[1] for x in env.obs]
        plt.scatter(obs_x, obs_y, s=1, color='black')
        plt.plot(env.start[0], env.start[1], "bs", label='Start')
        plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
        plt.title(f"Grid Size: {env.grid_size}, Density: {env.obs_density}, Distance: {env.euclidean_distance:.2f}")
        plt.legend()
        # plt.show()


# Set the parameters
grid_sizes = range(50, 301, 50)  # Grid sizes from 50 to 300 with a step of 50
start_state = (1, 1)
goal_state = (48, 48)  # Change the position of goal_state as per your requirement

# # Generate and save the environments
# grid_size_env_generate_and_save_environments(grid_sizes, start_state, goal_state)

# # Load environments and plot them
# envs = grid_size_env_load_environments()
# # grid_size_env_plot_environment(envs)

# for env in envs:
#     print(f"Start: {env.start}, Goal: {env.goal}, S/G Distance: {env.euclidean_distance:.2f}, Obstacle Density: {env.obs_density:.2f}, Grid Size: {env.grid_size}")
