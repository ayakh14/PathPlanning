import os
import pickle
import json
import random
from grid_env import GridEnv
import matplotlib.pyplot as plt
import numpy as np

def grid_size_env_generate_and_save_environments(grid_sizes, start_state, goal_state):
    directory = "grid_size_env"
    if not os.path.exists(directory):
        os.makedirs(directory)

    envs = []
    envs_dicts = []
    obs_density = 0.25  # Define the obstacle density as per your requirement

    for grid_size in grid_sizes:
        env = GridEnv(grid_size, grid_size, obs_density, start_state, goal_state)
        manhattan_distance = abs(start_state[0]-goal_state[0]) + abs(start_state[1]-goal_state[1])

        # Store the grid_size, obs_density and manhattan_distance in the GridEnv object
        env.grid_size = grid_size
        env.obs_density = obs_density
        env.manhattan_distance = manhattan_distance

        envs.append(env)

        env_dict = {
            "grid_size": grid_size,
            "start": env.start,
            "goal": env.goal,
            "obs": list(env.obs),
            "obs_density": obs_density,
            "manhattan_distance": manhattan_distance
        }
        envs_dicts.append(env_dict)

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

    for i, env in enumerate(envs):
        plt.figure(figsize=(5, 5))
        obs_x = [x[0] for x in env.obs]
        obs_y = [x[1] for x in env.obs]
        plt.scatter(obs_x, obs_y, s=1, color='black')
        plt.plot(env.start[0], env.start[1], "bs", label='Start')
        plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
        plt.title(f"Grid Size: {env.grid_size}, Density: {env.obs_density}, Distance: {env.manhattan_distance}")
        plt.legend()
        plt.show()


# Set the parameters
grid_sizes = range(50, 301, 50)  # grid sizes from 50 to 300 with a scale of 50
start_state = (1, 1)
goal_state = (48, 48)  # change the position of goal_state as per your requirement

# Generate and save the environments
# grid_size_env_generate_and_save_environments(grid_sizes, start_state, goal_state)

# Load environments and plot them
# envs = grid_size_env_load_environments()
# grid_size_env_plot_environment(envs)
