

import os
import pickle
import json
import random
from grid_env import GridEnv  # Make sure to adjust the import as per your project structure
import matplotlib.pyplot as plt
import numpy as np


def obstacle_density_generate_and_save_environments(x_range, y_range, start_state, goal_state):
    directory = "obstacle_density_env"
    if not os.path.exists(directory):
        os.makedirs(directory)
        
    envs = []
    envs_dicts = []
    densities = np.arange(0.1, 0.45, 0.05)
    
    for obs_density in densities:
        env = GridEnv(x_range, y_range, obs_density, start_state, goal_state)
        manhattan_distance = abs(start_state[0]-goal_state[0]) + abs(start_state[1]-goal_state[1])
        envs.append(env)

        env_dict = {
            "start": env.start,
            "goal": env.goal,
            "obs": list(env.obs),
            "obs_density": obs_density,
            "manhattan_distance": manhattan_distance
        }
        envs_dicts.append(env_dict)
    
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

    for i, env in enumerate(envs):
        plt.figure(figsize=(5, 5))
        obs_x = [x[0] for x in env.obs]
        obs_y = [x[1] for x in env.obs]
        plt.scatter(obs_x, obs_y, s=1, color='black')
        plt.plot(env.start[0], env.start[1], "bs", label='Start')
        plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
        plt.title(f"Density: {env.obs_density:.2f}, Distance: {env.manhattan_distance}")
        plt.legend()
        plt.show()
# Set the parameters
x_range = 301
y_range = 301
start_state = (10, 10)
goal_state = (200, 200)

# # Generate and save the environments
# obstacle_density_generate_and_save_environments(x_range, y_range, start_state, goal_state)

# # Load environments and plot them
# envs = obstacle_density_load_environments()
# obstacle_density_plot_environment(envs)
# # Plot the first environment
# obstacle_density_plot_environment()


# # plot the environment
# i = 0.1
# for i, env in enumerate(envs):
#     plot_environment(env, i + 0.05)

