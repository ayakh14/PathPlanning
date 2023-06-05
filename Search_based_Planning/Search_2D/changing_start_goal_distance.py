
import os
import pickle
import json
import random
from grid_env import GridEnv  # Make sure to adjust the import as per your project structure
import matplotlib.pyplot as plt
import numpy as np

def start_goal_distance_generate_and_save_environments(grid_size, start_states, goal_states):
    directory = "start_goal_distance_env"
    if not os.path.exists(directory):
        os.makedirs(directory)

    envs = []
    envs_dicts = []
    obs_density = 0.25  # Define the obstacle density as per your requirement

    for start_state, goal_state in zip(start_states, goal_states):
        env = GridEnv(grid_size, grid_size, obs_density, start_state, goal_state)
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
        with open(os.path.join(directory, 'start_goal_distance_env.pkl'), 'wb') as f:
            pickle.dump(envs, f)

        with open(os.path.join(directory, 'start_goal_distance_env.json'), 'w') as f:
            json.dump(envs_dicts, f)
        print(f'Successfully generated and saved environments with varying start and goal states.')
    except Exception as e:
        print(f'Failed to save environments. Error: {e}')

def start_goal_distance_load_environments():
    try:
        with open('start_goal_distance_env/start_goal_distance_env.pkl', 'rb') as f:
            envs = pickle.load(f)
        return envs
    except FileNotFoundError:
        print("Environments file not found.")
        return None

def start_goal_distance_plot_environment(envs):
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
        plt.title(f"Density: {env.obs_density}, Distance: {env.manhattan_distance}")
        plt.legend()
        plt.show()

# Set the parameters
grid_size = 300
start_states = [(1, 1), (30, 30), (60, 60), (90, 90), (120, 120), (140, 140)]
goal_states = [(298, 298), (270, 270), (240, 240), (210, 210), (180, 180), (160, 160)]

# # Generate and save the environments
# start_goal_distance_generate_and_save_environments(grid_size, start_states, goal_states)

# Load environments and plot them
# envs = start_goal_distance_load_environments()
# start_goal_distance_plot_environment(envs)
