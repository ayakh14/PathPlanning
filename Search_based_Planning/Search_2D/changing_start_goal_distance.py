"""
    This script generates randomly 6 grids with the a fixed size and obstacle density,
    however the start and goal positions are defined by the user. and for each grid 10 other 
    instances are generated with the same parameters e.i., same start and goal state position
    with obstacles randomly generated. 
"""



import os
import pickle
import json
import random
from grid_env import GridEnv  
import matplotlib.pyplot as plt

def start_goal_distance_generate_and_save_environments(grid_size, start_states, goal_states):
    directory = "start_goal_distance_env"
    if not os.path.exists(directory):
        os.makedirs(directory)

    envs = []
    envs_dicts = []
    obs_density = 0.25  # Define the obstacle density 

    for start_state, goal_state in zip(start_states, goal_states):
        for _ in range(10):  # Generate 10 instances for each start and goal state pair
            while True:
                env = GridEnv(grid_size, grid_size, obs_density, start_state, goal_state)
                euclidean_distance = ((start_state[0] - goal_state[0]) ** 2 + (start_state[1] - goal_state[1]) ** 2) ** 0.5
                env.euclidean_distance = euclidean_distance

                # Import AStar here, inside the function where it's used
                from Astar import AStar
                
                # Run A* on the grid to check if a path exists
                astar = AStar(env.start, env.goal, "euclidean", env)
                path, _ = astar.searching()
                if path is not None:  # A* found a path
                    envs.append(env)

                    env_dict = {
                        "start": env.start,
                        "goal": env.goal,
                        "obs": list(env.obs),
                        "obs_density": obs_density,
                        "euclidean_distance": euclidean_distance
                    }
                    envs_dicts.append(env_dict)
                    break

    try:
        with open(os.path.join(directory, 'start_goal_distance_env.pkl'), 'wb') as f:
            pickle.dump(envs, f)

        with open(os.path.join(directory, 'start_goal_distance_env.json'), 'w') as f:
            json.dump(envs_dicts, f)
        print('Successfully generated and saved environments with varying start and goal states.')
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

    if isinstance(envs, list):
        for i, env in enumerate(envs):
            plt.figure(figsize=(5, 5))
            obs_x = [x[0] for x in env.obs]
            obs_y = [x[1] for x in env.obs]
            plt.scatter(obs_x, obs_y, s=1, color='black')
            plt.plot(env.start[0], env.start[1], "bs", label='Start')
            plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
            plt.title(f"Density: {env.obs_density}, Distance: {env.euclidean_distance:.2f}")
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
        plt.title(f"Density: {env.obs_density}, Distance: {env.euclidean_distance:.2f}")
        plt.legend()
        # plt.show()

# Set the parameters
grid_size = 300
start_states = [(1, 1), (30, 30), (60, 60), (90, 90), (120, 120), (140, 140)]
goal_states = [(298, 298), (270, 270), (240, 240), (210, 210), (180, 180), (160, 160)]

# # Generate and save the environments
# start_goal_distance_generate_and_save_environments(grid_size, start_states, goal_states)

# Load environments and plot them
# envs = start_goal_distance_load_environments()
# # start_goal_distance_plot_environment(envs)
# for env in envs:
#     print(f"Start: {env.start}, Goal: {env.goal}, S/G Distance: {env.euclidean_distance:.2f}")
