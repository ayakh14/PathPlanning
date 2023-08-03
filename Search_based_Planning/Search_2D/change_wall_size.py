

"""
    This script generates grids with a predefined size with walls placed 
    in oposit directions every 10 unit. Howwver, each time a new grid is generated
    all walls length will be increased by 2 units (obstacles). 
"""

import os
import pickle
import json
from matplotlib import pyplot as plt
import math

from env import Env

def generate_and_save_environments(directions):
    directory = "vertical_wall_size_env"
    if not os.path.exists(directory):
        os.makedirs(directory)

    envs = []
    envs_dicts = []

    for wall_size in range(0, 14, 2):
        env = Env()
        for idx, y in enumerate(range(10, 61, 10)):
            direction = directions[idx % 2]
            env.add_wall(y, direction, wall_size)
        env_copy = pickle.loads(pickle.dumps(env))
        envs.append(env_copy)

        env_dict = {
            "start": env.start,
            "goal": env.goal,
            "obs": list(env.obs),
            "num_walls": env.num_walls,
            "wall_length": env.wall_length,
            "wall_y": env.wall_y,
            "wall_direction": env.wall_direction,
            "euclidean_distance":env.euclidean_distance
        }
        envs_dicts.append(env_dict)

    try:
        with open(os.path.join(directory, 'vertical_wall_size_env.pkl'), 'wb') as f:
            pickle.dump(envs, f)

        with open(os.path.join(directory, 'vertical_wall_size_env.json'), 'w') as f:
            json.dump(envs_dicts, f)
        print(f'Successfully generated and saved environments with varying wall directions.')
    except Exception as e:
        print(f'Failed to save environments. Error: {e}')


def load_vertical_wall_size_environments():
    try:
        with open('vertical_wall_size_env/vertical_wall_size_env.pkl', 'rb') as f:
            envs = pickle.load(f)
        return envs
    except FileNotFoundError:
        print("Environments file not found.")
        return None

import math

def plot_environments(envs):
    if envs is None:
        print("No environments to plot.")
        return

    if isinstance(envs, list):
        num_envs = len(envs)
        cols = round(math.sqrt(num_envs))
        rows = cols if cols * cols >= num_envs else cols + 1

        fig, axs = plt.subplots(rows, cols, figsize=(15, 15)) # set your figure size

        for i, ax in enumerate(axs.flat):
            if i < num_envs:
                env = envs[i]
                wall_x = [x[0] for x in env.obs]
                wall_y = [x[1] for x in env.obs]
                ax.scatter(wall_x, wall_y, s=1, color='black')
                ax.plot(env.start[0], env.start[1], "bs", label='Start')
                ax.plot(env.goal[0], env.goal[1], "gs", label='Goal')
                ax.set_title(f"Wall at y={env.wall_y}, length={env.wall_length}, num_walls={env.num_walls}, s/g distance={env.euclidean_distance:.2f}")
                ax.legend()
            else:
                ax.axis('off')  # hide axes if there's no environment

        plt.tight_layout()
        plt.savefig('wall_length_environments.png')  # save the figure as PNG
        # plt.show()
    else:
        env = envs
        wall_x = [x[0] for x in env.obs]
        wall_y = [x[1] for x in env.obs]
        plt.scatter(wall_x, wall_y, s=1, color='black')
        plt.plot(env.start[0], env.start[1], "bs", label='Start')
        plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
        plt.title(f"Wall at y={env.wall_y}, length={env.wall_length}, num_walls={env.num_walls}, s/g distance={env.euclidean_distance:.2f}")
        plt.legend()
        plt.savefig('wall_length_environment.png')  # save the figure as PNG
        # plt.show()


# Set the parameters
directions = ['left', 'right']

# # Generate and save the environments
# generate_and_save_environments(directions)

# # Load environments 
# envs = load_vertical_wall_size_environments()

# # Plot environments
# plot_environments(envs)
