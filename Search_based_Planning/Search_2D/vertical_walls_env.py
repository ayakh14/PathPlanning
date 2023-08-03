

"""
    This script generates grids with a predefined size. However each time a new grid 
    is generated, a wall will be added to it once from the left and once from the right. 
"""
import os
import pickle
import json
import random
from env import Env, VerticalWallEnv
import matplotlib.pyplot as plt

def generate_and_save_environments(directions):
    directory = "vertical_wall_env"
    if not os.path.exists(directory):
        os.makedirs(directory)

    envs = []
    envs_dicts = []

    env = VerticalWallEnv((1, 1), goal_state = None) # Create the object here outside the loop

    for i in range(10, 61, 10):
        direction = directions[i // 10 % 2]
        env.add_wall(i, direction)
        env_copy = pickle.loads(pickle.dumps(env))  # Deep copy of the environment
        envs.append(env_copy)

        env_dict = {
            "start": env.start,
            "goal": env.goal,
            "obs": list(env.obs),
            "wall_y": i,
            "wall_direction": direction,
            "euclidean_distance": env.euclidean_distance
        }
        envs_dicts.append(env_dict)

    try:
        with open(os.path.join(directory, 'vertical_wall_env.pkl'), 'wb') as f:
            pickle.dump(envs, f)

        with open(os.path.join(directory, 'vertical_wall_env.json'), 'w') as f:
            json.dump(envs_dicts, f)
        print(f'Successfully generated and saved environments with varying wall directions.')
    except Exception as e:
        print(f'Failed to save environments. Error: {e}')

 


def load_vertical_walls_environments():
    try:
        with open('vertical_wall_env/vertical_wall_env.pkl', 'rb') as f:
            envs = pickle.load(f)
        return envs
    except FileNotFoundError:
        print("Environments file not found.")
        return None

def plot_vertical_walls_environments(envs):
    if envs is None:
        print("No environments to plot.")
        return

    if isinstance(envs, list):
        num_envs = len(envs)
        num_cols = 2  # change to your preference
        num_rows = num_envs // num_cols if num_envs % num_cols == 0 else num_envs // num_cols + 1

        fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10))  # Adjust the size as needed
        axs = axs.flatten()  # To make indexing easier

        for i, env in enumerate(envs):
            wall_x = [x[0] for x in env.obs]
            wall_y = [x[1] for x in env.obs]
            axs[i].scatter(wall_x, wall_y, s=1, color='black')
            axs[i].plot(env.start[0], env.start[1], "bs", label='Start')
            axs[i].plot(env.goal[0], env.goal[1], "gs", label='Goal')
            axs[i].set_title(f"Wall at y={env.wall_y}, direction={env.wall_direction}, num_walls={env.num_walls}")
            axs[i].legend()

        # If the number of environments is not a perfect square, remove the extra subplots
        for i in range(num_envs, len(axs)):
            fig.delaxes(axs[i])

        plt.tight_layout()
        plt.savefig('vertical_walls_environments.png')  # save the figure to a file
        # plt.show()
    else:
        env = envs
        wall_x = [x[0] for x in env.obs]
        wall_y = [x[1] for x in env.obs]
        plt.scatter(wall_x, wall_y, s=1, color='black')
        plt.plot(env.start[0], env.start[1], "bs", label='Start')
        plt.plot(env.goal[0], env.goal[1], "gs", label='Goal')
        plt.title(f"Wall at y={env.wall_y}, direction={env.wall_direction}")
        plt.legend()
        plt.savefig('vertical_walls_environment.png')  # save the figure to a file
        # plt.show()



# Set the parameters
directions = ['left', 'right']

# # Generate and save the environments
# generate_and_save_environments(directions)

# # Load environments 
# envs = load_vertical_walls_environments()

# # Plot environments
# plot_vertical_walls_environments(envs)
