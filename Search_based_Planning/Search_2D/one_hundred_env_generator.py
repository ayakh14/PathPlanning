"""

    The script generates 100 random grids with a specific width and highet,
    25% obstacles, a s_start and g_state that were chosen randomly.
    all grids are saves in pickle and a jason file, so that is would be easy
    to use the same grids for each algorithm for a fair comparision.

    if anything went wrong during the creation of the grids a detailed error
    message will be printed
 
"""


import json
import pickle
import random
from random_env import RandomEnv  # Make sure to adjust the import as per your project structure
import matplotlib.pyplot as plt
import os 
def generate_and_save_environments(num_envs, x_range, y_range, obs_density):

    # Creating folder if it doesn't exist
    directory = "one_hundred_random_grids"
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
            print(f'Successfully created directory {directory}')
        except Exception as e:
            print(f'Failed to create directory {directory}. Error: {e}')
            return

    envs = []
    envs_dicts = []
    
    for i in range(num_envs):
        try:
            # Create a new environment
            env = RandomEnv(x_range, y_range, obs_density)
            
            # Make sure the start and goal states are different
            while env.start == env.goal:
                env.start = env.random_state()
                env.goal = env.random_state()

            # Add the environment to the list
            envs.append(env)

            # Convert the environment to a dictionary
            env_dict = {
                "start": env.start,
                "goal": env.goal,
                "obs": list(env.obs)
            }

            # Add the environment dictionary to the list
            envs_dicts.append(env_dict)
            print(f'Successfully created environment {i+1}')
        except Exception as e:
            print(f'Failed to create environment {i+1}. Error: {e}')
    
    try:

        # Saving the environments using pickle
        with open(os.path.join(directory, 'one_hundred_random_environments.pkl'), 'wb') as f:
            pickle.dump(envs, f)

        # Saving the environments using JSON
        with open(os.path.join(directory, 'one_hundred_random_environments.json'), 'w') as f:
            json.dump(envs_dicts, f)

        print(f'Successfully generated and saved {num_envs} environments.')
    
    except Exception as e:
        print(f'Failed to save environments. Error: {e}')



def load_environments():
    with open('one_hundred_random_grids/one_hundred_random_environments.pkl', 'rb') as f:
        envs = pickle.load(f)
    
    return envs



# Now you can access each environment from the list, for example:
# first_env = envs[0]
# print(first_env.start, first_env.goal)  # Prints the start and goal states of the first environment

def plot_environment(env, env_id):
    fig, ax = plt.subplots(figsize=(10, 10))  # increase figure size

    obs_x = [x[0] for x in env.obs]
    obs_y = [x[1] for x in env.obs]
    
    # Plot obstacles
    ax.scatter(obs_x, obs_y, s=1, color='black')  # decrease point size

    # # Plot start and goal states
    ax.plot(env.start[0], env.start[1], "bs", label='Start')
    ax.plot(env.goal[0], env.goal[1], "gs", label='Goal')
    # # same as the above ones but just incase you want to change the size of the s_state and g_state
    # ax.plot(env.start[0], env.start[1], "bs", markersize=10, label='Start')
    # ax.plot(env.goal[0], env.goal[1], "gs", markersize=10, label='Goal')
    
    # Print start and goal states
    print(f"Start state of environment {env_id}: {env.start}")
    print(f"Goal state of environment {env_id}: {env.goal}")
    
    ax.set_title(f"Environment {env_id}")
    ax.axis("equal")
    # Add legend to plot only if you uncomment markersize
    ax.legend()
    plt.show()

  

# # # Generate and save 100 environments with a 50x50 grid and 20% obstacle density
# generate_and_save_environments(100, 301, 301, 0.25)

# # Load the environments
# envs = load_environments()

# # # Plot the first environment
# plot_environment(envs[9],10)
