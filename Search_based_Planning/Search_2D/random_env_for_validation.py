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
import math

def generate_base_environment():

    """ Generate the initial random environment. """
    x_range = y_range = random.choice(range(50, 301, 50))
    obs_density = random.choice([i/100 for i in range(10, 46, 5)])
    env = RandomEnv(x_range, y_range, obs_density)
    distance = set_start_goal(env, x_range, y_range)
    return env, x_range, y_range, obs_density, distance

def set_start_goal(env, x_range, y_range):
    """ Set random start and goal states within the desired distance range. """
    max_possible_distance = math.sqrt(x_range**2 + y_range**2)
    min_distance = 28
    max_distance = min(420, max_possible_distance)
    
    distance = 0
    iterations = 0
    max_iterations = 10000

    while not (min_distance <= distance <= max_distance) and iterations < max_iterations:
        env.start = env.random_state()
        env.goal = env.random_state()
        
        # Ensure start and goal are not obstacles, if they are, remove them from obstacles.
        if env.start in env.obs:
            env.obs.remove(env.start)
        if env.goal in env.obs:
            env.obs.remove(env.goal)
        
        distance = math.sqrt((env.start[0] - env.goal[0])**2 + (env.start[1] - env.goal[1])**2)
        iterations += 1

    if iterations == max_iterations:
        print("Couldn't find a valid start-goal pair after max attempts.")
        return None
    
    return distance
def set_distance_fixed_start_goal(env, start, goal):
    """ Set the start and goal states of env to the specified values. """
    env.start = start
    env.goal = goal

def generate_and_save_environments():
    directory = "one_hundred_random_grids"
    if not os.path.exists(directory):
        os.makedirs(directory)

    base_env, x_range, y_range, obs_density, distance = generate_base_environment()

    envs = [base_env]  # For pickle
    envs_dicts = []  # For JSON

    # Print base environment details
    print_environment_details("Base Environment", 0, base_env, x_range, y_range, obs_density, distance)

    # Convert base_env to dictionary format and append to envs_dicts
    envs_dicts.append({
        "grid_size": f"{x_range}x{y_range}",
        "start": base_env.start,
        "goal": base_env.goal,
        "obs": list(base_env.obs),
        "obs_density": obs_density,
        "euclidean_distance": distance
    })

    params_to_change = ['size', 'density', 'distance']
    
    for param in params_to_change:
        x_range_temp, y_range_temp = x_range, y_range
        obs_density_temp = obs_density
        
        if param == 'size':
            possible_sizes = [size for size in range(50, 301, 50) if size > x_range]
            if not possible_sizes:  # If no larger sizes are available
                continue
            x_range_temp = y_range_temp = random.choice(possible_sizes)

        elif param == 'density':
            obs_density_temp = random.choice([i/100 for i in range(10, 46, 5)])
        
        env = RandomEnv(x_range_temp, y_range_temp, obs_density_temp)
        
        if param != 'distance':
            set_distance_fixed_start_goal(env, base_env.start, base_env.goal)
            distance = math.sqrt((env.start[0] - env.goal[0])**2 + (env.start[1] - env.goal[1])**2)
        else:
            distance = set_start_goal(env, x_range_temp, y_range_temp)

        envs.append(env)

        # Print environment details
        print_environment_details(f"Environment {len(envs)}", len(envs)-1, env, x_range_temp, y_range_temp, obs_density_temp, distance)

        envs_dicts.append({
            "grid_size": f"{x_range_temp}x{y_range_temp}",
            "start": env.start,
            "goal": env.goal,
            "obs": list(env.obs),
            "obs_density": obs_density_temp,
            "euclidean_distance": distance
        })

    try:
        with open(os.path.join(directory, 'random_environments2.pkl'), 'wb') as f:
            pickle.dump(envs, f)
        
        with open(os.path.join(directory, 'random_environments2.json'), 'w') as f:
            json.dump(envs_dicts, f)
        print(f"Successfully generated and saved {len(envs)} environments in '{directory}'.")
    except Exception as e:
        print(f"Failed to save environments in '{directory}'. Error: {e}")


def print_environment_details(title, env_id, env, x_range, y_range, obs_density, distance):
    print(title + ":")
    print(f"Grid Size: {x_range}x{y_range}")
    print(f"Obstacle Density: {obs_density*100}%")
    print(f"Start: {env.start}")
    print(f"Goal: {env.goal}")
    print(f"Distance between Start and Goal: {distance:.2f} units")
    print("--------------------")


def load_environments():
    
    with open('one_hundred_random_grids/random_environments2.pkl', 'rb') as f:
        envs = pickle.load(f)
    return envs



def plot_environment(env):
    fig, ax = plt.subplots(figsize=(10, 10))  # increase figure size

    obs_x = [x[0] for x in env.obs]
    obs_y = [x[1] for x in env.obs]
    
    # Plot obstacles
    ax.scatter(obs_x, obs_y, s=1, color='black')  # decrease point size

    # Plot start and goal states
    ax.plot(env.start[0], env.start[1], "bs", label='Start')
    ax.plot(env.goal[0], env.goal[1], "gs", label='Goal')
    
    # Uncomment below if you want to change the size of the start and goal states
    # ax.plot(env.start[0], env.start[1], "bs", markersize=10, label='Start')
    # ax.plot(env.goal[0], env.goal[1], "gs", markersize=10, label='Goal')
    
    # Print start and goal states
    print(f"Start state: {env.start}")
    print(f"Goal state: {env.goal}")
    
    ax.set_title("Environment Plot")
    ax.axis("equal")
    # Add legend to plot only if you uncomment markersize
    ax.legend()
    plt.show()




# def generate_wall_environment():
#     """ Generate an environment with wall-like obstacles. """
#     x_range = y_range = 301
#     num_walls = random.randint(5, 15)  # choose a random number of walls, say between 5 and 15

#     env = RandomEnv(x_range, y_range, 0)  # 0 obstacle density, since we'll manually add the walls

#     for _ in range(num_walls):
#         orientation = random.choice(['horizontal', 'vertical'])
#         if orientation == 'horizontal':
#             start_x = random.randint(0, x_range-1)
#             end_x = random.randint(start_x, x_range-1)
#             y = random.randint(0, y_range-1)
#             for x in range(start_x, end_x+1):
#                 env.obs.add((x, y))
#         else:  # vertical
#             start_y = random.randint(0, y_range-1)
#             end_y = random.randint(start_y, y_range-1)
#             x = random.randint(0, x_range-1)
#             for y in range(start_y, end_y+1):
#                 env.obs.add((x, y))

#     distance = set_start_goal(env, x_range, y_range)

#     return env, x_range, y_range, distance
  

# # Generate and save environments as per your code
# generate_and_save_environments()

# # Load the environments
# envs = load_environments()

# # Plot each environment
# for i, env in enumerate(envs):
#     plot_environment(env, i+1)


