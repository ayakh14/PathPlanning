import random
import os
import sys
from matplotlib import pyplot as plt
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../Search_based_Planning/")

from Search_2D.changing_obstacle_density import (obstacle_density_load_environments, obstacle_density_plot_environment)


from Search_2D.Astar import AStar
from Search_2D.D_star import DStar
from Search_2D.D_star_Lite import DStarLite
from Search_2D.LPAstar import LPAStar
from Search_2D.LRTAstar import LrtAStarN
from Search_2D.RTAAStar import RTAAStar

def main():
    # Load all environments
    environments = {
        "obstacle_density": obstacle_density_load_environments()
    }

    # Randomly select one environment
    env_type = random.choice(list(environments.keys()))
    env = random.choice(environments[env_type])

    # Display the selected environment
    plt.imshow(env.grid)
    plt.title(f"Environment: {env_type}")
    plt.show()

    # # Select the best algorithm for the environment
    # best_algo = select_algorithm(env.grid, env.start, env.goal)
    # print(f"Best algorithm for this environment is: {best_algo}")

    # # Create an instance of each algorithm
    # astar = AStar(env.grid, env.start, env.goal)
    # dstar = DStar(env.grid, env.start, env.goal)
    # dstarlite = DStarLite(env.grid, env.start, env.goal)
    # lpastar = LPAStar(env.grid, env.start, env.goal)
    # lrta_star = LrtAStarN(env.grid, env.start, env.goal)
    # rtaa_star = RTAAStar(env.grid, env.start, env.goal)

    # # Run each algorithm and measure execution time
    # algorithms = [astar, dstar, dstarlite, lpastar, lrta_star, rtaa_star]
    # for algo in algorithms:
    #     time, path = algo.run()  # Modify this line based on the actual function used to run your algorithms
    #     print(f"Algorithm {type(algo).__name__} took {time} ms to find a path.")

if __name__ == "__main__":
    main()




# import random
# import os
# import sys
# from matplotlib import pyplot as plt
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../../Search_based_Planning/")
# import random
# from Search_2D.Astar import AStar
# from D_star import DStar
# from D_star_Lite import DStarLite
# from LPAstar import LPAStar
# from LRTAstar import LrtAStarN
# from RTAAStar import RTAAStar
# from Search_2D.changing_obstacle_density import obstacle_density_load_environments


# # Define the function for selecting the most suitable algorithm
# def select_algorithm(env):
#     grid_size = env.x_range * env.y_range
#     obstacle_density = env.obs_density
#     start_goal_distance = env.euclidean_distance

#     # Based on the thresholds and results of the previous experiments, select the most suitable algorithm
#     if grid_size < 50 or obstacle_density < 0.3 or start_goal_distance < 200:
#         return RTAAStar
#     elif 50 <= grid_size <= 300 or 0.3 <= obstacle_density <= 0.7 or 200 <= start_goal_distance <= 500:
#         return DStarLite
#     else:
#         return AStar


# # Load the environments
# environments = obstacle_density_load_environments()

# # Select a random environment
# selected_env = random.choice(environments)

# # Print some characteristics of the selected environment
# print(f"Selected environment characteristics:\nSize: {selected_env.x_range * selected_env.y_range}, "
#       f"Obstacle density: {selected_env.obs_density}, Start-goal distance: {selected_env.euclidean_distance}")

# # Select the most suitable algorithm for the chosen environment
# SelectedAlgorithm = select_algorithm(selected_env)

# # Run the selected algorithm and measure its performance
# selected_algo = SelectedAlgorithm(selected_env.start, selected_env.goal, "euclidean", selected_env)
# path, cost = selected_algo.searching()
# print(f"Selected Algorithm: {SelectedAlgorithm.__name__}\nPath: {path}\nCost: {cost}")

# # Also run the other algorithms and measure their performance for comparison
# algorithms = [AStar, DStar, DStarLite, LPAStar, LrtAStarN, RTAAStar]
# for Algorithm in algorithms:
#     if Algorithm is not SelectedAlgorithm:  # We have already run the selected algorithm
#         algo = Algorithm(selected_env.start, selected_env.goal, "euclidean", selected_env)
#         path, cost = algo.searching()
#         print(f"\nAlgorithm: {Algorithm.__name__}\nPath: {path}\nCost: {cost}")
