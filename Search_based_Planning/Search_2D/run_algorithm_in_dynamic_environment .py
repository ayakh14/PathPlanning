"""
to run algorithms in random environment

"""



import sys
import plotting
from random_env import RandomEnv
from Anytime_D_star import ADStar
from ARAstar import AraStar
from Astar import AStar
from D_star_Lite import DStar as DStarLite
from D_star import DStar
from LPAstar import LPAStar
from LRTAstar import LrtAStarN
from RTAAStar import RTAAStar
import threading
import time


def add_obstacles_periodically(environment, delay):
    while True:
        environment.add_random_obstacle()
        time.sleep(delay)


def run_algorithm(algorithm_name):
    x_range = 100
    y_range = 40
    obs_density = 0.2  # 20% of the cells will have obstacles

    random_env = RandomEnv(x_range, y_range, obs_density)
    s_start = random_env.start
    s_goal = random_env.goal
    plot = plotting.Plotting(s_start, s_goal, random_env)

    # Create a separate thread to add obstacles periodically
    obstacle_thread = threading.Thread(target=add_obstacles_periodically, args=(random_env, 5)) # Add an obstacle every 5 seconds
    obstacle_thread.start()


    
    if algorithm_name == "ADStar":
        algorithm = ADStar(s_start, s_goal, 2.5, "euclidean", random_env)
        algorithm.run()
    elif algorithm_name == "AraStar":
        algorithm = AraStar(s_start, s_goal, 2.5, "euclidean", random_env)
        path, visited = algorithm.searching()
        plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")
    elif algorithm_name == "AStar":
        algorithm = AStar(s_start, s_goal, "euclidean", random_env)
        start_time = time.time()
        path, visited = algorithm.searching()
        end_time = time.time()
        execution_time = end_time - start_time
        plot.animation(path, visited, "A*")
    elif algorithm_name == "DStarLite":
        algorithm = DStarLite(s_start, s_goal, "euclidean", random_env)
        algorithm.run()
    elif algorithm_name == "DStar":
        algorithm = DStar(s_start, s_goal, random_env)
        algorithm.run()
    elif algorithm_name == "LPAStar":
        algorithm = LPAStar(s_start, s_goal, "Euclidean", random_env)
        algorithm.run()
    elif algorithm_name == "LrtAStarN":
        algorithm = LrtAStarN(s_start, s_goal, 250, "euclidean", random_env)
        algorithm.searching()
        plot.animation_lrta(algorithm.path, algorithm.visited,
                            "Learning Real-time A* (LRTA*)")
    elif algorithm_name == "RTAAStar":
        algorithm = RTAAStar(s_start, s_goal, 5, "euclidean", random_env)
        algorithm.searching()
        plot.animation_lrta(algorithm.path, algorithm.visited,
                            "Real-time Adaptive A* (RTAA*)")
    else:
        print(f"Invalid algorithm name: {algorithm_name}")


def main():
    if len(sys.argv) != 2:
        print("Usage: python run_algorithm.py <algorithm_name>")
        sys.exit(1)

    algorithm_name = sys.argv[1]
    run_algorithm(algorithm_name)

if __name__ == "__main__":
    main()



