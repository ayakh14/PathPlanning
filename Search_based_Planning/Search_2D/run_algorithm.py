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
import logging
from metrics_helper import measure_and_log_metrics


# configuring the logger
logging.basicConfig(filename="algorithm_metrics.log", level=logging.INFO, format="%(asctime)s - %(message)s", datefmt="%Y-%m-%d %H:%M:%S")



# run the algorithms based on there name to execute it: python run_algorithm.py ""algo_name""

def run_algorithm(algorithm_name):
    
    # the random environment size, obstacles density, s_start, and s_goal
    x_range = 200
    y_range = 200
    obs_density = 0.2  # 20% of the cells will have obstacles

    random_env = RandomEnv(x_range, y_range, obs_density)
    s_start = random_env.start
    s_goal = random_env.goal
    plot = plotting.Plotting(s_start, s_goal, random_env)


    # all possible algorithms to be used
    if algorithm_name == "ADStar":
        algorithm = ADStar(s_start, s_goal, 2.5, "euclidean", random_env)
        algorithm.run()
    
    
    elif algorithm_name == "AraStar":
        algorithm = AraStar(s_start, s_goal, 2.5, "euclidean", random_env)
        path, visited = algorithm.searching()
        plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")
    
    
    elif algorithm_name == "AStar":
        algorithm = AStar(s_start, s_goal, "euclidean", random_env)
        path, visited = algorithm.searching()
        # plot.animation(path, visited, "A*")
        measure_and_log_metrics("A*", algorithm, path, visited,  plot.animation, random_env)
        print(f"number of expanded nodes: ", len(visited))
    
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
        # path, visited =  algorithm.searching()
        # lrta.searching()
        plot = plotting.Plotting(s_start, s_goal, random_env)
        total_path_length, expanded_nodes_count = algorithm.searching()
        print(f"Total path length: {total_path_length}")
        print(f"Number of expanded nodes: {expanded_nodes_count}")
        plot.animation_lrta(algorithm.path, algorithm.visited,
                            "Learning Real-time A* (LRTA*)")
        
    
    elif algorithm_name == "RTAAStar":
        algorithm = RTAAStar(s_start, s_goal, 240, "euclidean", random_env)
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
