"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import time
import psutil
import gc
import linecache
import tracemalloc
import csv
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
# from random_env import RandomEnv
# from changing_distance_position_of_s_g import start_goal_distance_load_environments_second
# from one_hundred_env_generator import load_environments

from changing_obstacle_density import obstacle_density_load_environments
from changing_start_goal_distance import start_goal_distance_load_environments
from changing_grid_size import grid_size_env_load_environments
from vertical_walls_env import load_vertical_walls_environments 
from change_wall_size import load_vertical_wall_size_environments

class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type, environment):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.Env = environment  # class Env
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

        self.total_path_cost = 0
        self.total_expanded_nodes = 0
        self.total_searches = 0
        self.expanded_nodes_per_lookahead = []

        self.process = psutil.Process()
        self.memory_usage_before = None
        self.memory_usage_after = None

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """
        # self.memory_usage_before = self.process.memory_info()
        self.total_searches += 1   # Increment total searches here

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))
        
        if self.CLOSED[-1] != self.s_goal:
            print("No solution was found.")
            return None, self.CLOSED
        # Measuring memory usage and time at the end of the search
        return self.extract_path(self.PARENT), self.CLOSED

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)
    def load_environments_and_search():
    #     from changing_obstacle_density import obstacle_density_load_environments
    #     envs = obstacle_density_load_environments()
        # from changing_grid_size import grid_size_env_load_environments
        # envs = grid_size_env_load_environments()
        from changing_start_goal_distance import start_goal_distance_load_environments
        envs = start_goal_distance_load_environments()

        # Continue with your code

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])




def display_top(snapshot, key_type='lineno', limit=20):
    print(tracemalloc.__file__)
    snapshot = snapshot.filter_traces((
        #tracemalloc.Filter(False, tracemalloc.__file__),
        #tracemalloc.Filter(False, linecache.__file__),
        tracemalloc.Filter(True, __file__),
    ))
    top_stats = snapshot.statistics(key_type)

    print("Top %s lines" % limit)
    for index, stat in enumerate(top_stats[:limit], 1):
        frame = stat.traceback[0]
        print("#%s: %s:%s: %.1f KiB"
              % (index, frame.filename, frame.lineno, stat.size / 1024))
        line = linecache.getline(frame.filename, frame.lineno).strip()
        if line:
            print('    %s' % line)

    other = top_stats[limit:]
    if other:
        size = sum(stat.size for stat in other)
        print("%s other: %.1f KiB" % (len(other), size / 1024))
    total = sum(stat.size for stat in top_stats)
    print("Total allocated size: %.1f KiB" % (total / 1024))
    return total

def start_astar(env, writer,exp, i):
    s_start = env.start
    s_goal = env.goal        

    astar = AStar(s_start, s_goal, "euclidean", env)
    plot = plotting.Plotting(s_start, s_goal, env)
    
    tracemalloc.start()
    
    # start measuring time and memory usage at the start of the search
    start_time = time.perf_counter_ns()
    astar.memory_usage_before = astar.process.memory_info()
    path, visited = astar.searching()
    astar.memory_usage_after = astar.process.memory_info()
    end_time = time.perf_counter_ns()
    snapshot = tracemalloc.take_snapshot()
    tracemalloc.stop()
    memo_rss = (astar.memory_usage_after.rss - astar.memory_usage_before.rss)/ 1024
    memo_vms = (astar.memory_usage_after.vms - astar.memory_usage_before.vms)/ 1024
    total = display_top(snapshot, limit=0) / 1024
    # plot.animation(path, visited, "A*")  # animation
    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")

    total_expanded_nodes = sum(len(nodes) for nodes in visited)
    nodes_per_search = [len(nodes) for nodes in visited]
    total_path_cost = sum(astar.cost(path[i], path[i-1]) for i in range(1, len(path))) # Calculate total path cost
    execution_time = (end_time - start_time) / 1e6 
    
    # Print the gathered information
    print(f"Total path cost: {total_path_cost}")
    print(f"Total number of expanded nodes: {total_expanded_nodes}")
    print(f"Number of searches made to find a solution: {astar.total_searches}")
    # print(f"Number of expanded nodes per lookahead (iteration): {nodes_per_search}")
    print(f"Total allocated memory: {total} KB")
    print(f"Memory consumption RSS: {memo_rss} KB")
    print(f"Memory consumption VMS: {memo_vms} KB")
    print(f"Execution time: {execution_time} ms")
    writer.writerow([exp, i+1, env.obs_density, env.x_range , env.euclidean_distance, "-", total_path_cost, total_expanded_nodes, astar.total_searches, total, memo_rss, memo_vms,execution_time])
    print("the environment has been processed.")


# Function to process a specific environment
def process_env(env_loader, directory, result_file):
    # Create the directory if it doesn't exist
    if not os.path.exists(directory):
        os.makedirs(directory)
    # Print the directory being processed
    print(f"\n Processing environment: {directory}")
    # Load environments
    envs = env_loader()

    with open(result_file, 'w', newline='') as file:
        writer = csv.writer(file)
        
        # Write the header of the CSV file
        writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])
        for i, env in enumerate(envs):
    
            print(f"Running algorithm on grid {i+1}, s_state {env.start}, g_state {env.goal}, env_size {env.x_range}, obs_dancity {env.obs_density:.2f}, s/g distance {env.euclidean_distance} ")
            
            for exp in range(1, 101):
                start_astar(env, writer, exp, i)
                # Call garbage collector to free up memory
                gc.collect()
                
    print("the environment has been processed.")
 
def main():
    # Define environment loaders, directories and result files
    env_loaders = [grid_size_env_load_environments, start_goal_distance_load_environments, load_vertical_walls_environments, load_vertical_wall_size_environments, obstacle_density_load_environments]
    directories = ["grid_size_env/results", "start_goal_distance_env/results", "vertical_wall_env/results", "vertical_wall_size_env/results", "obstacle_density_env/results"]
    result_files = ['grid_size_env/results/A_star_100tes_run_results.csv', 'start_goal_distance_env/results/A_star_100tes_run_results.csv', 'vertical_wall_env/results/A_star_100tes_run_results.csv', 'vertical_wall_size_env/results/A_star_100tes_run_results.csv', 'obstacle_density_env/results/A_star_100tes_run_results.csv']


    # # Define environment loaders, directories and result files
    # env_loaders = [start_goal_distance_load_environments]
    # directories = ["start_goal_distance_env/results"]
    # result_files = ['start_goal_distance_env/results/A_star_resultsffffff.csv']
  

    # Process each environment
    for env_loader, directory, result_file in zip(env_loaders, directories, result_files):
        process_env(env_loader, directory, result_file)

    print("All environments have been processed.")


if __name__ == '__main__':
    main()