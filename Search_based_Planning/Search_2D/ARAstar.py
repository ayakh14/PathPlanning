"""
ARA_star 2D (Anytime Repairing A*)
@author: huiming zhou

@description: local inconsistency: g-value decreased.
g(s) decreased introduces a local inconsistency between s and its successors.

"""

import os
import sys
import math
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




class AraStar:
    def __init__(self, s_start, s_goal, e, heuristic_type, environment):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = environment                                                # class Env

        self.u_set = self.Env.motions                                       # feasible input set
        self.obs = self.Env.obs                                             # position of obstacles
        self.e = e                                                          # weight

        self.g = dict()                                                     # Cost to come
        self.OPEN = dict()                                                  # priority queue / OPEN set
        self.CLOSED = set()                                                 # CLOSED set
        self.INCONS = {}                                                    # INCONSISTENT set
        self.PARENT = dict()                                                # relations
        self.path = []                                                      # planning path
        self.visited = [] 
        
        self.searches = 0   #added
        self.process = psutil.Process()
        self.memory_usage_before = None
        self.memory_usage_after = None                                                  # order of visited nodes

    def init(self):
        """
        initialize each set.
        """

        self.g[self.s_start] = 0.0
        self.g[self.s_goal] = math.inf
        self.OPEN[self.s_start] = self.f_value(self.s_start)
        self.PARENT[self.s_start] = self.s_start

    def searching(self):

        self.init()
        self.ImprovePath()
        self.path.append(self.extract_path())
        self.searches += 1  #added                                               # Increment count of searches made

        while self.update_e() > 1:                                          # continue condition
            self.e -= 0.4                                                   # increase weight
            self.OPEN.update(self.INCONS)
            self.OPEN = {s: self.f_value(s) for s in self.OPEN}             # update f_value of OPEN set

            self.INCONS = dict()
            self.CLOSED = set()
            self.ImprovePath()                                              # improve path
            self.path.append(self.extract_path())

            self.searches += 1    # Added line - Increment the counter after each ImprovePath call
        return self.path, self.visited

    def ImprovePath(self):
        """
        :return: a e'-suboptimal path
        """

        visited_each = []

        while True:
            s, f_small = self.calc_smallest_f()

            if self.f_value(self.s_goal) <= f_small:
                break

            self.OPEN.pop(s)
            self.CLOSED.add(s)

            for s_n in self.get_neighbor(s):
                if s_n in self.obs:
                    continue

                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g or new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    visited_each.append(s_n)

                    if s_n not in self.CLOSED:
                        self.OPEN[s_n] = self.f_value(s_n)
                    else:
                        self.INCONS[s_n] = 0.0

        self.visited.append(visited_each)

    def calc_smallest_f(self):
        """
        :return: node with smallest f_value in OPEN set.
        """

        s_small = min(self.OPEN, key=self.OPEN.get)

        return s_small, self.OPEN[s_small]

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return {(s[0] + u[0], s[1] + u[1]) for u in self.u_set}

    def update_e(self):
        v = float("inf")

        if self.OPEN:
            v = min(self.g[s] + self.h(s) for s in self.OPEN)
        if self.INCONS:
            v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))

        return min(self.e, self.g[self.s_goal] / v)

    def f_value(self, x):
        """
        f = g + e * h
        f = cost-to-come + weight * cost-to-go
        :param x: current state
        :return: f_value
        """

        return self.g[x] + self.e * self.h(x)

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = self.PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def h(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type                                # heuristic type
        goal = self.s_goal                                                  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

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
        s_start = tuple(s_start)
        s_end = tuple(s_end)
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

    def calculate_path_cost(self):
            
            """
            Calculating the total cost of the final path.
            :return: Total path cost
            """

            total_cost = 0.0

            if self.path:
                final_path = self.path[-1]  # Final path is the last element
                for i in range(len(final_path) - 1):
                    # print(f"s_start: {final_path[i]}, s_goal: {final_path[i + 1]}")  # Debug line
                    total_cost += self.cost(final_path[i], final_path[i + 1])


            return total_cost
    
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

def ara_star (env, writer,exp, i):
    s_start = env.start
    s_goal = env.goal

    arastar = AraStar(s_start, s_goal, 2.5, "euclidean", env)
    plot = plotting.Plotting(s_start, s_goal, env)
    tracemalloc.start()
 # start measuring time and memory usage at the start of the search
    start_time = time.perf_counter_ns()
    arastar.memory_usage_before = arastar.process.memory_info()
    path, visited = arastar.searching()
    arastar.memory_usage_after = arastar.process.memory_info()
    end_time = time.perf_counter_ns()
    snapshot = tracemalloc.take_snapshot()
    tracemalloc.stop()
    memo_rss = (arastar.memory_usage_after.rss - arastar.memory_usage_before.rss)/ 1024
    memo_vms = (arastar.memory_usage_after.vms - arastar.memory_usage_before.vms)/ 1024
    total = display_top(snapshot, limit=0) / 1024

    print(f"\nGrid {i+1} with N =:")

    path_cost = arastar.calculate_path_cost()
    num_expanded_nodes = sum(len(nodes) for nodes in visited)
    num_searches = arastar.searches
    expanded_nodes_per_lookahead = [len(nodes) for nodes in visited]
    execution_time = (end_time - start_time) / 1e6 


    # Added lines - Print the gathered information
    print(f"Total path cost: {path_cost}")
    print(f"Total number of expanded nodes: {num_expanded_nodes}")
    print(f"Number of searches made to find a solution: {num_searches}")
    print(f"Number of expanded nodes per lookahead (iteration): {expanded_nodes_per_lookahead}")
    print(f"Total allocated memory: {total} KB")
    print(f"Memory consumption RSS: {memo_rss} KB")
    print(f"Memory consumption VMS: {memo_vms} KB")
    print(f"Execution time: {execution_time} ms")

    # plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")
    
    # Write the results into the CSV file
    writer.writerow([exp, i+1, env.obs_density, env.x_range , env.euclidean_distance, "-", path_cost, num_expanded_nodes, num_searches, total, memo_rss, memo_vms,execution_time])
            


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
                ara_star(env, writer, exp, i)
                # Call garbage collector to free up memory
                gc.collect()

    print("the environment has been processed.")
             
def main():
    # Define environment loaders, directories and result files
    env_loaders = [grid_size_env_load_environments, start_goal_distance_load_environments, load_vertical_walls_environments, load_vertical_wall_size_environments, obstacle_density_load_environments]
    directories = ["grid_size_env/results", "start_goal_distance_env/results", "vertical_wall_env/results", "vertical_wall_size_env/results", "obstacle_density_env/results"]
    result_files = ['grid_size_env/results/ARA_star_100_run_results.csv', 'start_goal_distance_env/results/ARA_star_100_run_results.csv', 'vertical_wall_env/results/ARA_star_100_run_results.csv', 'vertical_wall_size_env/results/ARA_star_100_run_results.csv', 'obstacle_density_env/results/ARA_star_100_run_results.csv']
    
    # Process each environment
    for env_loader, directory, result_file in zip(env_loaders, directories, result_files):
        process_env(env_loader, directory, result_file)

    print("All environments have been processed.")

if __name__ == '__main__':
    main()  
    