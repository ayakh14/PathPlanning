"""
LPA_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt
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


class LPAStar:

    def __init__(self, s_start, s_goal, heuristic_type, env_instance=None):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        if env_instance is None:
            self.Env = env.Env()
        else:
            self.Env = env_instance
        self.Plot = plotting.Plotting(self.s_start, self.s_goal, self.Env)

        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.g, self.rhs, self.U = {}, {}, {}

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_start] = 0
        self.U[self.s_start] = self.CalculateKey(self.s_start)
        self.visited = set()
        self.count = 0

        self.fig = plt.figure()
        
        
        self.expanded_nodes = 0   # counter for the total number of expanded nodes
        self.searches = 0   # counter for the total number of searches
        self.expanded_nodes_per_search = []   # list for tracking the number of expanded nodes per search
        
        self.m1 = 0
        self.m2 = 0
        self.start_time = 0
        self.end_time = 0
        
        self.process = psutil.Process()
        self.memory_usage_before = None
        self.memory_usage_after = None
        
    # def run(self):
                
        
    #     global process, start_time, end_time, m1, m2 
    #     # measuring time at the start
    #     start_time = time.time()
    #     # print(f"mempry with pustil 1 {process.memory_info().rss} --> MB {process.memory_info().rss/1024/1024}")  # in bytes 
    #     m1 = process.memory_info().rss
        
    #     self.Plot.plot_grid("Lifelong Planning A*")
    #     self.ComputeShortestPath()
    #     self.plot_path(self.extract_path())
    #     self.ComputeShortestPath()
    #     self.plot_path(self.extract_path())
       
    #     m2 = process.memory_info().rss

    #     total_path_cost = self.get_total_path_cost()
    #     # End measuring time
    #     end_time = time.time()
    #     print("Total path cost:", total_path_cost)    # print the total path cost
    #     print("Total number of expanded nodes:", self.expanded_nodes)   # print the total number of expanded nodes
    #     print("Number of searches made to find a solution:", self.searches)   # print the number of searches
    #     print("Number of expanded nodes per search:", self.expanded_nodes_per_search)   # print the number of expanded nodes per search
    #     print(f"Total memory consumption {(m2 - m1)/1024/1024} MB")
    #     print(f"Execution time: {end_time - start_time} seconds")
    #     self.fig.canvas.mpl_connect('button_press_event', self.on_press)
    #     plt.show()
    def run(self):

        self.Plot.plot_grid("Lifelong Planning A*")
        self.ComputeShortestPath()
        self.plot_path(self.extract_path())

        self.ComputeShortestPath()
        self.plot_path(self.extract_path())

        total_path_cost = self.get_total_path_cost()
        # End measuring time
        # print("Total path cost:", total_path_cost)    # print the total path cost
        # print("Total number of expanded nodes:", self.expanded_nodes)   # print the total number of expanded nodes
        # print("Number of searches made to find a solution:", self.searches)   # print the number of searches
        # print("Number of expanded nodes per search:", self.expanded_nodes_per_search)   # print the number of expanded nodes per search
        # print(f"Total memory consumption {(self.m2 - self.m1)/1024/1024} MB")
        # print(f"Execution time: {self.end_time - self.start_time} seconds")
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        # plt.show()


    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)

            self.visited = set()
            self.count += 1

            if (x, y) not in self.obs:
                self.obs.add((x, y))
            else:
                self.obs.remove((x, y))
                self.UpdateVertex((x, y))

            self.Plot.update_obs(self.obs)

            for s_n in self.get_neighbor((x, y)):
                self.UpdateVertex(s_n)

            self.ComputeShortestPath()

            plt.cla()
            self.Plot.plot_grid("Lifelong Planning A*")
            self.plot_visited(self.visited)
            self.plot_path(self.extract_path())
            self.fig.canvas.draw_idle()
            # Print the information after the modification
            print("Total path cost:", self.get_total_path_cost())    # print the total path cost
            print("Total number of expanded nodes:", self.expanded_nodes)   # print the total number of expanded nodes
            print("Number of searches made to find a solution:", self.searches)   # print the number of searches
            print("Number of expanded nodes per search:", self.expanded_nodes_per_search)   # print the number of expanded nodes per search
    
    
    def ComputeShortestPath(self):
        self.searches += 1
        expanded_nodes_this_search = 0
        while True:
            s, v = self.TopKey()

            if v >= self.CalculateKey(self.s_goal) and \
                    self.rhs[self.s_goal] == self.g[self.s_goal]:
                break

            self.U.pop(s)
            self.visited.add(s)
            self.expanded_nodes += 1
            expanded_nodes_this_search += 1


            if self.g[s] > self.rhs[s]:

                # Condition: over-consistent (eg: deleted obstacles)
                # So, rhs[s] decreased -- > rhs[s] < g[s]
                self.g[s] = self.rhs[s]
            else:

                # Condition: # under-consistent (eg: added obstacles)
                # So, rhs[s] increased --> rhs[s] > g[s]
                self.g[s] = float("inf")
                self.UpdateVertex(s)

            for s_n in self.get_neighbor(s):
                self.UpdateVertex(s_n)
        
        
        self.expanded_nodes_per_search.append(expanded_nodes_this_search)
        if self.g[self.s_goal] == float("inf"):
            print("No path found to the goal.")

    def UpdateVertex(self, s):
        """
        update the status and the current cost to come of state s.
        :param s: state s
        """

        if s != self.s_start:

            # Condition: cost of parent of s changed
            # Since we do not record the children of a state, we need to enumerate its neighbors
            self.rhs[s] = min(self.g[s_n] + self.cost(s_n, s)
                              for s_n in self.get_neighbor(s))

        if s in self.U:
            self.U.pop(s)

        if self.g[s] != self.rhs[s]:

            # Condition: current cost to come is different to that of last time
            # state s should be added into OPEN set (set U)
            self.U[s] = self.CalculateKey(s)

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.U, key=self.U.get)

        return s, self.U[s]

    def CalculateKey(self, s):

        return [min(self.g[s], self.rhs[s]) + self.h(s),
                min(self.g[s], self.rhs[s])]

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                s_list.add(s_next)

        return s_list

    def h(self, s):
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

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
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

    def extract_path(self):

        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        for k in range(1000):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_start:
                break
        
        return list(reversed(path))

    def plot_path(self, path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, linewidth=2)
        plt.plot(self.s_start[0], self.s_start[1], "bs")
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])

    def get_total_path_cost(self):
        path = self.extract_path()
        total_cost = 0
        for i in range(len(path) - 1):
            total_cost += self.cost(path[i], path[i+1])
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

def lpa_algo(env, writer,exp, i):

    s_start = env.start
    s_goal = env.goal

    lpastar = LPAStar(s_start, s_goal, "Euclidean", env)
    tracemalloc.start()
    # start measuring time and memory usage at the start of the search
    start_time = time.perf_counter_ns()
    lpastar.memory_usage_before = lpastar.process.memory_info()
    lpastar.run()
    lpastar.memory_usage_after = lpastar.process.memory_info()
    end_time = time.perf_counter_ns()
    snapshot = tracemalloc.take_snapshot()
    tracemalloc.stop()
    memo_rss = (lpastar.memory_usage_after.rss - lpastar.memory_usage_before.rss)/ 1024
    memo_vms = (lpastar.memory_usage_after.vms - lpastar.memory_usage_before.vms)/ 1024
    total = display_top(snapshot, limit=0) / 1024

    # get results from the Dstar instance
    path_cost = lpastar.get_total_path_cost()
    num_expanded_nodes = lpastar.expanded_nodes
    num_searches = lpastar.searches
    # expanded_nodes_per_lookahead = LPAStar.expanded_nodes_per_search
    # memory_consumption = (lpastar.m2 - lpastar.m1)/1024/1024
    execution_time = (end_time - start_time) / 1e6 
    writer.writerow([exp, i+1, env.obs_density, env.x_range , env.euclidean_distance, "-", path_cost, num_expanded_nodes, num_searches, total, memo_rss, memo_vms,execution_time])
    plt.close(lpastar.fig)
    # Call garbage collector to free up memory


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
                lpa_algo(env, writer,exp, i)
                gc.collect()
    
    print("the environment has been processed.")

def main():
    # Define environment loaders, directories and result files
    env_loaders = [grid_size_env_load_environments, start_goal_distance_load_environments, load_vertical_walls_environments, load_vertical_wall_size_environments, obstacle_density_load_environments]
    directories = ["grid_size_env/results", "start_goal_distance_env/results", "vertical_wall_env/results", "vertical_wall_size_env/results", "obstacle_density_env/results"]
    result_files = ['grid_size_env/results/LPA_star_100_run_results.csv', 'start_goal_distance_env/results/LPA_star_100_run_results.csv', 'vertical_wall_env/results/LPA_star_100_run_results.csv', 'vertical_wall_size_env/results/LPA_star_100_run_results.csv', 'obstacle_density_env/results/LPA_star_100_run_results.csv']
    
    # Process each environment
    for env_loader, directory, result_file in zip(env_loaders, directories, result_files):
        process_env(env_loader, directory, result_file)

    print("All environments have been processed.")

if __name__ == '__main__':
    main()  