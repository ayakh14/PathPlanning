"""
RTAAstar 2D (Real-time Adaptive A*)
@author: huiming zhou
"""

import os
import sys
import copy
import math
import time
import psutil
import gc
import linecache
import tracemalloc
import csv
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import queue_, plotting, env

# from random_env import RandomEnv
# from changing_distance_position_of_s_g import start_goal_distance_load_environments_second
# from one_hundred_env_generator import load_environments

from changing_obstacle_density import obstacle_density_load_environments
from changing_start_goal_distance import start_goal_distance_load_environments
from changing_grid_size import grid_size_env_load_environments
from vertical_walls_env import load_vertical_walls_environments 
from change_wall_size import load_vertical_wall_size_environments


class RTAAStar:
    def __init__(self, s_start, s_goal, N, heuristic_type, environment):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type
        self.Env = environment
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.N = N  # number of expand nodes each iteration
        self.visited = []  # order of visited nodes in planning
        self.path = []  # path of each iteration
        self.h_table = {}  # h_value table
        self.num_nodes_explored = 0  # initializing hte number of expanded nodes
        self.total_searches = 0  # added
 
        # # Starting measuring time
        # self.start_time = None
        # # End measuring time
        # self.end_time = None

        self.process = psutil.Process()
        self.memory_usage_before = None
        self.memory_usage_after = None    
    
    def init(self):
        """
        initialize the h_value of all nodes in the environment.
        it is a global table.
        """

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.h_table[(i, j)] = self.h((i, j))

    def searching(self):

        # # measuring time at the start
        # self.start_time = time.time()

        self.init()
        s_start = self.s_start  # initialize start node

        while True:
            OPEN, CLOSED, g_table, PARENT = \
                self.Astar(s_start, self.N)

            if OPEN == "FOUND":  # reach the goal node
                self.path.append(CLOSED)
                self.total_searches += 1  # added
                break

            s_next, h_value = self.cal_h_value(OPEN, CLOSED, g_table, PARENT)

            for x in h_value:
                self.h_table[x] = h_value[x]

            s_start, path_k = self.extract_path_in_CLOSE(s_start, s_next, h_value)
            self.path.append(path_k)
            self.total_searches += 1  # added

        # # End measuring time
        # self.end_time = time.time()

    def cal_h_value(self, OPEN, CLOSED, g_table, PARENT):
        v_open = {}
        h_value = {}
        for (_, x) in OPEN.enumerate():
            v_open[x] = g_table[PARENT[x]] + 1 + self.h_table[x]
        s_open = min(v_open, key=v_open.get)
        f_min = v_open[s_open]
        for x in CLOSED:
            h_value[x] = f_min - g_table[x]

        return s_open, h_value

    def iteration(self, CLOSED):
        h_value = {}

        for s in CLOSED:
            h_value[s] = float("inf")  # initialize h_value of CLOSED nodes

        while True:
            h_value_rec = copy.deepcopy(h_value)
            for s in CLOSED:
                h_list = []
                for s_n in self.get_neighbor(s):
                    if s_n not in CLOSED:
                        h_list.append(self.cost(s, s_n) + self.h_table[s_n])
                    else:
                        h_list.append(self.cost(s, s_n) + h_value[s_n])
                h_value[s] = min(h_list)  # update h_value of current node

            if h_value == h_value_rec:  # h_value table converged
                return h_value

    def Astar(self, x_start, N):
        OPEN = queue_.QueuePrior()  # OPEN set
        OPEN.put(x_start, self.h_table[x_start])
        CLOSED = []  # CLOSED set
        g_table = {x_start: 0, self.s_goal: float("inf")}  # Cost to come
        PARENT = {x_start: x_start}  # relations
        count = 0  # counter

        while not OPEN.empty():
            count += 1
            s = OPEN.get()
            CLOSED.append(s)
            self.num_nodes_explored += 1 #increament the number of expanded nodes when a node is added in CLOSED
            if s == self.s_goal:  # reach the goal node
                self.visited.append(CLOSED)
                return "FOUND", self.extract_path(x_start, PARENT), [], []

            for s_n in self.get_neighbor(s):
                if s_n not in CLOSED:
                    new_cost = g_table[s] + self.cost(s, s_n)
                    if s_n not in g_table:
                        g_table[s_n] = float("inf")
                    if new_cost < g_table[s_n]:  # conditions for updating Cost
                        g_table[s_n] = new_cost
                        PARENT[s_n] = s
                        OPEN.put(s_n, g_table[s_n] + self.h_table[s_n])
                        
            if count == N:  # expand needed CLOSED nodes
                break

        self.visited.append(CLOSED)  # visited nodes in each iteration

        return OPEN, CLOSED, g_table, PARENT

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

    def extract_path_in_CLOSE(self, s_end, s_start, h_value):
        path = [s_start]
        s = s_start

        while True:
            h_list = {}
            for s_n in self.get_neighbor(s):
                if s_n in h_value:
                    h_list[s_n] = h_value[s_n]
            s_key = max(h_list, key=h_list.get)  # move to the smallest node with min h_value
            path.append(s_key)  # generate path
            s = s_key  # use end of this iteration as the start of next

            if s_key == s_end:  # reach the expected node in OPEN set
                return s_start, list(reversed(path))

    def extract_path(self, x_start, parent):
        """
        Extract the path based on the relationship of nodes.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = parent[s]
            path.append(s)
            if s == x_start:
                break      
        return list(reversed(path))

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
        path = [s_start, s_goal]
        if self.path_collision(path):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True

        x0, y0 = s_start
        x1, y1 = s_end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while x0 != x1 or y0 != y1:
            if (x0, y0) in self.obs:
                return True
            e2 = 2*err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return False
    
    def path_collision(self, path):
        for i in range(len(path) - 1):
            if self.is_collision(path[i], path[i+1]):
                print("Collision between", path[i], "and", path[i+1])
                return True
        return False
    
    def expanded_nodes_per_lookahead(self):  # added
        return self.num_nodes_explored / self.total_searches
    
    def path_length(self, path):
        cost = 0.0
        for i in range(len(path) - 1):
            cost += self.cost(path[i], path[i+1])
        return cost


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

def rtaa_algo(env, writer,exp, i):
    s_start = env.start
    s_goal = env.goal

    rtaa = RTAAStar(s_start, s_goal, 250, "euclidean", env)
    plot = plotting.Plotting(s_start, s_goal, env)

    tracemalloc.start()
    # start measuring time and memory usage at the start of the search
  
    start_time = time.perf_counter_ns()
    rtaa.memory_usage_before = rtaa.process.memory_info()
    rtaa.searching()
    # plot.animation_lrta(rtaa.path, rtaa.visited,f"Real-time Adaptive A* (RTAA*) on grid {i+1} start state {env.start} and goal state {env.goal}")
    rtaa.memory_usage_after = rtaa.process.memory_info()
    end_time = time.perf_counter_ns()
    snapshot = tracemalloc.take_snapshot()
    tracemalloc.stop()
    memo_rss = (rtaa.memory_usage_after.rss - rtaa.memory_usage_before.rss)/ 1024
    memo_vms = (rtaa.memory_usage_after.vms - rtaa.memory_usage_before.vms)/ 1024
    total = display_top(snapshot, limit=0) / 1024
    # print(f"\nGrid {i+1} with N = {N}:")
    path_cost = rtaa.path_length([node for segment in rtaa.path for node in segment])
    num_expanded_nodes = rtaa.num_nodes_explored
    num_searches = rtaa.total_searches
    expanded_nodes_per_lookahead = rtaa.expanded_nodes_per_lookahead()
    execution_time = (end_time - start_time) / 1e6 
    
    # Print the requested information

    print(f"Total path cost: {path_cost}") 
    print(f"Total number of expanded nodes: {num_searches}") 
    print(f"Number of searches made to find a solution: {rtaa.total_searches}")  
    # print(f"4. Number of expanded nodes per lookahead (iteration): {expanded_nodes_per_lookahead}")  # added
    # print(f"5. Total memory consumption {memory_consumption} MBi")
    print(f"Total allocated memory: {total} KB")
    print(f"Memory consumption RSS: {memo_rss} KB")
    print(f"Memory consumption VMS: {memo_vms} KB")
    print(f"Execution time: {execution_time} ms") 
    # I commentes this line to avoid the animation of the algorithm
    #     plot.animation_lrta(rtaa.path, rtaa.visited,f"Real-time Adaptive A* (RTAA*) on grid {i+1}")

    # Write the results into the CSV file
    writer.writerow([exp, i+1, env.obs_density, env.x_range , env.euclidean_distance, 250, path_cost, num_expanded_nodes, num_searches, total, memo_rss, memo_vms, execution_time])
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
        writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches",  "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])
        for i, env in enumerate(envs):          
            print(f"Running algorithm on grid {i+1}, s_state {env.start}, g_state {env.goal}, env_size {env.x_range}, obs_dancity {env.obs_density:.2f}, s/g distance {env.euclidean_distance} ")
            for exp in range(1, 101):
                rtaa_algo(env, writer,exp, i)
                # Call garbage collector to free up memory
                gc.collect()

    print("Environment have been processed.")

                      

def main():
    # Define environment loaders, directories and result files
    env_loaders = [grid_size_env_load_environments, start_goal_distance_load_environments, load_vertical_walls_environments, load_vertical_wall_size_environments, obstacle_density_load_environments]
    directories = ["grid_size_env/results", "start_goal_distance_env/results", "vertical_wall_env/results", "vertical_wall_size_env/results", "obstacle_density_env/results"]
    result_files = ['grid_size_env/results/RTAA_star_100_run_results.csv', 'start_goal_distance_env/results/RTAA_star_100_run_results.csv', 'vertical_wall_env/results/RTAA_star_100_run_results.csv', 'vertical_wall_size_env/results/RTAA_star_100_run_results.csv', 'obstacle_density_env/results/RTAA_star_100_run_results.csv']
   
    # Process each environment
    for env_loader, directory, result_file in zip(env_loaders, directories, result_files):
        process_env(env_loader, directory, result_file)

    print("All environments have been processed.")

if __name__ == '__main__':
    main()
