"""
LRTA_star 2D (Learning Real-time A*)
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
# from 

class LrtAStarN:
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
       
        self.total_path_cost = 0
        self.total_expanded_nodes = 0
        self.total_searches = 0
        self.expanded_nodes_per_lookahead = []
       
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

 
        self.init()
        s_start = self.s_start  # initialize start node

        # Add the following lines at the beginning of the searching method in the LrtAStarN class
        self.total_searches = 0

        while True:
            OPEN, CLOSED = self.AStar(s_start, self.N)  # OPEN, CLOSED sets in each iteration

            if OPEN == "FOUND":  # reach the goal node
                self.path.append(CLOSED)
                break

            h_value = self.iteration(CLOSED)  # h_value table of CLOSED nodes

            for x in h_value:
                self.h_table[x] = h_value[x]

            s_start, path_k = self.extract_path_in_CLOSE(s_start, h_value)  # x_init -> expected node in OPEN set
            self.path.append(path_k)
            self.total_searches += 1

    def extract_path_in_CLOSE(self, s_start, h_value):
        path = [s_start]
        s = s_start

        while True:
            h_list = {}

            for s_n in self.get_neighbor(s):
                if s_n in h_value:
                    h_list[s_n] = h_value[s_n]
                else:
                    h_list[s_n] = self.h_table[s_n]

            s_key = min(h_list, key=h_list.get)  # move to the smallest node with min h_value
            path.append(s_key)  # generate path
            s = s_key  # use end of this iteration as the start of next

            if s_key not in h_value:  # reach the expected node in OPEN set
                return s_key, path

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

    def AStar(self, x_start, N):
        # Add the following lines at the beginning of the AStar method in the LrtAStarN class
        expanded_nodes_count = 0
        OPEN = queue_.QueuePrior()  # OPEN set
        OPEN.put(x_start, self.h(x_start))
        CLOSED = []  # CLOSED set
        g_table = {x_start: 0, self.s_goal: float("inf")}  # Cost to come
        PARENT = {x_start: x_start}  # relations
        count = 0  # counter

        while not OPEN.empty():
            count += 1
            # Add the following line after the "count += 1" line in the AStar method in the LrtAStarN class
            expanded_nodes_count += 1
            # print(f"expanded_nodes_count",expanded_nodes_count)
            s = OPEN.get()
            CLOSED.append(s)

            if s == self.s_goal:  # reach the goal node
                self.visited.append(CLOSED)
                # print(f"expanded_nodes_count",expanded_nodes_count)
                self.total_expanded_nodes += expanded_nodes_count
                self.expanded_nodes_per_lookahead.append(expanded_nodes_count)
                self.total_searches += 1  # increment search count when goal is found
                return "FOUND", self.extract_path(x_start, PARENT)

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
       
        # Add the following line before the "return OPEN, CLOSED" line in the AStar method in the LrtAStarN class
        self.total_expanded_nodes += expanded_nodes_count
        self.expanded_nodes_per_lookahead.append(expanded_nodes_count)
        
        return OPEN, CLOSED
    
    # Add the following method to the LrtAStarN class
    def calculate_total_path_cost(self):
        total_cost = 0
        for path_segment in self.path:
            for i in range(len(path_segment) - 1):
                segment_cost = self.cost(path_segment[i], path_segment[i + 1])
                if segment_cost == float("inf"):
                    self.total_path_cost = float("inf")
                    return
                total_cost += segment_cost
        self.total_path_cost = total_cost


    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = []

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                s_list.append(s_next)

        return s_list

    def extract_path(self, x_start, parent):
        """
        Extract the path based on the relationship of nodes.

        :return: The planning path
        """

        path_back = [self.s_goal]
        x_current = self.s_goal

        while True:
            x_current = parent[x_current]
            path_back.append(x_current)

            if x_current == x_start:
                break

        return list(reversed(path_back))

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
        points_on_path = bresenham_line(s_start, s_end)
        for point in points_on_path:
            if point in self.obs:
                return True
        return False

    
def bresenham_line(s_start, s_end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end

    :param s_start: start coordinate
    :param s_end: end coordinate
    :returns: list of points in the path
    """
    # Setup initial conditions
    x1, y1 = s_start
    x2, y2 = s_end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points

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

def lrta_star_alg (env, writer,exp, i):
    s_start = env.start
    s_goal = env.goal

    lrta = LrtAStarN(s_start, s_goal, 250, "euclidean", env)
    plot = plotting.Plotting(s_start, s_goal, env)
    
    tracemalloc.start()
    
    # start measuring time and memory usage at the start of the search
    start_time = time.perf_counter_ns()
    lrta.memory_usage_before = lrta.process.memory_info()
    lrta.searching()
    lrta.memory_usage_after = lrta.process.memory_info()
    end_time = time.perf_counter_ns()
    snapshot = tracemalloc.take_snapshot()
    tracemalloc.stop()
    memo_rss = (lrta.memory_usage_after.rss - lrta.memory_usage_before.rss)/ 1024
    memo_vms = (lrta.memory_usage_after.vms - lrta.memory_usage_before.vms)/ 1024
    total = display_top(snapshot, limit=0) / 1024
    lrta.calculate_total_path_cost()
    
    # print(f"\nGrid {i+1} with N = {N}:")

    path_cost = lrta.total_path_cost
    num_expanded_nodes = lrta.total_expanded_nodes
    num_searches = lrta.total_searches
    expanded_nodes_per_lookahead = lrta.expanded_nodes_per_lookahead
    execution_time = (end_time - start_time) / 1e6 
    
    # Print the requested information

    print(f"Total path cost: {path_cost}")
    print(f"Total number of expanded nodes: {num_expanded_nodes}")
    print(f"Number of searches made to find a solution: {num_searches}")
    # print(f"4. Number of expanded nodes per lookahead (iteration): {expanded_nodes_per_lookahead}")
    print(f"Total allocated memory: {total} KB")
    print(f"Memory consumption RSS: {memo_rss} KB")
    print(f"Memory consumption VMS: {memo_vms} KB")
    print(f"Execution time: {execution_time} ms")
    # I commentes this line to avoid the animation of the algorithm
    # plot.animation_lrta(lrta.path, lrta.visited,"Learning Real-time A* (LRTA*)")
    
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
        writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory Allocation (KB)", "RSS (KB)", "VMS (KB)", "Execution time (ms)"])
        for i, env in enumerate(envs):          
            print(f"Running algorithm on grid {i+1}, s_state {env.start}, g_state {env.goal}, env_size {env.x_range}, obs_dancity {env.obs_density:.2f}, s/g distance {env.euclidean_distance} ")
            for exp in range(1, 101):
                lrta_star_alg (env, writer,exp, i)
                # Call garbage collector to free up memory
                gc.collect()
def main():
    # Define environment loaders, directories and result files
    env_loaders = [grid_size_env_load_environments, start_goal_distance_load_environments, load_vertical_walls_environments, load_vertical_wall_size_environments, obstacle_density_load_environments]
    directories = ["grid_size_env/results", "start_goal_distance_env/results", "vertical_wall_env/results", "vertical_wall_size_env/results", "obstacle_density_env/results"]
    result_files = ['grid_size_env/results/LRTA_star_100_run_results.csv', 'start_goal_distance_env/results/LRTA_star_100_run_results.csv', 'vertical_wall_env/results/LRTA_star_100_run_results.csv', 'vertical_wall_size_env/results/LRTA_star_100_run_results.csv', 'obstacle_density_env/results/LRTA_star_100_run_results.csv']

    # Process each environment
    for env_loader, directory, result_file in zip(env_loaders, directories, result_files):
        process_env(env_loader, directory, result_file)

    print("All environments have been processed.")


if __name__ == '__main__':
    main()