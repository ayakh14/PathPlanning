"""
D_star 2D
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


class DStar:
    def __init__(self, s_start, s_goal, env_instance=None):
        self.s_start, self.s_goal = s_start, s_goal

        if env_instance is None:
            self.Env = env.Env()
        else:
            self.Env = env_instance

        self.Plot = plotting.Plotting(self.s_start, self.s_goal, self.Env)

        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.fig = plt.figure()

        self.OPEN = set()
        self.t = dict()
        self.PARENT = dict()
        self.h = dict()
        self.k = dict()
        self.path = []
        self.visited = set()
        self.count = 0


        self.total_path_cost = 0.0
        self.total_expanded_nodes = 0
        self.total_searches = 0

        self.process = psutil.Process()
        self.memory_usage_before = None
        self.memory_usage_after = None
    
    def init(self):
        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.t[(i, j)] = 'NEW'
                self.k[(i, j)] = 0.0
                self.h[(i, j)] = float("inf")
                self.PARENT[(i, j)] = None

        self.h[self.s_goal] = 0.0

    def run(self, s_start, s_end):

        self.init()
        self.insert(s_end, 0)

        while True:
            self.process_state()
            if self.t[s_start] == 'CLOSED':
                break

        self.path = self.extract_path(s_start, s_end)
        self.Plot.plot_grid("Dynamic A* (D*)")
        self.plot_path(self.path)

        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        # plt.show()


        print(f"1. Total path cost: {self.total_path_cost}")
        print(f"2. Total number of expanded nodes: {self.total_expanded_nodes}")
        print(f"3. Number of searches made to find a solution: {self.total_searches}")
        print(f"4. Number of expanded nodes per lookahead (iteration): {self.total_expanded_nodes / self.total_searches}")
        # print(f"5. Total memory consumption {(m2 - m1)/1024/1024} MB")
        # print(f"6. Execution time: {end_time - start_time} seconds")
    
    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            if (x, y) not in self.obs:
                print("Add obstacle at: s =", x, ",", "y =", y)
                self.obs.add((x, y))
                self.Plot.update_obs(self.obs)

                s = self.s_start
                self.visited = set()
                self.count += 1

                while s != self.s_goal:
                    if self.is_collision(s, self.PARENT[s]):
                        self.modify(s)
                        continue
                    s = self.PARENT[s]

                self.path = self.extract_path(self.s_start, self.s_goal)

                plt.cla()
                self.Plot.plot_grid("Dynamic A* (D*)")
                self.plot_visited(self.visited)
                self.plot_path(self.path)

            self.fig.canvas.draw_idle()

    def extract_path(self, s_start, s_end):
        path = [s_start]
        s = s_start
        while True:
            s_next = self.PARENT[s]
            self.total_path_cost += self.cost(s, s_next)
            path.append(s_next)
            s = s_next
            if s == s_end:
                return path

    def process_state(self):
        self.total_searches += 1
        s = self.min_state()  # get node in OPEN set with min k value
        self.visited.add(s)

        if s is None:
            return -1  # OPEN set is empty

        k_old = self.get_k_min()  # record the min k value of this iteration (min path cost)
        self.delete(s)  # move state s from OPEN set to CLOSED set

        # k_min < h[s] --> s: RAISE state (increased cost)
        if k_old < self.h[s]:
            for s_n in self.get_neighbor(s):
                self.total_expanded_nodes += 1  # count the expanded node here

                if self.h[s_n] <= k_old and \
                        self.h[s] > self.h[s_n] + self.cost(s_n, s):

                    # update h_value and choose parent
                    self.PARENT[s] = s_n
                    self.h[s] = self.h[s_n] + self.cost(s_n, s)

        # s: k_min >= h[s] -- > s: LOWER state (cost reductions)
        if k_old == self.h[s]:
            for s_n in self.get_neighbor(s):
                self.total_expanded_nodes += 1  # count the expanded node here

                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)) or \
                        (self.PARENT[s_n] != s and self.h[s_n] > self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    # 3) s_n find a better parent
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
        else:
            for s_n in self.get_neighbor(s):
                self.total_expanded_nodes += 1  # count the expanded node here

                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
                else:
                    if self.PARENT[s_n] != s and \
                            self.h[s_n] > self.h[s] + self.cost(s, s_n):

                        # Condition: LOWER happened in OPEN set (s), s should be explored again
                        self.insert(s, self.h[s])
                    else:
                        if self.PARENT[s_n] != s and \
                                self.h[s] > self.h[s_n] + self.cost(s_n, s) and \
                                self.t[s_n] == 'CLOSED' and \
                                self.h[s_n] > k_old:

                            # Condition: LOWER happened in CLOSED set (s_n), s_n should be explored again
                            self.insert(s_n, self.h[s_n])

        return self.get_k_min()

    def min_state(self):
        """
        choose the node with the minimum k value in OPEN set.
        :return: state
        """

        if not self.OPEN:
            return None

        return min(self.OPEN, key=lambda x: self.k[x])

    def get_k_min(self):
        """
        calc the min k value for nodes in OPEN set.
        :return: k value
        """

        if not self.OPEN:
            return -1

        return min([self.k[x] for x in self.OPEN])

    def insert(self, s, h_new):
        """
        insert node into OPEN set.
        :param s: node
        :param h_new: new or better cost to come value
        """

        if self.t[s] == 'NEW':
            self.k[s] = h_new
        elif self.t[s] == 'OPEN':
            self.k[s] = min(self.k[s], h_new)
        elif self.t[s] == 'CLOSED':
            self.k[s] = min(self.h[s], h_new)

        self.h[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def delete(self, s):
        """
        delete: move state s from OPEN set to CLOSED set.
        :param s: state should be deleted
        """

        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'

        self.OPEN.remove(s)

    def modify(self, s):
        """
        start processing from state s.
        :param s: is a node whose status is RAISE or LOWER.
        """

        self.modify_cost(s)

        while True:
            k_min = self.process_state()

            if k_min >= self.h[s]:
                break

    def modify_cost(self, s):
        # if node in CLOSED set, put it into OPEN set.
        # Since cost may be changed between s - s.parent, calc cost(s, s.p) again

        if self.t[s] == 'CLOSED':
            self.insert(s, self.h[self.PARENT[s]] + self.cost(s, self.PARENT[s]))

    def get_neighbor(self, s):
        nei_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

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

def dstar_algo(env, writer,exp, i):
    s_start = env.start
    s_goal = env.goal

    dstar = DStar(s_start, s_goal, env)
    tracemalloc.start()
    # start measuring time and memory usage at the start of the search
    start_time = time.perf_counter_ns()
    dstar.memory_usage_before = dstar.process.memory_info()
    dstar.run(s_start, s_goal)
    dstar.memory_usage_after = dstar.process.memory_info()
    end_time = time.perf_counter_ns()
    snapshot = tracemalloc.take_snapshot()
    tracemalloc.stop()
    memo_rss = (dstar.memory_usage_after.rss - dstar.memory_usage_before.rss)/ 1024
    memo_vms = (dstar.memory_usage_after.vms - dstar.memory_usage_before.vms)/ 1024
    total = display_top(snapshot, limit=0) / 1024

    # get results from the Dstar instance
    path_cost = dstar.total_path_cost
    num_expanded_nodes = dstar.total_expanded_nodes
    num_searches = dstar.total_searches
    # memory_consumption = (m2 - m1)/1024/1024
    execution_time = (end_time - start_time) / 1e6 
    writer.writerow([exp, i+1, env.obs_density, env.x_range , env.euclidean_distance, "-", path_cost, num_expanded_nodes, num_searches, total, memo_rss, memo_vms,execution_time])
    plt.close(dstar.fig)


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
                dstar_algo(env, writer,exp, i)
                # Call garbage collector to free up memory
                gc.collect()
    print("the environment has been processed.")

def main():
    # Define environment loaders, directories and result files
    env_loaders = [grid_size_env_load_environments, start_goal_distance_load_environments, load_vertical_walls_environments, load_vertical_wall_size_environments, obstacle_density_load_environments]
    directories = ["grid_size_env/results", "start_goal_distance_env/results", "vertical_wall_env/results", "vertical_wall_size_env/results", "obstacle_density_env/results"]
    result_files = ['grid_size_env/results/D_star_100_run_results.csv', 'start_goal_distance_env/results/D_star_100_run_results.csv', 'vertical_wall_env/results/D_star_100_run_results.csv', 'vertical_wall_size_env/results/D_star_100_run_results.csv', 'obstacle_density_env/results/D_star_100_run_results.csv']
    
    # Process each environment
    for env_loader, directory, result_file in zip(env_loaders, directories, result_files):
        process_env(env_loader, directory, result_file)

    print("All environments have been processed.")

if __name__ == '__main__':
    main()  