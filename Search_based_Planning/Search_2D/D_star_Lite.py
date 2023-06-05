"""
D_star_Lite 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt
import time
import psutil

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
from random_env import RandomEnv
# from one_hundred_env_generator import load_environments
import csv
from changing_obstacle_density import obstacle_density_load_environments
from changing_start_goal_distance import start_goal_distance_load_environments
from changing_grid_size import grid_size_env_load_environments


# Starting and End measuring time
start_time = None
end_time = None

m1 = None
m2 = None 
process = psutil.Process()

class DStar:
    def __init__(self, s_start, s_goal, heuristic_type, env_instance=None):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        if env_instance is None:
            self.Env = env.Env()
        else:
            self.Env = env_instance
        
        self.Plot = plotting.Plotting(s_start, s_goal, self.Env)

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0

        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self.CalculateKey(self.s_goal)
        self.visited = set()
        self.count = 0
        self.fig = plt.figure()

        self.total_path_cost = 0
        self.total_expanded_nodes = 0
        self.total_searches = 0

    def run(self):
        global process, start_time, end_time, m1, m2 
        # measuring time at the start
        start_time = time.time()
        # print(f"mempry with pustil 1 {process.memory_info().rss} --> MB {process.memory_info().rss/1024/1024}")  # in bytes 
        m1 = process.memory_info().rss

        self.Plot.plot_grid("D* Lite")
        self.ComputePath()

        # This is to simulate clicking on the grid so to get the complete path.
        # obj = type('obj', (object,), {'xdata' : 5, 'ydata' : 2})
        # self.on_press( obj )

        self.plot_path(self.extract_path())
        m2 = process.memory_info().rss
        # End measuring time
        end_time = time.time()
        self.total_path_cost = self.path_length(self.extract_path())
        
        print(f"1. Total path cost: {self.total_path_cost}")
        print(f"2. Total number of expanded nodes: {self.total_expanded_nodes}")
        print(f"3. Number of searches made to find a solution: {self.total_searches}")
        print(f"Total memory consumption {(m2 - m1)/1024/1024} MB")
        print(f"Execution time: {end_time - start_time} seconds")
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        # plt.show()
        

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)

            s_curr = self.s_start
            s_last = self.s_start
            i = 0
            path = [self.s_start]

            while s_curr != self.s_goal:
                s_list = {}

                for s in self.get_neighbor(s_curr):
                    s_list[s] = self.g[s] + self.cost(s_curr, s)
                s_curr = min(s_list, key=s_list.get)
                path.append(s_curr)

                if i < 1:
                    self.km += self.h(s_last, s_curr)
                    s_last = s_curr
                    if (x, y) not in self.obs:
                        self.obs.add((x, y))
                        plt.plot(x, y, 'sk')
                        self.g[(x, y)] = float("inf")
                        self.rhs[(x, y)] = float("inf")
                    else:
                        self.obs.remove((x, y))
                        plt.plot(x, y, marker='s', color='white')
                        self.UpdateVertex((x, y))
                    for s in self.get_neighbor((x, y)):
                        self.UpdateVertex(s)
                    i += 1

                    self.count += 1
                    self.visited = set()
                    self.ComputePath()

            self.plot_visited(self.visited)
            self.plot_path(path)
            self.fig.canvas.draw_idle()

    def ComputePath(self):
        self.total_searches += 1  # Increment the counter for total searches
        while True:
            s, v = self.TopKey()
            self.total_expanded_nodes += 1  # Increment the counter for total expanded nodes
           
            if v >= self.CalculateKey(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break

            k_old = v
            self.U.pop(s)
            self.visited.add(s)

            if k_old < self.CalculateKey(s):
                self.U[s] = self.CalculateKey(s)
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)

    def UpdateVertex(self, s):
        if s != self.s_goal:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.U:
            self.U.pop(s)

        if self.g[s] != self.rhs[s]:
            self.U[s] = self.CalculateKey(s)

    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.U, key=self.U.get)
        return s, self.U[s]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

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

    # def is_collision(self, s_start, s_end):
    #     if s_start in self.obs or s_end in self.obs:
    #         return True

    #     if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
    #         if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
    #             s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
    #             s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
    #         else:
    #             s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
    #             s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

    #         if s1 in self.obs or s2 in self.obs:
    #             return True

    #     return False

    def is_collision(self, s_start, s_end):
        points_on_path = bresenham_line(s_start, s_end)
        for point in points_on_path:
            if point in self.obs:
                return True
        return False

    



    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]
        s = self.s_start

        for k in range(10000):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_goal:
                break

        return list(path)

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

    def path_length(self, path):
        return sum(self.cost(path[i], path[i+1]) for i in range(len(path) - 1))

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
        writer.writerow(["Experiment", "Grid number", "Obstacle density", "Grid size", "s/g distance", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory consumption (MB)", "Execution time (s)"])
        for i, env in enumerate(envs):          
            print(f"Running algorithm on grid {i+1}, s_state {env.start}, g_state {env.goal}, env_size {env.x_range}, obs_dancity {env.obs_density:.2f}, s/g distance {env.manhattan_distance} ")
            for exp in range(10):
                s_start = env.start
                s_goal = env.goal

                dstar = DStar(s_start, s_goal, "euclidean", env)
                dstar.run()
                # get results from the Dstar instance
                path_cost = dstar.total_path_cost
                num_expanded_nodes = dstar.total_expanded_nodes
                num_searches = dstar.total_searches
                # expanded_nodes_per_lookahead =
                memory_consumption = (m2 - m1)/1024/1024
                execution_time = end_time - start_time
                writer.writerow([exp, i+1, env.obs_density, env.x_range , env.manhattan_distance, "-", path_cost, num_expanded_nodes, num_searches, memory_consumption, execution_time])
    print("All environments have been processed.")

def main():
    # Define environment loaders, directories and result files
    env_loaders = [grid_size_env_load_environments, start_goal_distance_load_environments, obstacle_density_load_environments]
    directories = ["grid_size_env/results", "start_goal_distance_env/results", "obstacle_density_env/results"]
    result_files = ['grid_size_env/results/D_star_Lite_results.csv', 'start_goal_distance_env/results/D_star_Lite_results.csv', 'obstacle_density_env/results/D_star_Lite_results.csv']
    
    # Process each environment
    for env_loader, directory, result_file in zip(env_loaders, directories, result_files):
        process_env(env_loader, directory, result_file)

    print("All environments have been processed.")

if __name__ == '__main__':
    main()  