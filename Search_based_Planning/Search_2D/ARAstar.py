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

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
from random_env import RandomEnv
from one_hundred_env_generator import load_environments
import csv


# Starting measuring time
start_time = None
# End measuring time
end_time = None
process = psutil.Process()
m2 = None
m1 = None

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
                                                          # order of visited nodes

    def init(self):
        """
        initialize each set.
        """

        self.g[self.s_start] = 0.0
        self.g[self.s_goal] = math.inf
        self.OPEN[self.s_start] = self.f_value(self.s_start)
        self.PARENT[self.s_start] = self.s_start

    def searching(self):

        global process, start_time, end_time, m1, m2
        # measuring time at the start
        start_time = time.time()
        # print(f"mempry with pustil 1 {process.memory_info().rss} --> MB {process.memory_info().rss/1024/1024}")  # in bytes 
        m1 = process.memory_info().rss


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

        m2 = process.memory_info().rss
        # End measuring time
        end_time = time.time()
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

def main():
    # Create the directory if it doesn't exist
    directory = "one_hundred_random_grids/results"
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Load environments
    envs = load_environments()
    with open('one_hundred_random_grids/results/ARA_star_results.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        # Write the header of the CSV file
        writer.writerow(["Experiment", "Grid", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory consumption (MB)", "Execution time (s)"])
        
        e = 1
        for exp in range(1, 21):  # loop for 20 experiments
            for i, env in enumerate(envs):
                
                print(f"Running algorithm on grid {i+1} with start state {env.start} and goal state {env.goal}")

                s_start = env.start
                s_goal = env.goal
    
                arastar = AraStar(s_start, s_goal, e, "euclidean", env)
                plot = plotting.Plotting(s_start, s_goal, env)

                path, visited = arastar.searching()
                
                print(f"\nGrid {i+1} with N = {e}:")

                path_cost = arastar.calculate_path_cost()
                num_expanded_nodes = sum(len(nodes) for nodes in visited)
                num_searches = arastar.searches
                expanded_nodes_per_lookahead = [len(nodes) for nodes in visited]
                memory_consumption = (m2 - m1)/1024/1024
                execution_time = end_time - start_time
                

                # Added lines - Print the gathered information
                print(f"1.Total path cost: {path_cost}")
                print(f"2.Total number of expanded nodes: {num_expanded_nodes}")
                print(f"3.Number of searches made to find a solution: {num_searches}")
                print(f"4.Number of expanded nodes per lookahead (iteration): {expanded_nodes_per_lookahead}")
                print(f"5.Total memory consumption {memory_consumption} MBi")
                print(f"6.Execution time: {execution_time} seconds")

                # plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")
                # Write the results into the CSV file
                writer.writerow([exp, i+1, e, path_cost, num_expanded_nodes, num_searches, memory_consumption, execution_time])
                
            e += 1  # increase N for the next grid

    print("All environments have been processed.")
                


if __name__ == '__main__':
    main()


######################################################
######################################################
##################randm env#####################
######################################################
######################################################
######################################################

# def main():
#     x_range = 51
#     y_range = 51
#     obs_density = 0.2  # 20% of the cells will have obstacles

#     random_env = RandomEnv(x_range, y_range, obs_density)
#     s_start = random_env.start
#     s_goal = random_env.goal

#     arastar = AraStar(s_start, s_goal, 2.5, "euclidean", random_env)
#     plot = plotting.Plotting(s_start, s_goal, random_env)

#     path, visited = arastar.searching()

#     total_expanded_nodes = sum(len(nodes) for nodes in visited)
#     nodes_per_search = [len(nodes) for nodes in visited]
#     total_path_cost = arastar.calculate_path_cost()    # Added line - Calculate total path cost

#     # Added lines - Print the gathered information
#     print(f"Total path cost: {total_path_cost}")
#     print(f"Total number of expanded nodes: {total_expanded_nodes}")
#     print(f"Number of searches made to find a solution: {arastar.searches}")
#     print(f"Number of expanded nodes per lookahead (iteration): {nodes_per_search}")
#     print(f"Total memory consumption {(m2 - m1)/1024/1024} MB")
#     print(f"Execution time: {end_time - start_time} seconds")

#     plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")



# if __name__ == '__main__':
#     main()

# def main():
#     s_start = (5, 5)
#     s_goal = (45, 25)

#     arastar = AraStar(s_start, s_goal, 2.5, "euclidean")
#     plot = plotting.Plotting(s_start, s_goal)

#     path, visited = arastar.searching()
#     plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")

#     total_expanded_nodes = sum(len(nodes) for nodes in visited)
#     nodes_per_search = [len(nodes) for nodes in visited]
#     total_path_cost = arastar.calculate_path_cost()    # Added line - Calculate total path cost

#     # Added lines - Print the gathered information
#     print(f"Total path cost: {total_path_cost}")
#     print(f"Total number of expanded nodes: {total_expanded_nodes}")
#     print(f"Number of searches made to find a solution: {arastar.searches}")
#     print(f"Number of expanded nodes per lookahead (iteration): {nodes_per_search}")
#     print(f"Total memory consumption {(m2 - m1)/1024/1024} MB")
#     print(f"Execution time: {end_time - start_time} seconds")



# if __name__ == '__main__':
#     main()
