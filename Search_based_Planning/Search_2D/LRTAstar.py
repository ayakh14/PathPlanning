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

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import queue, plotting, env
from random_env import RandomEnv

# Starting measuring time
start_time = None
# End measuring time
end_time = None
process = psutil.Process()
m2 = None
m1 = None

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
        

    def init(self):
        """
        initialize the h_value of all nodes in the environment.
        it is a global table.
        """

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.h_table[(i, j)] = self.h((i, j))

    def searching(self):

        global process, start_time, end_time, m1, m2
        # measuring time at the start
        start_time = time.time()
        # print(f"mempry with pustil 1 {process.memory_info().rss} --> MB {process.memory_info().rss/1024/1024}")  # in bytes 
        m1 = process.memory_info().rss

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

        m2 = process.memory_info().rss
        # End measuring time
        end_time = time.time()
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
        OPEN = queue.QueuePrior()  # OPEN set
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
        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            s1 = (s_start[0], s_end[1])
            s2 = (s_end[0], s_start[1])

            if s1 in self.obs and s2 in self.obs:
                return True

        return False



def main():
    x_range = 50
    y_range = 50
    obs_density = 0.2  # 20% of the cells will have obstacles

    random_env = RandomEnv(x_range, y_range, obs_density)
    s_start = random_env.start
    s_goal = random_env.goal

    lrta = LrtAStarN(s_start, s_goal, 30, "euclidean", random_env)
    plot = plotting.Plotting(s_start, s_goal, random_env)

    lrta.searching()
    lrta.calculate_total_path_cost()


    # Print the requested information

    print(f"1. Total path cost: {lrta.total_path_cost}")
    print(f"2. Total number of expanded nodes: {lrta.total_expanded_nodes}")
    print(f"3. Number of searches made to find a solution: {lrta.total_searches}")
    print(f"4. Number of expanded nodes per lookahead (iteration): {lrta.expanded_nodes_per_lookahead}")
    print(f"5. Total memory consumption {(m2 - m1)/1024/1024} MB")
    print(f"6. Execution time: {end_time - start_time} seconds") 
    plot.animation_lrta(lrta.path, lrta.visited,
                        "Learning Real-time A* (LRTA*)")
    
    # def main():
    # s_start = (10, 5)
    # s_goal = (45, 25)

    # lrta = LrtAStarN(s_start, s_goal, 400, "euclidean")
    # plot = plotting.Plotting(s_start, s_goal)

    # lrta.searching()
    # lrta.calculate_total_path_cost()


    # # Print the requested information


    # print(f"1. Total path cost: {lrta.total_path_cost}")
    # print(f"2. Total number of expanded nodes: {lrta.total_expanded_nodes}")
    # print(f"3. Number of searches made to find a solution: {lrta.total_searches}")
    # print(f"4. Number of expanded nodes per lookahead (iteration): {lrta.expanded_nodes_per_lookahead}")
    # print(f"5. Total memory consumption {(m2 - m1)/1024/1024} MB")
    # print(f"6. Execution time: {end_time - start_time} seconds") 
    # plot.animation_lrta(lrta.path, lrta.visited,
    #                     "Learning Real-time A* (LRTA*)")
if __name__ == '__main__':
    main()
