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
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env
from random_env import RandomEnv
from one_hundred_env_generator import load_environments
import csv




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

        # Starting measuring time
        self.start_time = None
        # End measuring time
        self.end_time = None
        self.process = psutil.Process()
        self.memory_usage_before = None
        self.memory_usage_after = None


        self.total_path_cost = 0
        self.total_expanded_nodes = 0
        self.total_searches = 0
        self.expanded_nodes_per_lookahead = []


    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """
        # start measuring time and memory usage at the start of the search
        self.start_time = time.time()
        self.memory_usage_before = self.process.memory_info().rss
        
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
        self.memory_usage_after = self.process.memory_info().rss
        self.end_time = time.time()
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



# main for random env

def main():
    # Load environments
    envs = load_environments()
    # os.makedirs('results', exist_ok=True)
    with open('one_hundred_random_grids/results/A_star_results.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        # Write the header of the CSV file
        writer.writerow(["Experiment", "Grid", "lookahead" , "Path cost", "Number of expanded nodes", "Number of searches", "Memory consumption (MB)", "Execution time (s)"])
        for i, env in enumerate(envs):
                        
            print(f"Running algorithm on grid {i+1} with start state {env.start} and goal state {env.goal}")

            s_start = env.start
            s_goal = env.goal        

            astar = AStar(s_start, s_goal, "euclidean", env)
            plot = plotting.Plotting(s_start, s_goal, env)

            path, visited = astar.searching()

            total_expanded_nodes = sum(len(nodes) for nodes in visited)
            nodes_per_search = [len(nodes) for nodes in visited]
            total_path_cost = sum(astar.cost(path[i], path[i-1]) for i in range(1, len(path))) # Calculate total path cost
            execution_time = astar.end_time - astar.start_time
            memory_consumption = (astar.memory_usage_after - astar.memory_usage_before)/1024/1024
            
            # Added lines - Print the gathered information
            print(f"Total path cost: {total_path_cost}")
            print(f"Total number of expanded nodes: {total_expanded_nodes}")
            print(f"Number of searches made to find a solution: {astar.total_searches}")
            # print(f"Number of expanded nodes per lookahead (iteration): {nodes_per_search}")
            print(f"Total memory consumption {memory_consumption} MB")
            print(f"Execution time: {execution_time} seconds")
            writer.writerow([1, i+1, "-", total_path_cost, total_expanded_nodes, astar.total_searches, memory_consumption, execution_time])

            # plot.animation(path, visited, "A*")  # animation

            # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
            # plot.animation_ara_star(path, visited, "Repeated A*")
   
    print("All environments have been processed.")


if __name__ == '__main__':
    main()


# # main for random env

# def main():
#     x_range = 51
#     y_range = 51
#     obs_density = 0.2  # 20% of the cells will have obstacles

#     random_env = RandomEnv(x_range, y_range, obs_density)
#     s_start = random_env.start
#     s_goal = random_env.goal
#     astar = AStar(s_start, s_goal, "euclidean", random_env)
#     plot = plotting.Plotting(s_start, s_goal, random_env)

#     path, visited = astar.searching()

#     total_expanded_nodes = sum(len(nodes) for nodes in visited)
#     nodes_per_search = [len(nodes) for nodes in visited]
#     total_path_cost = sum(astar.cost(path[i], path[i-1]) for i in range(1, len(path))) # Calculate total path cost
#     # Added lines - Print the gathered information
#     print(f"Total path cost: {total_path_cost}")
#     print(f"Total number of expanded nodes: {total_expanded_nodes}")
#     print(f"Number of searches made to find a solution: {astar.total_searches}")
#     # print(f"Number of expanded nodes per lookahead (iteration): {nodes_per_search}")
#     print(f"Total memory consumption {(astar.memory_usage_after - astar.memory_usage_before)/1024/1024} MB")
#     print(f"Execution time: {astar.end_time - astar.start_time} seconds")

#     plot.animation(path, visited, "A*")  # animation

#     # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
#     # plot.animation_ara_star(path, visited, "Repeated A*")



# if __name__ == '__main__':
#     main()



######################### main for static environment 
# def main():
#     s_start = (5, 5)
#     s_goal = (45, 25)

#     astar = AStar(s_start, s_goal, "euclidean")
#     plot = plotting.Plotting(s_start, s_goal)

#     path, visited = astar.searching()
#     plot.animation(path, visited, "A*")  # animation

#     # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
#     # plot.animation_ara_star(path, visited, "Repeated A*")


# if __name__ == '__main__':
#     main()
