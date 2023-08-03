
# import threading

# import random
# from env import Env  # Assuming that the original Env class is defined in a file named "env.py"
# import math 

# class RandomEnv(Env):
#     def __init__(self, x_range, y_range, obs_density):
#         super().__init__()
#         self.x_range = x_range
#         self.y_range = y_range
#         self.obs_density = obs_density
#         self.start = self.random_state()
#         self.goal = self.random_state()
#         self.obs = self.random_obs_map()
#         self.euclidean_distance = math.sqrt((self.start[0] - self.goal[0])**2 + (self.start[1] - self.goal[1])**2)

#     def random_state(self):
#         x = random.randint(1, self.x_range - 2)
#         y = random.randint(1, self.y_range - 2)
#         return (x, y)

#     def random_obs_map(self):
#         obs = set()

#         # Add border obstacles
#         for i in range(self.x_range):
#             obs.add((i, 0))
#             obs.add((i, self.y_range - 1))

#         for i in range(self.y_range):
#             obs.add((0, i))
#             obs.add((self.x_range - 1, i))

#         # Add random obstacles
#         num_obs = int(self.obs_density * (self.x_range - 2) * (self.y_range - 2))

#         while len(obs) < num_obs:
#             x, y = self.random_state()
#             if (x, y) != self.start and (x, y) != self.goal:
#                 obs.add((x, y))

#         return obs

#     # def add_random_obstacle(self):
#     #     obstacle_added = False
#     #     while not obstacle_added:
#     #         x = random.randint(0, self.x_range - 1)
#     #         y = random.randint(0, self.y_range - 1)
#     #         new_obstacle = (x, y)

#     #         if new_obstacle not in self.obs and new_obstacle != self.start and new_obstacle != self.goal:
#     #             self.obs.append(new_obstacle)
#     #             obstacle_added = True

 


# class CustomEnv(Env):
#     def __init__(self, dimensions, obstacles):
#         super().__init__()
#         self.x_range, self.y_range = dimensions
#         self.obs = obstacles

#     def obs_map(self):
#         return self.obs





import threading

import random
from env import Env  # Assuming that the original Env class is defined in a file named "env.py"
import math 

class RandomEnv(Env):
    def __init__(self, x_range, y_range, obs_density):
        super().__init__()
        self.x_range = x_range
        self.y_range = y_range
        self.obs_density = obs_density
        self.obs = self.random_obs_map()  # Initialize obstacles first
        self.start = self.random_state()
        self.goal = self.random_state()
        while self.goal == self.start:   # Make sure goal isn't the same as start
            self.goal = self.random_state()
        self.euclidean_distance = math.sqrt((self.start[0] - self.goal[0])**2 + (self.start[1] - self.goal[1])**2)
    def random_state(self):
        x = random.randint(1, self.x_range - 2)
        y = random.randint(1, self.y_range - 2)
        return (x, y)

    def random_obs_map(self):
        obs = set()

        # Add border obstacles
        for i in range(self.x_range):
            obs.add((i, 0))
            obs.add((i, self.y_range - 1))

        for i in range(self.y_range):
            obs.add((0, i))
            obs.add((self.x_range - 1, i))

        # Add random obstacles
        num_obs = int(self.obs_density * (self.x_range - 2) * (self.y_range - 2))

        while len(obs) < num_obs:
            x, y = self.random_state()
            
            # Ensure obstacle isn't at start or goal
            if (x, y) != self.start and (x, y) != self.goal:  
                obs.add((x, y))

        return obs


    # def add_random_obstacle(self):
    #     obstacle_added = False
    #     while not obstacle_added:
    #         x = random.randint(0, self.x_range - 1)
    #         y = random.randint(0, self.y_range - 1)
    #         new_obstacle = (x, y)

    #         if new_obstacle not in self.obs and new_obstacle != self.start and new_obstacle != self.goal:
    #             self.obs.append(new_obstacle)
    #             obstacle_added = True

 


class CustomEnv(Env):
    def __init__(self, dimensions, obstacles):
        super().__init__()
        self.x_range, self.y_range = dimensions
        self.obs = obstacles

    def obs_map(self):
        return self.obs
