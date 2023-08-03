import random
from env import Env
import math

class GridEnv(Env):
    def __init__(self, x_range, y_range, obs_density, start_state, goal_state):
        super().__init__()
        self.x_range = x_range
        self.y_range = y_range
        self.start = start_state
        self.goal = goal_state
        self.obs_density = obs_density
        self.obs = self.random_obs()
        self.euclidean_distance = math.sqrt((start_state[0] - goal_state[0])**2 + (start_state[1] - goal_state[1])**2)

    def random_obs(self):
        obs = set()
        num_obs = int(self.obs_density * (self.x_range - 2) * (self.y_range - 2))

        while len(obs) < num_obs:
            x, y = self.random_state()
            if (x, y) == self.start or (x, y) == self.goal:
                continue
            else:
                obs.add((x, y))

        # Add border obstacles
        for i in range(self.x_range):
            obs.add((i, 0))
            obs.add((i, self.y_range - 1))

        for i in range(self.y_range):
            obs.add((0, i))
            obs.add((self.x_range - 1, i))

        # Make sure the start and goal states are not obstacles
        obs.discard(self.start)
        obs.discard(self.goal)

        return obs


    def random_state(self):
        x = random.randint(1, self.x_range - 2)
        y = random.randint(1, self.y_range - 2)
        return (x, y)

    def add_obstacle(self, obstacle_state):
            self.obs.add(obstacle_state)

# # Initialize the grid
# x_range = 51  # specify the x dimension
# y_range = 31  # specify the y dimension
# obs_density = 0.2  # specify the obstacle density
# start_state = (1, 1)  # specify the start state
# goal_state = (49, 29)  # specify the goal state

# grid = GridEnv(x_range, y_range, obs_density, start_state, goal_state)
