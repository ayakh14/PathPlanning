import random
from env import Env


class GridEnv(Env):
    def __init__(self, x_range, y_range, obs_density, start_state, goal_state):
        super().__init__()
        self.x_range = x_range
        self.y_range = y_range
        self.start = start_state
        self.goal = goal_state
        self.obs_density = obs_density
        self.obs = self.random_obs()
        self.manhattan_distance = abs(start_state[0]-goal_state[0]) + abs(start_state[1]-goal_state[1])

    def random_obs(self):
        obs = set()
        num_obs = int(self.obs_density * (self.x_range - 2) * (self.y_range - 2))

        while len(obs) < num_obs:
            x, y = self.random_state()
            if (x, y) == self.start or (x, y) == self.goal:
                continue
            else:
                obs.add((x, y))

        # Add border obstacles, but make sure it does not overlap with the start and goal states
        for i in range(self.x_range):
            if (i, 0) != self.start and (i, 0) != self.goal:
                obs.add((i, 0))
            if (i, self.y_range - 1) != self.start and (i, self.y_range - 1) != self.goal:
                obs.add((i, self.y_range - 1))

        for i in range(self.y_range):
            if (0, i) != self.start and (0, i) != self.goal:
                obs.add((0, i))
            if (self.x_range - 1, i) != self.start and (self.x_range - 1, i) != self.goal:
                obs.add((self.x_range - 1, i))

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
