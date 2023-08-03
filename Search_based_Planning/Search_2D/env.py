import matplotlib.pyplot as plt
import math

class Env:
    obs_density = 0  # Set the static value for obs_density

    def __init__(self, x_range=31, y_range=71, s_state=(1,1), goal_state=(29,69)):
        self.x_range = x_range
        self.y_range = y_range
        self.start = s_state
        self.goal = goal_state
        self.num_walls = 0
        self.wall_length = 0
        self.wall_lengths = []

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.init_obs()
        self.directions = ['left', 'right', 'left', 'right', 'left', 'right']
        self.euclidean_distance = math.sqrt((s_state[0] - goal_state[0])**2 + 
                                             (s_state[1] - goal_state[1])**2)

    def init_obs(self):
        obs = set()
        for i in range(self.x_range):
            obs.add((i, 0))
            obs.add((i, self.y_range - 1))
        for i in range(self.y_range):
            obs.add((0, i))
            obs.add((self.x_range - 1, i))
        return obs

    def add_wall(self, wall_position, wall_direction, wall_size=0):
        current_num_obs = len(self.obs)
        self.wall_y = wall_position
        self.wall_direction = wall_direction

        half_width = self.x_range // 2
        if wall_direction == 'left':
            start = 0
            end = min(half_width + wall_size, self.x_range - 1)
        else:  # 'right'
            start = max(half_width - wall_size, 0)
            end = min(start + half_width + wall_size, self.x_range - 1)
        for j in range(start, end):
            self.obs.add((j, wall_position))
        new_num_obs = len(self.obs)
        wall_added = new_num_obs - current_num_obs
        self.wall_lengths.append(wall_added)
        self.num_walls += 1
        self.wall_length = wall_added
class VerticalWallEnv(Env):
    obs_density = 0  # Set the static value for obs_density
   
    def __init__(self, start_state=(1, 1), goal_state=None, x_range=31, y_range=71):
        self.x_range = x_range
        self.y_range = y_range
        self.start = start_state
        self.num_walls = 0  # initialize the wall counter
        self.wall_lengths = []

        if goal_state is None:
            goal_state = ((self.x_range - 2), self.y_range - 2)
        self.goal = goal_state
        super().__init__(x_range=self.x_range, y_range=self.y_range, s_state=start_state, goal_state=goal_state)
        self.wall_y = None
        self.wall_direction = None
        self.euclidean_distance = math.sqrt((start_state[0] - goal_state[0])**2 + 
                                             (start_state[1] - goal_state[1])**2)

    def add_wall(self, wall_position, wall_direction):
        current_num_obs = len(self.obs)

        self.wall_y = wall_position
        self.wall_direction = wall_direction
        half_width = self.x_range // 2
        start = 0 if wall_direction == 'right' else half_width
        end = half_width if wall_direction == 'right' else self.x_range
        for j in range(start, end):
            self.obs.add((j, wall_position))
        self.num_walls += 1
        new_num_obs = len(self.obs)
        wall_added = new_num_obs - current_num_obs
        self.wall_lengths.append(wall_added)
        self.wall_length = wall_added