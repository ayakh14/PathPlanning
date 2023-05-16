import math


# def calculate_path_length(path):
#     total_length = 0
#     for i in range(len(path) - 1):
#         s_start = path[i]
#         s_end = path[i + 1]
#         distance = math.sqrt((s_end[0] - s_start[0])**2 + (s_end[1] - s_start[1])**2)
#         total_length += distance
#     return total_length


def calculate_path_length(path):
    return len(path) - 1  # Subtract 1 since the start node is not considered a step
