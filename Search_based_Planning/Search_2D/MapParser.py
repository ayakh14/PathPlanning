def parse_map_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    metadata, grid = lines[:4], lines[4:]
    dimensions = [int(val.split()[1]) for val in metadata[:2]]
    height, width = dimensions

    obstacles = []
    for y, line in enumerate(grid):
        for x, char in enumerate(line.strip()):
            if char in {'@', 'O', 'T', 'S'}:
                obstacles.append((x, height - y - 1))

    return obstacles, dimensions
