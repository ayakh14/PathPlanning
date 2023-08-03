import csv
from collections import defaultdict

def calculate_mean(data):
    mean_data = {}
    for experiment, experiment_data in data.items():
        path_cost_sum = 0
        memory_sum = 0
        execution_time_sum = 0
        num_experiments = len(experiment_data)

        for d in experiment_data:
            path_cost_sum += d['Path cost']
            memory_sum += d['Memory Allocation (KB)']
            execution_time_sum += d['Execution time (ms)']

        mean_data[experiment] = {
            'Mean Path cost': path_cost_sum / num_experiments,
            'Mean Memory Allocation (KB)': memory_sum / num_experiments,
            'Mean Execution time (ms)': execution_time_sum / num_experiments
        }

    return mean_data

# Read data from CSV file
filename = 'grid_size_execution_time_ara_star.csv'
data = defaultdict(list)
with open(filename, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        experiment = row['Experiment']
        data[experiment].append({
            'Path cost': float(row['Path cost']),
            'Memory Allocation (KB)': float(row['Memory Allocation (KB)']),
            'Execution time (ms)': float(row['Execution time (ms)'])
        })

# Calculate the mean of path cost, memory allocation, and execution time
mean_data = calculate_mean(data)

# Print the mean data for each experiment
for experiment, mean_values in mean_data.items():
    print(f"Experiment {experiment}:")
    for key, value in mean_values.items():
        print(f"{key}: {value}")
    print()
