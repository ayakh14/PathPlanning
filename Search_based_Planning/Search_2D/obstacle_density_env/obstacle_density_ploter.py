import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os 

# Load the CSV files
rtaa_star_df = pd.read_csv(os.path.join('results', 'RTAA_star_results.csv'))
a_star_df = pd.read_csv(os.path.join('results', 'A_star_results.csv'))
lrta_star_df = pd.read_csv(os.path.join('results','LRTA_star_results.csv'))
ara_star_df = pd.read_csv(os.path.join('results', 'ARA_star_results.csv'))
d_star_df = pd.read_csv(os.path.join('results', 'D_star_results.csv'))
d_star_lite_df = pd.read_csv(os.path.join('results', 'D_star_Lite_results.csv'))
lpa_star_df = pd.read_csv(os.path.join('results', 'LPA_star_results.csv'))

dataframes = [rtaa_star_df, a_star_df, lrta_star_df, ara_star_df, d_star_df, d_star_lite_df, lpa_star_df]

# Define line styles and markers
line_styles = ['-', '--', ':', '-.']
markers = ['o', '^', 's', 'd', '*', 'x', '+']

# Calculate means for each obstacle density
for i in range(len(dataframes)):
    dataframes[i] = dataframes[i].groupby('Obstacle density').mean().reset_index()

algorithm_names = ["RTAA*", "A*", "LRTA*", "ARA*", "D*", "D* Lite", "LPA*"]

# Calculate some basic statistics
for i, df in enumerate(dataframes):
    print(f"Algorithm {i+1}")
    print(f"Mean path cost: {df['Path cost'].mean()}")
    print(f"Mean number of expanded nodes: {df['Number of expanded nodes'].mean()}")
    print(f"Mean memory allocated: {df['Memory Allocation (KB)'].mean()}")
    print(f"Mean RSS memory: {df['RSS (KB)'].mean()}")
    print(f"Mean VMS memory: {df['VMS (KB)'].mean()}")
    print(f"Mean execution time: {df['Execution time (ms)'].mean()}")
    print("\n")

# Plotting
fig, axs = plt.subplots(2, 3, figsize=(20, 12))

# Mean Path cost per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[0, 0].plot(df['Obstacle density'], df['Path cost'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[0, 0].set_xlabel('Obstacle density')
axs[0, 0].set_ylabel('Path cost')
axs[0, 0].set_yscale('log')  # Set y-axis to logarithmic scale
axs[0, 0].legend()

# Mean number of expanded nodes per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[0, 1].plot(df['Obstacle density'], df['Number of expanded nodes'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[0, 1].set_xlabel('Obstacle density')
axs[0, 1].set_ylabel('Number of expanded nodes')
axs[0, 1].set_yscale('log')  # Set y-axis to logarithmic scale
axs[0, 1].legend()

# Mean execution time per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[0, 2].plot(df['Obstacle density'], df['Execution time (ms)'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[0, 2].set_xlabel('Obstacle density')
axs[0, 2].set_ylabel('Execution time (ms)')
axs[0, 2].set_yscale('log')  # Set y-axis to logarithmic scale
axs[0, 2].legend()

# Mean memory consumption per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[1, 0].plot(df['Obstacle density'], df['Memory Allocation (KB)'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[1, 0].set_xlabel('Obstacle density')
axs[1, 0].set_ylabel('Memory Allocation (KB)')
axs[1, 0].set_yscale('log')  # Set y-axis to logarithmic scale
axs[1, 0].legend()

# Mean RSS memory per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[1, 1].plot(df['Obstacle density'], df['RSS (KB)'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[1, 1].set_xlabel('Obstacle density')
axs[1, 1].set_ylabel('RSS (KB)')
axs[1, 1].set_yscale('log')  # Set y-axis to logarithmic scale
axs[1, 1].legend()

# Mean VMS memory per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[1, 2].plot(df['Obstacle density'], df['VMS (KB)'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[1, 2].set_xlabel('Obstacle density')
axs[1, 2].set_ylabel('VMS (KB)')
axs[1, 2].set_yscale('log')  # Set y-axis to logarithmic scale
axs[1, 2].legend()

# Save the figure as a PNG file
plt.savefig('obs_density_results_plot.png')

plt.show()
