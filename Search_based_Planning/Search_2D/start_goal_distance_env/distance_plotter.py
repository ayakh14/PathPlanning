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

# Calculate means for each obstacle density
for i in range(len(dataframes)):
    dataframes[i] = dataframes[i].groupby('s/g distance').mean().reset_index()

algorithm_names = ["RTAA*", "A*", "LRTA*", "ARA*", "D*", "D* Lite", "LPA*"]

# Define line styles and markers
line_styles = ['-', '--', ':', '-.']
markers = ['o', 's', '^', 'v', '*', 'D', 'X']

# Calculate some basic statistics
for i, df in enumerate(dataframes):
    print(f"Algorithm {i+1}")
    print(f"Mean path cost: {df['Path cost'].mean()}")
    print(f"Mean number of expanded nodes: {df['Number of expanded nodes'].mean()}")
    print(f"Mean memory consumption: {df['Memory consumption (MB)'].mean()}")
    print(f"Mean execution time: {df['Execution time (s)'].mean()}")
    print("\n")

# Plotting
fig, axs = plt.subplots(2, 2, figsize=(12, 12))

# Mean Path cost per s/g distance
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[0, 0].plot(df['s/g distance'], df['Path cost'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[0, 0].set_xlabel('s/g distance')
axs[0, 0].set_ylabel('Path cost')
axs[0, 0].set_yscale('log')  # Set y-axis to logarithmic scale
axs[0, 0].legend()

# Mean number of expanded nodes per s/g distance
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[0, 1].plot(df['s/g distance'], df['Number of expanded nodes'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[0, 1].set_xlabel('s/g distance')
axs[0, 1].set_ylabel('Number of expanded nodes')
axs[0, 1].set_yscale('log')  # Set y-axis to logarithmic scale
axs[0, 1].legend()

# Mean memory consumption per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[1, 0].plot(df['s/g distance'], df['Memory consumption (MB)'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[1, 0].set_xlabel('s/g distance')
axs[1, 0].set_ylabel('Memory consumption (MB)')
axs[1, 0].set_yscale('log')  # Set y-axis to logarithmic scale
axs[1, 0].legend()

# Mean execution time per Obstacle density
for i, df in enumerate(dataframes):
    style = line_styles[i % len(line_styles)]  # Cycle through line styles
    marker = markers[i % len(markers)]  # Cycle through markers
    axs[1, 1].plot(df['s/g distance'], df['Execution time (s)'], label=algorithm_names[i], linestyle=style, marker=marker, alpha=0.7)
axs[1, 1].set_xlabel('s/g distance')
axs[1, 1].set_ylabel('Execution time (s)')
axs[1, 1].set_yscale('log')  # Set y-axis to logarithmic scale
axs[1, 1].legend()

# Save the figure as a PNG file
plt.savefig('results_plot.png')

plt.show()
