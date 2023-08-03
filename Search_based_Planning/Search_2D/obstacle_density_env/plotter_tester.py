import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

# load your data into a DataFrames
data_files = [
    ('A_star_30_run_results.csv', 'A*'),
    ('ARA_star_30_run_results.csv', 'ARA*'),
    ('RTAA_star_30_run_results.csv', 'RTAA**'),
    ('LRTA_star_30_run_results.csv', 'LRTA*'),
    ('D_star_30_run_results.csv', 'D*'),
    ('D_star_Lite_30_run_results.csv', 'D* Lite'),
    ('LPA_star_30_run_results.csv', 'LPA*')
]

df_list = []
for file_name, algo_name in data_files:
    temp_df = pd.read_csv(os.path.join('results', file_name))
    temp_df['Algorithm'] = algo_name
    df_list.append(temp_df)

# Concatenate the dataframes
df = pd.concat(df_list)

# Set a color palette
sns.set_palette('mako')

# Set style
sns.set_style("whitegrid")

# Create distribution plots and box plots for each metric
metrics = ['Execution time (ms)', 'Path cost', 'Memory Allocation (KB)', 'Number of expanded nodes', 'Number of searches']
algorithms = df['Algorithm'].unique()

# for metric in metrics:
#     plt.figure(figsize=(10, 6))
#     sns.boxplot(x='Algorithm', y=metric, data=df, width= .5)
#     plt.title(f'{metric} Boxplot for Each Algorithm', fontsize=14)
#     plt.xlabel('Algorithm', fontsize=12)
#     plt.ylabel(metric, fontsize=12)

#     # adding the mean point to the plot
#     means = df.groupby('Algorithm')[metric].mean().values
#     mean_labels = [f"Mean: {np.round(m, 2)}" for m in means]
#     plt.scatter(x=range(df['Algorithm'].nunique()), y=means, color='red', label=mean_labels)

#     plt.legend()
#     plt.show()



# Ensure Obstacle density is a float
df['Obstacle density'] = df['Obstacle density'].astype(float)

# Create line plots for each metric
metrics = ['Execution time (ms)', 'Path cost', 'Memory Allocation (KB)', 'Number of expanded nodes', 'Number of searches']
algorithms = df['Algorithm'].unique()
line_styles = ['-', '--', '-.', ':', '-', '--', '-.']

# for metric in metrics:
#     plt.figure(figsize=(15, 8))  # Increase figure size
    
#     for algo, line_style in zip(algorithms, line_styles):
#         algo_df = df[df['Algorithm'] == algo]
#         means = algo_df.groupby('Obstacle density')[metric].mean()
#         std_devs = algo_df.groupby('Obstacle density')[metric].std()
        
#         plt.plot(means.index, means, line_style, label=f"{algo} Mean")
#         plt.fill_between(std_devs.index, means - std_devs, means + std_devs, alpha=0.1)
    
#     plt.title(f'{metric} vs Obstacle Density for Each Algorithm', fontsize=14)
#     plt.xlabel('Obstacle Density', fontsize=12)
#     plt.ylabel(metric, fontsize=12)
#     plt.yscale('log')
#     plt.legend()
#     plt.show()
###################################################################333
# Bar plots
# bar_width = 0.05  # Width of each bar
# space_width = 0.05  # Width of the space between groups of bars

# for metric in metrics:
#     plt.figure(figsize=(10, 6))
#     x = np.arange(len(algorithms))
#     group_width = bar_width * len(algorithms) + space_width * (len(algorithms) - 1)
    
#     for i, algo in enumerate(algorithms):
#         algo_df = df[df['Algorithm'] == algo]
#         means = algo_df.groupby('Obstacle density')[metric].mean()
#         plt.bar(x + (i * (bar_width + space_width)), means, bar_width, label=algo)
    
#     plt.xlabel('Obstacle Density')
#     plt.ylabel(metric)
#     plt.yscale('log')
#     plt.title(f'{metric} vs Obstacle Density for Each Algorithm')
#     plt.xticks(x + (group_width / 2), [f'{density:.2f}' for density in means.index])
#     plt.legend()
#     plt.show()


# # Violin plots
# for metric in metrics:
#     plt.figure(figsize=(10, 6))
#     sns.violinplot(x='Algorithm', y=metric, data=df)
#     plt.title(f'{metric} Distribution for Each Algorithm')
#     plt.xlabel('Algorithm')
#     plt.ylabel(metric)
#     plt.yscale('log')
#     plt.show()



# box_width = 0.2  # Width of each box
# box_space_width = 0.4  # Width of the space between groups of boxes
# colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'gray']  # Assign colors to algorithms

# for metric in metrics:
#     plt.figure(figsize=(10, 6))
#     x = np.arange(len(algorithms))
#     group_width = box_width * len(algorithms) + box_space_width * (len(algorithms) - 1)
    
#     for i, algo in enumerate(algorithms):
#         algo_df = df[df['Algorithm'] == algo]
#         means = algo_df.groupby('Obstacle density')[metric].mean()
#         std_devs = algo_df.groupby('Obstacle density')[metric].std()
#         plt.errorbar(x + (i * (box_width + box_space_width)), means, yerr=std_devs, fmt='o', color=colors[i], label=algo, capsize=3)
    
#     plt.xlabel('Obstacle Density')
#     plt.ylabel(metric)
#     plt.yscale('symlog')
#     plt.title(f'{metric} Distribution for Each Algorithm')
#     plt.xticks(x + (group_width / 2), [f'{density:.2f}' for density in df['Obstacle density'].unique()])
    
#     # Add legend
#     legend_patches = [plt.Rectangle((0, 0), 1, 1, facecolor=color) for color in colors[:len(algorithms)]]
#     plt.legend(legend_patches, algorithms)
    
#     plt.show()


# line graph 
line_width = 2  # Width of the lines
colors = sns.color_palette("mako", len(algorithms))  # Assign colors using the 'mako' palette
markers = ['o', 's', 'v', '^', 'D', '*', 'x']  # Assign markers to algorithms
line_styles = ['-', '--', '-.', ':', '-', '--', '-.']  # Assign line styles to algorithms

for metric in metrics:
    plt.figure(figsize=(10, 6))
    x = np.arange(len(algorithms))
    
    for i, algo in enumerate(algorithms):
        algo_df = df[df['Algorithm'] == algo]
        means = algo_df.groupby('Obstacle density')[metric].mean()
        std_devs = algo_df.groupby('Obstacle density')[metric].std()
        plt.errorbar(means.index, means, yerr=std_devs, linestyle=line_styles[i], marker=markers[i], markersize=6, linewidth=line_width, color=colors[i], label=algo, capsize=3)
    
    plt.xlabel('Obstacle Density')
    plt.ylabel(metric)
    plt.yscale('log')
    plt.title(f'{metric} vs Obstacle Density for Each Algorithm')
    
    # Add legend
    legend_patches = [plt.Line2D([0], [0], marker=markers[i], linestyle=line_styles[i], color='w', markerfacecolor=colors[i], markersize=8, linewidth=0) for i in range(len(algorithms))]
    plt.legend(legend_patches, algorithms)
    
    plt.grid(True)
    plt.show()