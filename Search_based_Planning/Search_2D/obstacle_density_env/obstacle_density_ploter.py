import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

# Create a directory for saving plot images
save_dir = 'plot_images'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Load your data into a DataFrame
data_files = [
    ('A_star_10_run_results.csv', 'A*'),
    ('ARA_star_10_run_results.csv', 'ARA*'),
    ('RTAA_star_10_run_results.csv', 'RTAA**'),
    ('LRTA_star_10_run_results.csv', 'LRTA*'),
    # ('D_star_10_run_results.csv', 'D*'),
    ('D_star_Lite_10_run_results.csv', 'D* Lite'),
    ('LPA_star_10_run_results.csv', 'LPA*')
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

# Ensure Obstacle density is a float
df['Obstacle density'] = df['Obstacle density'].astype(float)

# Create line plots for each metric
metrics = ['Execution time (ms)', 'Path cost', 'Memory Allocation (KB)', 'Number of expanded nodes', 'Number of searches']
algorithms = df['Algorithm'].unique()
line_styles = ['-', '--', '-.', ':', '-', '--', '-.']  # Assign line styles to algorithms
markers = ['o', 's', 'v', '^', 'D', '*', 'X']  # Assign markers to algorithms
colors = sns.color_palette("mako", len(algorithms))  # Assign colors using the 'mako' palette

# Line plots with error bars and filled areas to represent the mean and standard deviation of a metric
for metric in metrics:
    plt.figure(figsize=(15, 8))
    
    for i, algo in enumerate(algorithms):
        algo_df = df[df['Algorithm'] == algo]
        means = algo_df.groupby('Obstacle density')[metric].mean()
        std_devs = algo_df.groupby('Obstacle density')[metric].std()
        
        plt.plot(means.index, means, line_styles[i], marker=markers[i], label=f"{algo} Mean")  # Add marker parameter
        plt.fill_between(std_devs.index, means - std_devs, means + std_devs, alpha=0.1)
    
    plt.title(f'{metric} vs Obstacle Density for Each Algorithm', fontsize=14)
    plt.xlabel('Obstacle Density', fontsize=12)
    plt.ylabel(metric, fontsize=12)
    plt.yscale('log')
    plt.legend()
    plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_vs_Obstacle_Density_line_plot.png"))
    plt.close()

# Line graph error bar plots to represent the mean and standard deviation of a metric
line_width = 2  # Width of the lines
for metric in metrics:
    plt.figure(figsize=(10, 6))
    
    for i, algo in enumerate(algorithms):
        algo_df = df[df['Algorithm'] == algo]
        means = algo_df.groupby('Obstacle density')[metric].mean()
        std_devs = algo_df.groupby('Obstacle density')[metric].std()
        
        # Error bar
        plt.errorbar(means.index, means, yerr=std_devs, linestyle=line_styles[i], marker=markers[i], markersize=6, linewidth=line_width, color=colors[i], label=algo, capsize=3)
    
    plt.xlabel('Obstacle Density')
    plt.ylabel(metric)
    plt.yscale('log')
    plt.title(f'{metric} vs Obstacle Density for Each Algorithm')
    
    # Add legend
    legend_patches = [plt.Line2D([0], [0], marker=markers[i], linestyle=line_styles[i], color='w', markerfacecolor=colors[i], markersize=8, linewidth=0) for i in range(len(algorithms))]
    plt.legend(legend_patches, algorithms)
    
    plt.grid(True)
    plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_vs_Obstacle_Density_error_bar_plot.png"))
    plt.close()

# # Bar plots
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
#     plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_vs_Obstacle_Density_bar_plot.png"))
#     plt.close()

# Set up box plots
box_width = 0.4  # Width of each box
box_space_width = 0.4  # Width of the space between groups of boxes
color_dict = dict(zip(algorithms, colors))

for metric in metrics:
    plt.figure(figsize=(10, 6))
    
    # Draw boxplots
    box_plot = sns.boxplot(data=df, x='Obstacle density', y=metric, hue='Algorithm', width=box_width, palette=colors)
    box_plot.set_xticklabels(box_plot.get_xticklabels(), rotation=90)
    
    plt.xlabel('Obstacle Density')
    plt.ylabel(metric)
    plt.yscale('symlog')
    plt.title(f'{metric} Distribution for Each Algorithm')
    
    plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_Distribution_box_plot.png"))
    plt.close()


table_data = []
for metric in metrics:
    for algo in algorithms:
        algo_df = df[df['Algorithm'] == algo]
        for sg_dist in algo_df['Obstacle density'].unique():
            sg_dist_df = algo_df[algo_df['Obstacle density'] == sg_dist]
            mean = sg_dist_df[metric].mean()
            median = sg_dist_df[metric].median()
            std_dev = sg_dist_df[metric].std()
            min_val = sg_dist_df[metric].min()
            max_val = sg_dist_df[metric].max()

            row = [algo, metric, sg_dist, mean, median, std_dev, min_val, max_val]
            table_data.append(row)

table_df = pd.DataFrame(table_data, columns=['Algorithm', 'Metric', 'Obstacle Density', 'Mean', 'Median', 'Standard Deviation', 'Minimum', 'Maximum'])

# Export the DataFrame to a CSV file
table_df.to_csv('summary_statistics.csv', index=False)


# # Violin plots
# for metric in metrics:
#     plt.figure(figsize=(10, 6))
#     sns.violinplot(x='Algorithm', y=metric, data=df)
#     plt.title(f'{metric} Distribution for Each Algorithm')
#     plt.xlabel('Algorithm')
#     plt.ylabel(metric)
#     plt.yscale('log')
#     plt.show()



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
