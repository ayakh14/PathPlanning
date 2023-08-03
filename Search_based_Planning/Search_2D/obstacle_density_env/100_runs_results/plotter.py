# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# import os

# # Define the file names of your CSV files
# file_names = ['A_star_100_run_results.csv', 
#               'ARA_star_100_run_results.csv', 
#               'RTAA_star_100_run_results.csv', 
#               'LRTA_star_100_run_results.csv', 
#               'D_star_Lite_100_run_results.csv', 
#               'LPA_star_100_run_results.csv']


# # Define a function to calculate summary statistics
# def calc_summary_stats(group):
#     return pd.Series({
#         'Mean path cost': group['Path cost'].mean(),
#         'Std dev path cost': group['Path cost'].std(),
#         'Mean num expanded nodes': group['Number of expanded nodes'].mean(),
#         'Std dev num expanded nodes': group['Number of expanded nodes'].std(),
#         'Mean num searches': group['Number of searches'].mean(),
#         'Std dev num searches': group['Number of searches'].std(),
#         'Mean memory allocation': group['Memory Allocation (KB)'].mean(),
#         'Std dev memory allocation': group['Memory Allocation (KB)'].std(),
#         'Mean execution time': group['Execution time (ms)'].mean(),
#         'Std dev execution time': group['Execution time (ms)'].std()
#     })

# # Define a function to plot the given metric
# def plot_metric(obstacle_stats, metric_name, algorithm_name):
#     plt.errorbar(obstacle_stats.index, obstacle_stats[f'Mean {metric_name}'], yerr=obstacle_stats[f'Std dev {metric_name}'], capsize=3)
#     plt.xlabel('Obstacle density')
#     plt.ylabel(f'Mean {metric_name}')
#     plt.title(f'Algorithm: {algorithm_name}')
#     plt.savefig(f'{metric_name.lower()}_{os.path.splitext(algorithm_name)[0]}_100_run.png')
#     plt.clf()

# # Loop over the CSV files
# for file_name in file_names:
#     # Load the CSV file
#     df = pd.read_csv(file_name)
    
#     # Calculate summary statistics for each grid
#     grid_stats = df.groupby(['Obstacle density', 'Grid number']).apply(calc_summary_stats)

#     # Calculate summary statistics for each obstacle density
#     obstacle_stats = grid_stats.groupby('Obstacle density').mean()

#     # Save the summary statistics to a new CSV file
#     algorithm_name = os.path.splitext(file_name)[0]
#     grid_stats.to_csv(f'grid_stats_{algorithm_name}.csv')
#     obstacle_stats.to_csv(f'obstacle_stats_{algorithm_name}.csv')

#     # Plot the mean metrics with error bars representing standard deviation
#     plot_metric(obstacle_stats, 'Path cost', algorithm_name)
#     plot_metric(obstacle_stats, 'Number of expanded nodes', algorithm_name)
#     plot_metric(obstacle_stats, 'Number of searches', algorithm_name)
#     plot_metric(obstacle_stats, 'Memory allocation', algorithm_name)
#     # # Plotting the mean execution time with error bars representing standard deviation
#     # plt.errorbar(obstacle_stats.index, obstacle_stats['Mean execution time'], yerr=obstacle_stats['Std dev execution time'], capsize=3)
#     # plt.xlabel('Obstacle density')
#     # plt.ylabel('Mean execution time (ms)')
#     # plt.title(f'Algorithm: {os.path.splitext(file_name)[0]}')
#     # plt.savefig(f'execution_time_{os.path.splitext(file_name)[0]}_100_run.png')
#     # plt.clf()






import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

# Create a directory for saving plot images
save_dir = 'plot_images_100_run'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Load your data into a DataFrame
data_files = [
    # ('A_star_100_run_results.csv', 'A*'),
    ('ARA_star_100_run_results.csv', 'ARA*'),
    ('RTAA_star_100_run_results.csv', 'RTAA**'),
    ('LRTA_star_100_run_results.csv', 'LRTA*'),
    ('D_star_100_run_results.csv', 'D*'),
    ('D_star_Lite_100_run_results.csv', 'D* Lite'),
    ('LPA_star_100_run_results.csv', 'LPA*')
]

df_list = []
for file_name, algo_name in data_files:
    temp_df = pd.read_csv(os.path.join(file_name))
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
    plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_vs_Obstacle_Density_line_plot_100_run.png"))
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
    plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_vs_Obstacle_Density_error_bar_plot_100_run.png"))
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




# # Set up box plots
# box_width = 0.4  # Width of each box
# box_space_width = 0.4  # Width of the space between groups of boxes
# color_dict = dict(zip(algorithms, colors))

# for metric in metrics:
#     plt.figure(figsize=(10, 6))
    
#     # Draw boxplots
#     box_plot = sns.boxplot(data=df, x='Obstacle density', y=metric, hue='Algorithm', width=box_width, palette=colors)
#     box_plot.set_xticklabels(box_plot.get_xticklabels(), rotation=90)
    
#     plt.xlabel('Obstacle Density')
#     plt.ylabel(metric)
#     plt.yscale('symlog')
#     plt.title(f'{metric} Distribution for Each Algorithm')
    
#     plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_Distribution_box_plot_100_run.png"))
#     plt.close()

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
#     plt.savefig(os.path.join(save_dir, f"{metric.replace(' ', '_')}_vs_Obstacle_Density_bar_plot_100_run.png"))
#     plt.close()



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
 