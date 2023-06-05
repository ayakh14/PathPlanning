import pandas as pd
import matplotlib.pyplot as plt
import os
from one_hundred_env_generator import load_environments

envs = load_environments()

def manhattan_distance(s_start, g_goal):
    return abs(s_start[0] - g_goal[0]) + abs(s_start[1] - g_goal[1])

distances = []
for env in envs:
    distances.append(manhattan_distance(env.start, env.goal))



# Make directories to put results in it if it doesn't exist
os.makedirs('one_hundred_random_grids/results/rnd_grid_search_results_plots', exist_ok=True)
# os.makedirs('one_hundred_random_grids/results/results_average', exist_ok=True)

# 1st step: Load the data from the .csv files
rtaa_star_df = pd.read_csv(os.path.join('one_hundred_random_grids', 'results', 'RTAA_star_results.csv'))
a_star_df = pd.read_csv(os.path.join('one_hundred_random_grids', 'results', 'A_star_results.csv'))
lrta_star_df = pd.read_csv(os.path.join('one_hundred_random_grids', 'results', 'LRTA_star_results.csv'))
ara_star_df = pd.read_csv(os.path.join('one_hundred_random_grids', 'results', 'ARA_star_results.csv'))
d_star_df = pd.read_csv(os.path.join('one_hundred_random_grids', 'results', 'D_star_results.csv'))
d_star_lite_df = pd.read_csv(os.path.join('one_hundred_random_grids', 'results', 'D_star_Lite_results.csv'))
lpa_star_df = pd.read_csv(os.path.join('one_hundred_random_grids', 'results', 'LPA_star_results.csv'))

# create a list to store the columns you want to calculate the average of
average_columns = ['Path cost', 'Number of expanded nodes', 'Number of searches', 'Memory consumption (MB)', 'Execution time (s)']

# create a new DataFrame to store the averages
average_df  = pd.DataFrame()
average_df['lookahead'] = rtaa_star_df['lookahead'].unique()
# # add distances to average_df
# average_df['Distance'] = distances
for column in average_columns:
    for algo_df, algo_name in zip([rtaa_star_df, lrta_star_df, ara_star_df, d_star_df, d_star_lite_df, lpa_star_df, a_star_df, ],
                                  [ ' (RTAA_star)', ' (LRTA_star)', ' (ARA_star)', ' (D_star)', ' (D_star_Lite)', ' (LPA_star)', ' (A_star)']):
        avg_values = algo_df.groupby('lookahead')[column].mean().values
        if len(avg_values) == 1:
            avg_values = [avg_values[0]] * len(average_df)  # Repeat the single value
        average_df[column + algo_name] = avg_values

# Print the results in terminal
# print(average_df)

# Save the results in a .csv
average_df.to_csv(os.path.join('one_hundred_random_grids', 'results', 'average_results.csv'), index=False)


# For each algorithm create its own plots of metrics 
for column in average_columns:
    for algo in [' (RTAA_star)', ' (LRTA_star)', ' (ARA_star)', ' (D_star)', ' (D_star_Lite)', ' (LPA_star)', ' (A_star)']:
        plt.figure(figsize=(10,6))
        plt.plot(average_df['lookahead'], average_df[column + algo], label=column+algo)
        plt.xlabel('Lookahead')
        plt.ylabel(column)
        plt.yscale('log')  # Make the y-axis logarithmic
        plt.title(f'{column} vs Lookahead for {algo}')
        plt.legend()
        plt.grid(True)
        # save the figure as a PNG image in the folder 'grid_search_results'
        plt.savefig(os.path.join('one_hundred_random_grids', 'results', 'rnd_grid_search_results_plots', f"{column.replace(' ', '_')}_{algo.replace(' ', '_')}_vs_Lookahead.png"), dpi=300)  
        plt.close()
        # plt.show()

# Creat a plot that combines a plot for each metric 
for metric in average_columns:
    plt.figure(figsize=(10,6))
    for algo in [' (RTAA_star)', ' (LRTA_star)', ' (ARA_star)', ' (D_star)', ' (D_star_Lite)', ' (LPA_star)', ' (A_star)']:
        plt.plot(average_df['lookahead'], average_df[metric + algo], label=metric+algo)
    plt.xlabel('Lookahead')
    plt.ylabel(metric)
    plt.yscale('log')  # Make the y-axis logarithmic
    plt.title(f'{metric} vs Lookahead')
    plt.legend()
    plt.grid(True)

    # save resulting figures in a .PNG format in 'grid_search_results' folder
    plt.savefig(os.path.join('one_hundred_random_grids', 'results', 'rnd_grid_search_results_plots', f"{metric.replace(' ', '_')}_vs_Lookahead.png"), dpi=300)  
    plt.close()
    # plt.show()





# Creating combined plots for each metric in group 1: D_star, D_star_lite, LPA_star
group_1 = [' (D_star)', ' (D_star_Lite)', ' (LPA_star)', ' (A_star)']
for metric in average_columns:
    plt.figure(figsize=(10,6))
    for algo in group_1:
        plt.plot(average_df['lookahead'], average_df[metric + algo], label=metric+algo)  # Scale lookahead by 10
    plt.xlabel('Lookahead ')
    plt.ylabel(metric)
    plt.yscale('log')  # Make the y-axis logarithmic
    plt.title(f'{metric} vs Lookahead - Group 1')
    plt.legend()
    plt.grid(True)
    # save the figure as a PNG image in the folder 'grid_search_results'
    plt.savefig(os.path.join('one_hundred_random_grids', 'results', 'rnd_grid_search_results_plots', f"{metric.replace(' ', '_')}_vs_Lookahead_Group1.png"), dpi=300)  
    plt.close()
    # plt.show()





