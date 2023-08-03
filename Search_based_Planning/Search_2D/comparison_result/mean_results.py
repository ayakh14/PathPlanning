import pandas as pd


file_names = [
    "comparison_result_Grid Size Environments_ExecutionTime_ara_star.csv",
    "comparison_result_Grid Size Environments_PathCost_dstar_lite_algo.csv",
    "comparison_result_Grid Size Environments_Memory_dstar_lite_algo.csv",
    
    "comparison_result_Obstacle Density Environments_ExecutionTime_rtaa_algo.csv",
    "comparison_result_Obstacle Density Environments_Memory_dstar_lite_algo.csv",
    "comparison_result_Obstacle Density Environments_PathCost_dstar_lite_algo.csv",

    "comparison_result_Start to Goal Distance Environments_Memory_dstar_lite_algo.csv",
    "comparison_result_Start to Goal Distance Environments_ExecutionTime_rtaa_algo.csv",
    "comparison_result_Start to Goal Distance Environments_PathCost_dstar_lite_algo.csv",

    # Add as many file names as you have
]


for file_name in file_names:
    # Read the CSV file
    df = pd.read_csv(file_name)  
    
    # Drop the specified columns
    columns_to_remove = ["lookahead", "Grid number", "Number of expanded nodes", "Number of searches", "RSS (KB)", "VMS (KB)"]
    data = df.drop(columns=columns_to_remove)
    
    # Group by 'Experiment' and calculate the mean
    results = data.groupby('Experiment').mean().reset_index()

    # Save the results to a new CSV file
    # Here, we're naming the new file by adding "_processed" before ".csv" in the original file name
    output_file_name = file_name.replace(".csv", "_processed.csv")
    results.to_csv(output_file_name, index=False)

    print(f"Processed {file_name} and saved to {output_file_name}")