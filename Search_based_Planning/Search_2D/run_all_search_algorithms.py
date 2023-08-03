

"""
    This script run all search algorithms on the selected environments.
    Each algorithm is executed as a separate subprocess and the multiprocessing 
    library is used to execute all algorithms in parallel
"""
import subprocess
import os
import sys
from multiprocessing import Pool

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

def run_script(script_name):
    subprocess.run(['python3', script_name])

script_names = ['RTAAStar.py', 'ARAstar.py', 'Astar.py' , 'LPAstar.py', 'D_star_Lite.py', 'LRTAstar.py', 'D_star.py']

# create a pool of processes and map the function and inputs to execute
with Pool() as p:
    p.map(run_script, script_names)


