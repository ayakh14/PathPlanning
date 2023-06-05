"""
    this script run all all algorithms on the environment selected
"""
import subprocess
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

def run_script(script_name):
    subprocess.run(['python', script_name])

script_names = ['Astar.py', 'D_star.py', 'ARAstar.py', 
                'LPAstar.py', 'D_star_lite.py', 'LRTAstar.py', 'RTAAStar.py']

for script in script_names:
    run_script(script)




