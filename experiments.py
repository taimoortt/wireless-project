import os
import fileinput
import sys
from analyze import gather_results
import time

micro_cells = 0
macro_cells = 1
total_cells = macro_cells + micro_cells
inter_micro_distance = 500
DURATION = 0.5

def run_exps():
        config_file = f'nomute.json'
        file_name = '1'
        file_name += '.log'
        print('File: ', file_name)

        final_cmd = f''
        cmd = (
                f"sudo ./LTE-Sim MultiCell {total_cells} 1 {2} 0 0 1 0 8 1 0 0.1 128 "
                f"{config_file} {macro_cells} {micro_cells} {inter_micro_distance} {DURATION} "
                f"100 3 > {file_name}"
        )

        final_cmd += cmd
        print(final_cmd)
        os.system(final_cmd)

        plot_cmd = f"python3 plot_geo.py {file_name}"
        os.system(plot_cmd)
run_exps()