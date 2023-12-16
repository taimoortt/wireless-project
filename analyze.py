#!/home/networklab/anaconda3/bin/python3
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
import statistics as stats
import math
import sys
from analysis_functions import *

def gather_results(dname, dname_muting, seed = -1, duration = -1, inter_micro_distance = -1, num_micro = 4):

    file = open('results.txt', 'a') 
    tp = get_throughput(dname)
    tp_mute = get_throughput(dname_muting)

    ue_tp = get_per_ue_throughput(dname)
    pf_metric = sum([math.log10(v) for v in ue_tp.values()])
    jain_index = calc_jain_fairness([v for v in ue_tp.values()])

    ue_tp_mute = get_per_ue_throughput(dname_muting)
    pf_metric_mute = sum([math.log10(v) for v in ue_tp_mute.values()])
    jain_index_mute = calc_jain_fairness([v for v in ue_tp_mute.values()])

    macro_muted_rbs, micro_muted_rbs, macro_mean_gain, micro_mean_gain = get_percentage_gain_muting_freq_per_cell(dname_muting, num_micro)


    ofile = open('results.txt', 'a')
    ofile.write(f'{dname_muting} | Seed | {seed} | \n'
            + f'TP | {round(tp , 2)} | TP(mute)| {round(tp_mute , 2)} | '
            + f'%age Imp in TP | {round((((tp_mute - tp) / tp) * 100) , 2)} | \n'
            + f'PF Metric | {round(pf_metric , 2)} |  PF Metric(mute) | {round(pf_metric_mute , 2)} | '
            + f'%age Imp in PF Metric | {round(((pf_metric_mute - pf_metric) / abs(pf_metric)) * 100, 2)} | \n'
            + f'Jain | {round(jain_index , 2)} | Jain(mute) | {round(jain_index_mute , 2)} | '
            + f'%age Imp in Jain | {((jain_index_mute - jain_index) / jain_index) * 100} | \n'
            + f'Macro Muted RBS | {round(macro_muted_rbs , 2)} | Micro Muted RBS | {round(micro_muted_rbs , 2)} | '
            + f'Macro Mean Gain | {round(macro_mean_gain , 2)} | Micro Mean Gain | {round(micro_mean_gain , 2)} | \n'
            + f'************************************\n')
    ofile.close()

    ofile = open('results.csv', 'a')
    ofile.write(
        f'{dname_muting}, {seed}, {round(tp , 2)}, {round(tp_mute , 2)}, '
        f'{round((((tp_mute - tp) / tp) * 100) , 2)}, {round(pf_metric , 2)}, '
        f'{round(pf_metric_mute , 2)}, {round(((pf_metric_mute - pf_metric) / abs(pf_metric)) * 100, 2)}, '
        f'{round(macro_muted_rbs , 2)}, {round(micro_muted_rbs , 2)}, {round(macro_mean_gain , 2)}, {round(micro_mean_gain , 2)}\n'
        )
    ofile.close()