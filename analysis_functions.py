import statistics as stats
import matplotlib.pyplot as plt
from collections import defaultdict
import math
import numpy as np
import seaborn as sns
import itertools
import random

def get_per_ue_tbs(dname):
    all_tbs = defaultdict(lambda: [])
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('eff_sinr' in i):
            ue_id = int(spl[1])
            tbs = int(spl[11][:-1])/1000 # kbps -> Mbps
            all_tbs[ue_id].append(tbs)
    return all_tbs

def get_exp_duration(dname):
    file = open(dname, 'r')
    max_tti = 0
    while True:
        line = file.readline()
        if not line:
            break
        else:
            if 'TTI' in line \
                and 'Muting Decision' not in line \
                and 'SliceID' not in line:
                spl = line.split()
                max_tti = int(spl[-1])
    return max_tti - 100

def get_per_ue_throughput(dname):
    per_ue_tp = {}
    ue_tbs_comb = get_per_ue_tbs(dname)
    exp_duration = get_exp_duration(dname)
    for k,v in ue_tbs_comb.items():
        ue = k
        ue_data = sum(v)
        ue_tp = ue_data/exp_duration
        per_ue_tp[ue] = ue_tp # returns in mbps
    return per_ue_tp

def get_cumubytes(fname):
    end_ts = get_exp_duration(fname)
    ratio = 1 / 1000 # kbps -> Mbps
    cumu_bytes = defaultdict(lambda: 0)
    cumu_rbs = defaultdict(lambda: 0)
    with open(fname, "r") as fin:
        for line in fin:
            words = line.split(" ")
            if words[0] == "\t\tflow":
                slice_id = int(words[5])
                cumu_bytes[slice_id] += int(words[-1]) / end_ts * ratio
                cumu_rbs[slice_id] += int(words[7]) / end_ts
    return cumu_bytes, cumu_rbs

def get_throughput(dname):
    aggre_bws = []
    cumu_bytes, _ = get_cumubytes(dname)
    aggre_bw = 0
    for k, v in cumu_bytes.items():
        aggre_bw += v
    print(dname,":",aggre_bw)
    return aggre_bw

def get_ue_slice_comb(dname):
    ue_slice = {}
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('nb_of_rbs' in i):
            ue = int(spl[1])
            slice_id = int(spl[5])
            ue_slice[ue] = slice_id
    result = defaultdict(lambda: [])
    for k, v in ue_slice.items():
        result[v].append(k)
    return result


def analyze_per_slice_performance(dname):
    per_slice_tp = {}
    per_slice_pf = {}
    per_slice_tp, _ = get_cumubytes(dname)
    # print('Cumu Bytes: ')
    # print(cumu_bytes)
    # for k, v in cumu_bytes.items():
        # print('Slice: ', k , ' TP: ', v)
        # per_slice_tp.append(v)

    # print('Per Slice TP:')
    # print(per_slice_tp)
    per_ue_tp = get_per_ue_throughput(dname)
    ue_slice_comb = get_ue_slice_comb(dname)
    slice_tp = defaultdict(lambda:[])

    # Analyze PF Metric Per Slice:
    for ue, tp in per_ue_tp.items():
        slice_id = ue_slice_comb[ue]
        slice_tp[slice_id].append(tp)
    # print(len(slice_tp[0]))
    # print(len(slice_tp[1]))

    for slice_id, ue_tp_vals in slice_tp.items():
        sum_of_log_tp = sum([math.log10(v) for v in ue_tp_vals])
        # print('Slice ID: ', slice_id, ' PF Metric: ', sum_of_log_tp)
        per_slice_pf[slice_id] = sum_of_log_tp

    return per_slice_tp, per_slice_pf


def get_cell_coordinates():
    cells_coordinates = {}
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('Created Cell,' in i or 'Created Micro,' in i):
            cell_id = int(float(spl[3][:-1]))
            x = int(float(spl[5]))
            y = int(float(spl[6][:-1]))
            cells_coordinates[cell_id] = [x,y]

def get_ue_coordinates():
    ues_coordinates = {}
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('m_idNetworkNode' in i):
            ue_id = int(float(spl[3]))
            x = int(float(spl[12]))
            y = int(float(spl[15][:-1]))
            cell_id = int(spl[6])
            ues_coordinates[ue_id] = [cell_id, x, y]

def get_all_eff_sinr():
    all_eff_sinr = []
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('eff_sinr' in i):
            if(spl[9] == 'inf'):
                eff_sinr = 30
            else:
                eff_sinr = float(spl[9])
            all_eff_sinr.append(eff_sinr)
    return all_eff_sinr

def get_per_ue_eff_sinr():
    all_eff_sinr = defaultdict(lambda: [])
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('eff_sinr' in i):
            ue_id = int(spl[1])
            cell_id = int(spl[3])
            if(spl[9] == 'inf'):
                eff_sinr = 30
            else:
                eff_sinr = float(spl[9])
                all_eff_sinr[ue_id].append(eff_sinr)
    print(all_eff_sinr)
    return all_eff_sinr

def get_per_ue_cell_comb(dname):
    all_ue_cell = defaultdict(lambda:set([]))
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('eff_sinr' in i):
            ue_id = int(spl[1])
            cell_id = int(spl[3])
            all_ue_cell[cell_id].add(ue_id)
    return all_ue_cell


def get_all_tbs():
    all_tbs = []
    file = open(dname)
    for i in file.readlines():
        spl = i.split(' ')
        if ('eff_sinr' in i):
            tbs = int(spl[11][:-1])/1000 # kbps -> Mbps
            all_tbs.append(tbs)
    return all_tbs


def get_total_tbs(dname):
    all_tbs = 0
    file = open(dname, 'r')
    for i in file:
        spl = i.split(' ')
        if (spl[0] == '\t\tflow'):
            all_tbs += int(spl[-1])/1000 # kbps -> Mbps
    return all_tbs


def barplot_nvars(y, xlabel,ylabel,title,labels,ticks=None,colors = None):
    N = len(y[0])
    print(N)
    ind = np.arange(N)
    plt.figure(figsize=(30,6))
    width = 0.3
    for i in range(len(y)):
        if colors != None:
            plt.bar(ind, y[i], width, label=labels[i], color=colors[i])
        else:
            plt.bar(ind, y[i], width, label=labels[i])
        ind = ind + width
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)

    # xticks()
    # First argument - A list of positions at which ticks should be placed
    # Second argument -  A list of labels to place at the given locations
    ind = np.arange(N)
    if (ticks != None):
        plt.xticks(ind + width / 2, ticks, rotation=45)

    # Finding the best position for legends and putting it
    plt.legend(loc='best')
    plt.show()


def percentage_gain_per_cell(dname):
    file = open(dname, 'r')
    lines = file.readlines()
    file.close()
    gains = defaultdict(lambda:[])
    for line in lines:
        if 'RB ID:' in line:
            spl = line.split()
            gain = float(spl[-1])
            cell_id = int(spl[6])
            gains[cell_id].append(gain)
    for cell_id in gains:
        gains[cell_id] = stats.mean(gains[cell_id])
    return gains


def get_percentage_gain_muting_freq_per_cell(dname, num_micro):
    file = open(dname, 'r')
    lines = file.readlines()
    file.close()
    gains = defaultdict(lambda:0)
    frequency = defaultdict(lambda:0)
    for line in lines:
        if 'Muting Decision:' in line:
            # print(line)
            spl = line.split()
            gain = float(spl[-1])
            if (gain < 0.001):
                continue
            cell_id = int(spl[6])
            if (cell_id != 0):
                gains[1] += gain
                frequency[1] += 1
            else:
                gains[0] += gain
                frequency[0] += 1

    for cell_id in gains.keys():
        gains[cell_id] = gains[cell_id]/frequency[cell_id]
    duration = get_exp_duration(dname)
    total_rbs = duration * 125
    return ((frequency[0])/total_rbs) * 100, ((frequency[1]/num_micro)/total_rbs) * 100, gains[0]*100, gains[1]*100


def analyze_pf_trends(dname):
    file = open(dname, 'r')
    lines = file.readlines()
    file.close()
    pf_vals = defaultdict(lambda:{})
    muted_pf_vals = []
    muted_cell_id = -1

    for line in lines:
        if 'Testing Muting' in line:
            spl = line.split()
            muted_cell_id = int(spl[2])
        elif 'allocating to' in line:
            spl = line.split()
            cell_id = int(spl[1])
            instantaneous_pf = float(spl[7])
            historical_pf = float(spl[9])
            pf_metric = float(spl[12])
            pf_vals[muted_cell_id][cell_id] = tuple((instantaneous_pf, historical_pf, pf_metric))

        elif 'RB ID:' in line:
            spl = line.split()
            final_muted_id = int(spl[6])
            muted_pf_vals.append(pf_vals[final_muted_id])
            pf_vals = defaultdict(lambda:{})


    # print(muted_pf_vals)

    organized_muted_pf_vals = defaultdict(lambda:[])

    for kv_pair in muted_pf_vals:
        for k,v in kv_pair.items():
            organized_muted_pf_vals[k].append(v)


    print()
    macro_inst = []
    macro_hist = []
    macro_metric = []

    macro_values = organized_muted_pf_vals[0]
    for i in macro_values:
        macro_inst.append(i[0])
        macro_hist.append(i[1])
        macro_metric.append(i[2])        

    micro_inst = []
    micro_hist = []
    micro_metric = []

    micro_values = organized_muted_pf_vals[1]
    for i in micro_values:
        micro_inst.append(i[0])
        micro_hist.append(i[1])
        micro_metric.append(i[2])

    return macro_inst, macro_hist, macro_metric, micro_inst, micro_hist, micro_metric


def plot_analyze_pf_trends(macro_inst, macro_hist, macro_metric, micro_inst, micro_hist, micro_metric):


    # print('Macro Instantaneous PF: ', macro_inst)
    # print('Macro Historical PF: ', macro_hist)
    # print('Macro PF Metric: ', macro_metric)

    # print('Micro Instantaneous PF: ', micro_inst)
    # print('Micro Historical PF: ', micro_hist)
    # print('Micro PF Metric: ', micro_metric)

    # plot all these 6 arrays on a single plot
    sample_size = 1000
    print(sample_size)
    # randomly shuffle select 1000 values from each array
    macro_indices = sorted(random.sample(range(len(macro_inst)), sample_size))
    micro_indices = sorted(random.sample(range(len(micro_inst)), sample_size))
    
    macro_inst_subset = [macro_inst[i] for i in macro_indices]
    macro_hist_subset =[macro_hist[i] for i in macro_indices]
    macro_metric_subset =[macro_metric[i] for i in macro_indices]
    micro_inst_subset =[micro_inst[i] for i in micro_indices]
    micro_hist_subset =[micro_hist[i] for i in micro_indices]
    micro_metric_subset =[micro_metric[i] for i in micro_indices]

    plt.figure(figsize=(22, 6))
    plt.plot(macro_inst_subset, label='Macro Instantaneous TP')
    plt.plot(macro_hist_subset, label='Macro Historical TP')
    plt.plot(macro_metric_subset, label='Macro PF Metric')
    plt.plot(micro_inst_subset, label='Micro Instantaneous TP')
    plt.plot(micro_hist_subset, label='Micro Historical TP')
    plt.plot(micro_metric_subset, label='Micro PF Metric')
    plt.yscale('log')
    plt.xlabel('TTI')
    plt.ylabel('Log Scale')
    plt.legend()
    plt.title('PF Trends for Macro and Micro Cells - Optimizing for PF - 13 Cells') 
    plt.show()

def analyze_impact_of_muting_micro_on_macro(dname):
    file = open(dname, 'r')
    lines = file.readlines()
    file.close()

    macro_tp_with_muting = []
    macro_tp_without_muting = []

    for line in lines:
        if 'allocating to' in line:
            if 'Cell: 0' in line:
                if 'No_Mute:' in line:
                    spl = line.split()
                    macro_tp_without_muting.append(float(spl[8]))
                else:
                    spl = line.split()
                    macro_tp_with_muting.append(float(spl[7]))

    return macro_tp_with_muting, macro_tp_without_muting


def plot_analyze_impact_of_muting_micro_on_macro(macro_tp_with_muting, macro_tp_without_muting):
    plt.figure(figsize=(10, 6))
    muting_mean_tp = np.mean(macro_tp_with_muting)/1000
    no_muting_mean_tp = np.mean(macro_tp_without_muting)/1000
    print(muting_mean_tp, no_muting_mean_tp)
    bars = plt.bar(['Muting', 'No Muting'], [muting_mean_tp, no_muting_mean_tp])
    bars[0].set_color('g')
    bars[1].set_color('b')
    plt.ylabel('Mean Achieveable Data Rate (Mbps)')
    plt.legend()
    plt.title('Impact of Muting Micro Cells on Macro Cell UEs - Optimizing for PF - 7 Cells') 
    plt.show()

def calc_jain_fairness(data_array):
    numerator = np.square(np.sum(data_array))
    denominator = len(data_array) * np.sum(np.square(data_array))
    return numerator / denominator

def which_mute_effects_which_cell(dname):
    file = open(dname)
    lines = file.readlines()
    mute_cell_change_cell = defaultdict(lambda:defaultdict(lambda:0))
    for line in lines:
        if "Flow Changes for Cell" in line:
            spl = line.split()
            changed_cell = int(spl[4])
            muted_cell = int(spl[-1])
            mute_cell_change_cell[muted_cell[changed_cell]]+=1

