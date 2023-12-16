#!/usr/bin/python3
TIMES = 1
RADIUS = 1000
import sys
import matplotlib.pyplot as plt

def plot_geo_loc(dname):
    # Extract Cell Coordinates
    for i in range (TIMES):
        file = open(dname)
        ues = []
        cells = []
        micro = []
        print(dname)

        for i in file.readlines():
            if ('Created UE' in i):
                spl = i.split(' ')
                ues.append((int(float(spl[4])),int(float(spl[6])),int(float(spl[7][:-1])), int(spl[9][:-1]))) # ID, X, Y, Cell ID
            
            elif ('Created Cell,' in i):
                spl = i.split(' ')
                cells.append((int(float(spl[3][:-1])),int(float(spl[5])),int(float(spl[6][:-1])))) # ID, X, Y
            
            elif ('Created Micro,' in i):
                spl = i.split(' ')
                # print(spl)
                micro.append((int(float(spl[3][:-1])),int(float(spl[5])),int(float(spl[6])))) # ID, X, Y

        cells_zipped = list(zip(*cells))
        ues_zipped = list(zip(*ues))
        micro_zipped = list(zip(*micro))
        print(ues_zipped)
        print(micro_zipped)
        print(cells_zipped)

        plt.figure(figsize=(20,12))
        # plt.xticks(range(min(ues_zipped[1]), 100, max(ues_zipped[1])))
        # plt.yticks(range(min(ues_zipped[2]), 100, max(ues_zipped[2])))

        markers = ['x', 'o', 'd', 'p', 'h', 'v', 'o']
        colors = ['red', 'blue', 'green', 'orange', 'magenta', 'maroon', 'turquoise', 'darkgreen', 'tan', 'darksalmon', 'purple', 'ivory', 'yellowgreen', 'slategray']

        for h,i,j,k in zip(ues_zipped[0],ues_zipped[1],ues_zipped[2],ues_zipped[3]): # ID, X, Y, Cell
            # print(h, '\t', i, '\t', j, '\t', k)
            plt.scatter(i, j, marker=markers[k%7], color = colors[k])
            plt.text(i, j, str(h), color = colors[k], fontsize=12)

        if (len(cells_zipped) > 0):
            for i,j,k in zip(cells_zipped[1],cells_zipped[2],cells_zipped[0]):
                plt.scatter(i, j, marker=markers[k%7], color = colors[k])
                circle1 = plt.Circle((i,j), RADIUS, facecolor='none', edgecolor=colors[k])
                plt.gca().add_patch(circle1)


        if (len(micro_zipped) > 0):
            for i,j,k in zip(micro_zipped[1],micro_zipped[2],micro_zipped[0]):
                plt.scatter(i, j, marker=markers[k%7], color = colors[k])
                circle1 = plt.Circle((i,j), RADIUS/4, facecolor='none', edgecolor=colors[k])
                plt.gca().add_patch(circle1)

        plt.title(dname, fontsize = 18)
        # plt.margins(x=0)
        plt.savefig(dname[:-4] + ".png" ,bbox_inches='tight')
        plt.figure(figsize=(20,12))



def plot_geo_loc_start(dname, filtered_ues = None):
    # Extract Cell Coordinates
    for i in range (TIMES):
        file = open(dname)
        ues_start = []
        ues_end = []
        cells = []
        micro = []

        for i in file.readlines():
            if ('m_idNetworkNode' in i):
                spl = i.split(' ')
                ues_start.append((int(float(spl[3])),int(float(spl[12])), int(float(spl[15][:-1])), int(float(spl[6])))) # ID, X, Y, Cell ID

            elif ('Created Cell,' in i):
                spl = i.split(' ')
                cells.append((int(float(spl[3][:-1])),int(float(spl[5])),int(float(spl[6][:-1])))) #ID, X, Y
            
            elif ('Created Micro,' in i):
                spl = i.split(' ')
                micro.append((int(float(spl[3][:-1])),int(float(spl[5])),int(float(spl[6][:-1]))))

        cells_zipped = list(zip(*cells))
        ues_start_zipped = list(zip(*ues_start))
        ues_end_zipped = list(zip(*ues_end))
        micro_zipped = list(zip(*micro))

        plt.figure(figsize=(20,12))

        markers = ['x', 'o', 'd', 'p', 'h', 'v', 'o']
        colors = ['red', 'blue', 'green', 'orange', 'magenta', 'maroon', 'turquoise', 'darkgreen', 'royalblue','aqua' ]

        for h,i,j,k in zip(ues_start_zipped[0],ues_start_zipped[1],ues_start_zipped[2],ues_start_zipped[3]): # ID, X, Y, Cell
            if (filtered_ues != None):
                if (h in filtered_ues):
                    plt.scatter(i, j, marker=markers[k%7], color= colors[k])
                    plt.text(i, j, str(h), color= colors[k], fontsize=22)
            else:
                plt.scatter(i, j, marker=markers[k%7], color= colors[k])
                plt.text(i, j, str(h), color= colors[k], fontsize=22)

        if (len(cells_zipped) > 0):
            for i,j,k in zip(cells_zipped[1],cells_zipped[2],cells_zipped[0]): # X, Y, ID
                plt.scatter(i, j, marker=markers[k%7], color= colors[k])
                circle1 = plt.Circle((i,j), RADIUS, facecolor='none', edgecolor=colors[k])
                plt.gca().add_patch(circle1)

        if (len(micro_zipped) > 0):
            for i,j,k in zip(micro_zipped[1],micro_zipped[2],micro_zipped[0]): # X, Y, ID
                plt.scatter(i, j, marker=markers[k%7], color= colors[k])
                circle1 = plt.Circle((i,j), RADIUS/4, facecolor='none', edgecolor=colors[k])
                plt.gca().add_patch(circle1)

        plt.title(dname, fontsize = 18)
        # plt.margins(x=0)
        plt.savefig(dname[:-4] + ".png" ,bbox_inches='tight')
        plt.figure(figsize=(20,12))

file_name = sys.argv[1]
filtered_ues = []
if (len(sys.argv)>2):
    # add all the remaining arguments to filetered ues
    for i in range(2, len(sys.argv)):
        filtered_ues.append(int(sys.argv[i]))
    save_name = file_name[:-4] + "_filtered.png"
    plot_geo_loc_start(file_name, filtered_ues)
else:
    plot_geo_loc_start(file_name)
