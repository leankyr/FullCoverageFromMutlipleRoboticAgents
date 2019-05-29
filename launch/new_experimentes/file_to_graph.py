#!/usr/bin/env python

from __future__ import print_function
#from collections import Counter

import matplotlib.pyplot as plt
import numpy as np

import datetime
import time


if __name__ == '__main__':

    # date = '0.012 Mon Apr  28 22:23:15 2019'
    with open("one_robot/rand_target/exp1_med_map.txt") as file_obj:
        values = file_obj.readlines()

# input vector to plot
    plot_vec = list()
    for i in range(len(values)):
        value_split = values[i].split()
        plot_vec.append(value_split)

    #se perate the values
    cov = list()
    stamp = list()
    for value in plot_vec:
        cov.append(float(value[0])) 
        stamp.append(float(value[1])) 
    
    # keep unique values  
    stamp = np.unique(stamp, return_index = True)
    # keep the related coverage values
    covn = list()
    for i in stamp[1]:
        covn.append(cov[i])

    #print (covn)
    #print (stamp[0])
    

    # Plot the data
    plt.plot(stamp[0], covn, linewidth=2.0)

    # Add a legend
    plt.legend()

    # Show the plot
    plt.show()

