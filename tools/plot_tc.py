#!/usr/bin/env python3

# KREPE TC packet plotter
# Matt Ruffner 2021
# University of Kentucky
#
# Given a list of iridium packets 

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

from scipy import ndimage, misc

if len(sys.argv) < 2:
    print("Need input file(s) as arg.")
    sys.exit(1)

numInputs = len(sys.argv)-1

# check if input is compressed or uncompressed
for i in range(0, numInputs):
    outfile = sys.argv[i+1]+'.dat'
    cmdStr = os.getcwd()+'/bin/decompress ' + sys.argv[i+1] + ' ' + outfile;
    cmdOut = os.system(cmdStr)
    csvfile = sys.argv[i+1]+'.csv'
    cmdStr = os.getcwd()+'/bin/binlog_to_csv '+outfile+' > '+csvfile
    cmdOut = os.system(cmdStr)
    data = np.genfromtxt(csvfile, delimiter=',')
    data = np.delete(data, 0, 0)
    if i==0:
        X=data
    else:
        X=np.append(X, data, axis=0)

print(X)
    
print('read in ',X.shape,' rows')

X = X[np.argsort(X[:, 0])]

result = ndimage.median_filter(X[:,0:4], size=5)

plt.figure()
plt.plot(X[:,0]/1000/60, result, linestyle='--', marker='o')
plt.ylim([0,60])
plt.legend(['TC 1','TC 2','TC 3','TC 4'])
plt.xlabel('Time (minutes)')
plt.ylabel('Temperature (deg. C)')

print("Enter plot title:")
plot_title = input()

plt.title(plot_title)
plt.show()
