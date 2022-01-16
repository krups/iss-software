#!/usr/bin/env python3

# KREPE TC packet plotter
# Matt Ruffner 2021
# University of Kentucky
#
# Given a list of iridium packets, decompress them and convert them to CSV files
# then read in those CSV files and perform a 5 point median filter on the data
# then plot for analysis and inspection of the data 

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

from scipy import ndimage, misc

if len(sys.argv) < 2:
    print("Need input file(s) as arg, need to be compressed iridium packets.")
    sys.exit(1)

noaxislabel = False

numInputs = len(sys.argv)-1

# iterate over input files which are assumed to be compressed iridium packets
# containing data as described in src/packets.h
# uses tools which are assumed to be built (decompress and binlog_to_csv)
packetDict = {}
packetVec = []
packetId = 0
for i in range(0, numInputs):
    
    print("decompressing dat file {}".format(sys.argv[i+1]))
    
    outfile = sys.argv[i+1]+'.dat'
    cmdStr = os.getcwd()+'/bin/decompress ' + sys.argv[i+1] + ' ' + outfile;
    cmdOut = os.system(cmdStr)
    
    #print(" output: {}".format(cmdOut))
    
    csvfile = sys.argv[i+1]+'.csv'
    cmdStr = os.getcwd()+'/bin/binlog_to_csv '+outfile+' > '+csvfile
    cmdOut = os.system(cmdStr)
    
    #print(" -> converting to CSV and reading in\n   output: {}".format(cmdOut))
    
    data = np.genfromtxt(csvfile, delimiter=',')
    data = np.delete(data, 0, 0)
    if i==0:
        X=data
        packetDict[packetId] = sys.argv[i+1]
    else:
        X=np.append(X, data, axis=0)
        packetDict[packetId] = sys.argv[i+1]
    
    data = data[np.argsort(data[:, 0])]
    print("   highest time index: {}".format(data[-3:,0]))
    
    #plt.figure()
    #plt.plot(data[:,0]/1000, data[:,1:], linestyle='--', marker='o')
    #plt.title(sys.argv[i+1])
    #plt.xlim([0,600])
    #plt.ylim([0, 1500])
    #plt.show(block=False)
    
    
    packetVec.append(packetId)
    packetId += 1

#print(X)
    
print('read in ',X.shape,' rows')

X = X[np.argsort(X[:, 0])]


X=X[0:-2,:]

#result = ndimage.median_filter(X[:,0:4], size=5)
result = X[:,1:]

plt.figure()
plt.plot(X[:,0]/1000, result, linestyle='--', marker='o')
plt.legend(['TC 1','TC 2','TC 3','TC 4'])
plt.xlabel('Time (seconds)')
plt.ylabel('Temperature (deg. C)')

if noaxislabel:
  frame1=plt.gca()
  frame1.axes.get_xaxis().set_visible(False)
  frame1.axes.get_yaxis().set_visible(False)
else:
  print("Enter plot title:")
  plot_title = input()
  plt.title(plot_title)
  
plt.show(block=False)


# plt.title("KREPE Re-entry heating data (LI2200 heatshield")
# np.savetxt('testdata/li2200/results/profile.csv', X, delimiter=',')
plt.savefig("images/{}.png".format(plot_title),dpi=300,bbox_inches='tight')






