#!/usr/bin/env python3

# KREPE TC packet plotter
# Matt Ruffner 2021
# University of Kentucky
#
# Updated July 2024
#
# Given a list of iridium packets, decompress them and convert them to CSV files
# then read in those CSV files and perform a 5 point median filter on the data
# then plot for analysis and inspection of the data 

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import pdb
import seaborn as sns
import scipy as sp

import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import shapely
import cartopy
from cartopy import config
import cartopy.crs as crs
from cartopy.mpl.gridliner import LONGITUDE_FORMATTER, LATITUDE_FORMATTER



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

gpsData = []
pressureData = []
tcData = []
imuData = []
highGData = []
specData = []

for i in range(0, numInputs):
    
    print("Reading binary Iridium packet {}".format(sys.argv[i+1]))
    
    outfile = sys.argv[i+1]
    # cmdStr = os.getcwd()+'/bin/decompress ' + sys.argv[i+1] + ' ' + outfile;
    # cmdOut = os.system(cmdStr)
    
    #print(" output: {}".format(cmdOut))
    
    csvfile = sys.argv[i+1]+'.csv'
    cmdStr = os.getcwd()+'/parser/log_parser packet ' + outfile + " | sed '1,5 d' > " + csvfile
    cmdOut = os.system(cmdStr)
    print(cmdOut)
    #print(" -> converting to CSV and reading in\n   output: {}".format(cmdOut))
    
    with open(csvfile) as f:
       data = f.read()

    #print("read in {}".format(data))

    for line in data.split('\n'):
      print("line {}".format(line))
      lineData = line.split(',')
      if lineData[0] == '1': # GPS GGA
         gpsData.append({"t": float(lineData[1])*100, "UTC_time": lineData[2], "Lat": lineData[3], "Lon": lineData[4], "Alt": lineData[6]})
      if lineData[0] == '2': # GPS RMC
         gpsData.append({"t": float(lineData[1])*100, "UTC_time": lineData[2], "Lat": lineData[3], "Lon": lineData[4], "SOG": lineData[5]})
      if lineData[0] == '3': # high g accel
         highGData.append( [ float(i) for i in lineData[1:] ] )
      if lineData[0] == '4': # IMU 
         imuData.append( [ float(i) for i in lineData[1:] ] )
      if lineData[0] == '5': # TC 
         tcData.append( [ float(i) for i in lineData[1:] ] )
      if lineData[0] == '6': # Pressure 
         pressureData.append( [ float(i) for i in lineData[1:] ] )
      if lineData[0] == '7': # Spectrometer 
         specData.append( [float(lineData[1])*100] + [float(i) for i in lineData[2:]] )

    #pdb.set_trace()

    # if i==0:
    #     X=data
    #     packetDict[packetId] = sys.argv[i+1]
    # else:
    #     X=np.append(X, data, axis=0)
    #     packetDict[packetId] = sys.argv[i+1]
    
    # data = data[np.argsort(data[:, 0])]
    # print("   highest time index: {}".format(data[-3:,0]))
    
    #plt.figure()
    #plt.plot(data[:,0]/1000, data[:,1:], linestyle='--', marker='o')
    #plt.title(sys.argv[i+1])
    #plt.xlim([0,600])
    #plt.ylim([0, 1500])
    #plt.show(block=False)
    
    
    #packetVec.append(packetId)
    packetId += 1

#print(X)
    
#pdb.set_trace()


print(specData)


print("Opening figures")
####################
#     IMU Plot
####################
if len(imuData) > 0:
   imuData = np.array(imuData)
   imuData[imuData[:,0].argsort()]
   #imuData = np.sort(np.array(imuData),0)
   #a[a[:, 1].argsort()]
   IMU_acc_x = imuData[:,1]
   IMU_acc_y = imuData[:,2]
   IMU_acc_z = imuData[:,3]
   IMU_gyr_x = imuData[:,4]
   IMU_gyr_y = imuData[:,5]
   IMU_gyr_z = imuData[:,6]
   IMU_time = imuData[:,0]

   IMU_axs = []
   IMU_fig, IMU_axs = plt.subplots(2, 3, sharex=True)

   IMU_axs[0, 0].scatter(IMU_time, IMU_acc_x)
   IMU_axs[0, 0].set_title("Acc x")
   IMU_axs[0, 0].set_ylabel("m/s/s")

   IMU_axs[0, 1].scatter(IMU_time, IMU_acc_y, color="blue")
   IMU_axs[0, 1].sharey(IMU_axs[0, 0])
   IMU_axs[0, 1].set_title("Acc y")
   IMU_axs[0, 1].set_ylabel("m/s/s")

   IMU_axs[0, 2].scatter(IMU_time, IMU_acc_z, color="green")
   IMU_axs[0, 2].sharey(IMU_axs[0, 0])
   IMU_axs[0, 2].set_title("Acc z")
   IMU_axs[0, 2].set_ylabel("m/s/s")

   IMU_axs[1, 0].scatter(IMU_time, IMU_gyr_x, color="purple")
   IMU_axs[1, 0].set_title("Gyro x")
   IMU_axs[1, 0].set_ylabel("deg/sec")

   IMU_axs[1, 1].scatter(IMU_time, IMU_gyr_y, color="#8B8000")
   IMU_axs[1, 1].sharey(IMU_axs[1, 0])
   IMU_axs[1, 1].set_title("Gyro y")
   IMU_axs[1, 1].set_ylabel("deg/sec")

   IMU_axs[1, 2].scatter(IMU_time, IMU_gyr_z, color="#ADD8E6")
   IMU_axs[1, 2].sharey(IMU_axs[1, 0])
   IMU_axs[1, 2].set_title("Gyro z")
   IMU_axs[1, 2].set_ylabel("deg/sec")

   IMU_axs[1, 0].set_xlabel("Time (s)")
   IMU_axs[1, 1].set_xlabel("Time (s)")
   IMU_axs[1, 2].set_xlabel("Time (s)")

   IMU_fig.suptitle("IMU Plots")
   IMU_fig.tight_layout()

   print("here 1")
   #pdb.set_trace()

####################
#      TC Plot
####################
if len(tcData) > 0:
   tcData = np.array(tcData)
   tcData = tcData[tcData[:,0].argsort()]
   TC_1 = tcData[:,1]
   TC_2 = tcData[:,2]
   TC_3 = tcData[:,3]
   TC_4 = tcData[:,4]
   TC_5 = tcData[:,5]
   TC_6 = tcData[:,6]
   TC_time = tcData[:,0]

   kernel_size = 15

   TC_fig, TC_axs = plt.subplots()
   TC_axs.plot(TC_time, sp.signal.medfilt(TC_1, kernel_size), label="TC 1")
   TC_axs.plot(TC_time, sp.signal.medfilt(TC_2, kernel_size), label="TC 2")
   TC_axs.plot(TC_time, sp.signal.medfilt(TC_3, kernel_size), label="TC 3")
   TC_axs.plot(TC_time, sp.signal.medfilt(TC_4, kernel_size), label="TC 4")
   TC_axs.plot(TC_time, sp.signal.medfilt(TC_5, kernel_size), label="TC 5")
   TC_axs.plot(TC_time, sp.signal.medfilt(TC_6, kernel_size), label="TC 6")
   TC_axs.get_yaxis().set_visible(False)
   TC_axs.legend()
   TC_axs.set_xlabel("Time (seconds)")
   TC_axs.set_ylabel("Temperature")
   TC_fig.suptitle("TC Plots")

   print("here2")



# ####################
# #     GGA Plot
# ####################
# # px.set_mapbox_access_token(
# #     "pk.eyJ1IjoiZm9sa2lzaGFjb3JuIiwiYSI6ImNsM2xxNWY3dTAzdDkzY284Z2dnbWZlYm0ifQ.gtXNkZAtJCFpu8Ir1m4kpw")
# # GGA_fig = px.scatter_mapbox(GGA_df, lat="Val_2", lon="Val_3", color="Time", size=[x+1 for x in range(len(GGA_df))],
# #                             color_continuous_scale=px.colors.sequential.OrRd, zoom=17, mapbox_style="satellite", title="GGA Coordinates Plot")
# # GGA_fig.show()

# GGA_fig = plt.figure()
# ax = GGA_fig.add_subplot(1,1,1, projection=crs.PlateCarree())
# ax.stock_img()
# ax.coastlines()

# # full view
# ax.set_extent([-158, -147, -30, -40],
#               crs=crs.PlateCarree()) ## Important

# # zoomed view of final points
# #ax.set_extent([-150.7, -150.55, -36.75, -36.9],
# #              crs=crs.PlateCarree()) ## Important


# #for row in jsc_data:
# #  ax.tissot(rad_km=(row[2]), lons=[row[1]], lats=[row[0]], alpha=0.3, color='green', zorder=10)

# print("here3")

# GGA_fig.scatter(x=[i["Lon"] for i in gpsData], y=[i["Lat"] for i in gpsData],
#             cmap='jet',
#             s=20,
#             alpha=0.5,
#             zorder=30,
#             transform=crs.PlateCarree()) ## Important
            
# # clb = plt.colorbar(ax=ax, orientation="vertical", pad=.15)
# # clb.ax.tick_params(labelsize=8) 
# # clb.ax.set_title('Packet #',fontsize=8)

# print("here4")
# gl = ax.gridlines(crs=crs.PlateCarree(), draw_labels=True,
#                   linewidth=2, color='gray', alpha=0.5, linestyle='--')
# gl.xlines = True
# gl.ylines = True

# GGA_fig.title("GPS positions")
# GGA_fig.show(block=False)

# print("here5")



# ####################
# #     RMC Plot
# ####################
# px.set_mapbox_access_token(
#     "pk.eyJ1IjoiZm9sa2lzaGFjb3JuIiwiYSI6ImNsM2xxNWY3dTAzdDkzY284Z2dnbWZlYm0ifQ.gtXNkZAtJCFpu8Ir1m4kpw")
# RMC_fig = px.scatter_mapbox(RMC_df, lat="Val_2", lon="Val_3", color="Time", size=[x+1 for x in range(len(GGA_df))],
#                             color_continuous_scale=px.colors.sequential.OrRd, zoom=17, mapbox_style="satellite", title="RMC Coordinates Plot")
# RMC_fig.show()


####################
#     ACC Plot
####################
if len(highGData) > 0:
   highGData = np.array(highGData)
   highGData = highGData[highGData[:,0].argsort()]
   
   ACC_x = highGData[:,1]
   ACC_y =  highGData[:,2]
   ACC_z =  highGData[:,3]
   ACC_time =  highGData[:,0]

   ACC_axs = []
   ACC_fig, ACC_axs = plt.subplots(3, sharex=True, sharey=True)

   ACC_axs[0].plot(ACC_time, ACC_x)
   ACC_axs[0].set_ylabel("X axis (g)")

   ACC_axs[1].plot(ACC_time, ACC_y, color="blue")
   ACC_axs[1].set_ylabel("Y axis (g)")

   ACC_axs[2].plot(ACC_time, ACC_z, color="green")
   ACC_axs[2].set_ylabel("Z axis (g)")
   ACC_axs[2].set_xlabel("Time (s)")

   ACC_fig.suptitle("High G importAccelerometer Plots")

####################
#    SPEC Plot
####################
if len(specData) > 0:
  specData = np.array(specData)
  specData = specData[specData[:,0].argsort()]
  SPEC_time = np.round(specData[:,0], decimals=2)
  SPEC_vals = specData[:,2:]
  binLabels = ["340-496nm", "497-568nm", "569-640nm", "641-712nm", "713-784nm", "785-856nm" ]
  plt.figure()
  SPEC_axs = sns.heatmap(SPEC_vals.T, linewidth=0.0, yticklabels=binLabels, xticklabels=SPEC_time)
  for ind, label in enumerate(SPEC_axs.get_xticklabels()):
    if ind % 20 == 0:  # every 10th label is kept
      label.set_visible(True)
    else:
      label.set_visible(False)
  plt.title("Spectrometer")
  plt.show(block=False)


# SPEC_fig, SPEC_axs = plt.subplots()
# SPEC_axs.set_ylim([0, 1])
# SPEC_axs.plot(SPEC_time, SPEC_1, '--bo', label="Spectrometer 1")
# SPEC_axs.plot(SPEC_time, SPEC_2, '--ro', label="Spectrometer 2")
# SPEC_axs.legend()

# SPEC_axs.set_xlabel("Time (s)")
# SPEC_axs.set_ylabel("Red light to blue light ratio")
# SPEC_fig.suptitle("Spectrometer Ratio Plots")



####################
#    Pressure Plot
####################
if len(pressureData) > 0:
   pressureData = np.array(pressureData)
   pressureData = pressureData[pressureData[:,0].argsort()]
   plt.figure()
   kernel_size = 5
   plt.plot(pressureData[:,0], sp.signal.medfilt(pressureData[:,1],kernel_size), linestyle='--')
   plt.plot(pressureData[:,0], sp.signal.medfilt(pressureData[:,2],kernel_size), linestyle='--')
   plt.plot(pressureData[:,0], sp.signal.medfilt(pressureData[:,3],kernel_size), linestyle='--')
   plt.plot(pressureData[:,0], sp.signal.medfilt(pressureData[:,4],kernel_size), linestyle='--')
   plt.plot(pressureData[:,0], sp.signal.medfilt(pressureData[:,5],kernel_size), linestyle='--')
   plt.legend(['Prs 1','Prs 2','Prs 3','Prs 4','Prs 5'])
   plt.xlabel('Time (seconds)')
   plt.ylabel('Pressure (kPa)')
   plt.show(block=False)

endd = input()

# if noaxislabel:
#   frame1=plt.gca()
#   frame1.axes.get_xaxis().set_visible(False)
#   frame1.axes.get_yaxis().set_visible(False)
# else:
#   print("Enter plot title:")
#   plot_title = input()
#   plt.title(plot_title)
  
# plt.show(block=False)


# # plt.title("KREPE Re-entry heating data (LI2200 heatshield")
# # np.savetxt('testdata/li2200/results/profile.csv', X, delimiter=',')
# plt.savefig("images/{}.png".format(plot_title),dpi=300,bbox_inches='tight')






