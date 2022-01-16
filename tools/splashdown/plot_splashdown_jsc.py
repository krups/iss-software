import os
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import shapely
import cartopy
from cartopy import config
import cartopy.crs as crs
from cartopy.mpl.gridliner import LONGITUDE_FORMATTER, LATITUDE_FORMATTER



li2200_data = np.array([[-36.69024,	-153.89555, 701],
[-36.59779,	-150.95408,2],
[-36.59699,	-156.36548,484],
[-36.59779,	-150.95408,2],
[-36.55005,	-150.97578,2],
[-36.56046,	-151.01488,2],
[-36.56883,	-150.95408,2],
[-36.55005,	-150.97578,2],
[-36.55005,	-150.97578,3],
[-36.55005,	-150.97578,3],
[-36.55005,	-150.97578,4],
[-36.55005,	-150.97578,5],
[-36.55005,	-150.97578,6],
[-36.55839,	-150.915,7],
[-36.56671,	-150.85418,7],
[-36.55005,	-150.97578,7]])

jsc_data = np.array([[-36.83201,	-150.64469,	3],
[-36.7843,	-150.66667,	2],
[-36.7843,	-150.66667,	2],
[-36.7843,	-150.66667,	2]])


fig = plt.figure()

ax = fig.add_subplot(1,1,1, projection=crs.PlateCarree())

ax.stock_img()
ax.coastlines()

# full view
#ax.set_extent([-158, -147, -30, -40],
#              crs=crs.PlateCarree()) ## Important

# zoomed view of final points
ax.set_extent([-151.4, -150.4, -36, -37],
              crs=crs.PlateCarree()) ## Important


for row in jsc_data:
  ax.tissot(rad_km=(row[2]), lons=[row[1]], lats=[row[0]], alpha=0.3, color='green', zorder=10)

plt.scatter(x=jsc_data[:,1], y=jsc_data[:,0], c=np.arange(0,jsc_data.shape[0]),
            cmap='jet',
            s=20,
            alpha=0.5,
            zorder=30,
            transform=crs.PlateCarree()) ## Important
            
clb = plt.colorbar(ax=ax, orientation="vertical", pad=.15)
clb.ax.tick_params(labelsize=8) 
clb.ax.set_title('Packet #',fontsize=8)


gl = ax.gridlines(crs=crs.PlateCarree(), draw_labels=True,
                  linewidth=2, color='gray', alpha=0.5, linestyle='--')
gl.xlines = True
gl.ylines = True


plt.title("AMTPS Capsule Packet Locations")



plt.show(block=False)

plot_title = "AMTPS Capsule Packet Locations"
plt.savefig("images/{}.png".format(plot_title),dpi=300,bbox_inches='tight')



