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


fig = plt.figure()

ax = fig.add_subplot(1,1,1, projection=crs.PlateCarree())

ax.stock_img()
ax.coastlines()
#ax.add_feature(cfeature.STATES)

# full view
ax.set_extent([-158, -147, -30, -40],
              crs=crs.PlateCarree()) ## Important

# zoomed view of final points
#ax.set_extent([-151.4, -150.4, -36, -37],
#              crs=crs.PlateCarree()) ## Important


for row in li2200_data:
  ax.tissot(rad_km=(row[2]), lons=[row[1]], lats=[row[0]], alpha=0.3, color='green', zorder=10)

#comap = plt.colorbar(cm.ScalarMappable(cmap="jet"), ax=ax)

plt.scatter(x=li2200_data[:,1], y=li2200_data[:,0], c=np.arange(0,li2200_data.shape[0]),
#            color="red",
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
#gl.xlocator = mticker.FixedLocator([-157, -156, -155, -154, -153, -152, -151, -150, -149])
#gl.xformatter = LONGITUDE_FORMATTER
#gl.yformatter = LATITUDE_FORMATTER
#gl.xlabel_style = {'size': 15, 'color': 'gray'}
#gl.xlabel_style = {'color': 'black', 'weight': 'bold'}

#plt.title("LI2200 Capsule Packet Locations")

def genCircle2(cx=0, cy=0, rad=1):
  """Generate points along perimeters of a circle"""
  points = []
  segs = 20
  for ea in range(segs+1):
    xi = cx + rad*np.cos(ea*2.*np.pi/segs)
    yi = cy + rad*np.sin(ea*2.*np.pi/segs)
    points.append([xi,yi])
  return np.array(points)

#cclpnts = genCircle2(cx=-153.89555, cy=-36.69024, rad=701000.)
#plt.scatter(x=cclpnts[:,0], y=cclpnts[:,1], color='blue', alpha=0.5, transform=crs.PlateCarree())






#ax.add_patch(Patch.Circle(xy=[-122.4015173428571, 37.78774634285715], radius = 0.021709041989311614 + 0.005, alpha=0.3, zorder=30, transform=ccrs.PlateCarree()))






plt.show(block=False)

plot_title = "LI2200 Capsule Packet Locations"
plt.savefig("images/{}.png".format(plot_title),dpi=300,bbox_inches='tight')

#plt.savefig("images/{}.png".format(plot_title),dpi=300,bbox_inches='tight')



